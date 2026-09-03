#!/usr/bin/env python3
"""编码器状态节点：节点内判断「正常 / 落地 / 未落地」，持续发布当前状态。

与 reference/encoder_driver 的区别：
    - encoder_driver 持续发布 encoder_angle/encoder_raw，只读不判断；
    - 本节点复用同一串口协议（ID CMD Len Data CRC，Modbus RTU 读保持寄存器），
      内部低频轮询维护角度环缓冲并做落地判断，**持续发布**当前状态到
      /grasp_hexapod/encoder_state（grasp_hexapod_msgs/EncoderState：
      normal / landed / not_landed / angle / reason），供行为树
      is_landing_confirmed / sensor_health 订阅。

三态判据（对齐空地协同时序图 ⑩/㉔：未接触值固定 -> 接触后突变）：
    normal      串口帧 CRC 合法且最近 stale_timeout_s 内有有效读数；
    not_landed  未检测到突变（窗口内值固定或尚未稳定基线）；
    landed      相对稳定基线出现 |Δ| > jump_threshold 的突变（检测后锁定）。

用法：
    rosrun encoder_driver encoder_status_node.py _port:=/dev/ttyUSB0
    python3 encoder_status_node.py --selftest     # 离线自检（假串口，不依赖 ROS）
"""

import argparse
import time
from collections import deque

# 编码器参数（与 encoder_driver/encoder_frame.hpp 一致）
RESOLUTION = 262144.0     # 18 位分辨率 2**18
FULL_CIRCLE = 360.0
CMD_READ = 0x03


# --------------------------------------------------------------------------
# Modbus RTU 帧协议（与 reference/encoder_driver 逐字节兼容）
# --------------------------------------------------------------------------
def modbus_crc16(data):
    """Modbus CRC16（多项式 0xA001，低字节在前）。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def append_crc(frame):
    """帧末追加两字节 CRC（低字节在前）。"""
    crc = modbus_crc16(frame)
    return bytes(frame) + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def check_crc(frame):
    """校验整帧（含末尾两字节 CRC）。"""
    if len(frame) < 4:
        return False
    crc = modbus_crc16(frame[:-2])
    return frame[-2] == (crc & 0xFF) and frame[-1] == ((crc >> 8) & 0xFF)


def build_read_query(slave_id=0x00, start_reg=0, reg_count=2):
    """构建读请求帧 [ID][0x03][start_hi][start_lo][count_hi][count_lo][CRC]。"""
    return append_crc([
        slave_id & 0xFF, CMD_READ,
        (start_reg >> 8) & 0xFF, start_reg & 0xFF,
        (reg_count >> 8) & 0xFF, reg_count & 0xFF,
    ])


def parse_response(frame):
    """解析响应帧；不合法抛 ValueError。

    帧结构：[ID][CMD][Len][Data(Len)][CRClo][CRChi]，Data 大端组合为原始计数。
    """
    if len(frame) < 5:
        raise ValueError("帧过短: {}".format(frame.hex()))
    if not check_crc(frame):
        raise ValueError("CRC 校验失败: {}".format(frame.hex()))
    slave_id, cmd, length = frame[0], frame[1], frame[2]
    if len(frame) != 3 + length + 2:
        raise ValueError("长度不匹配: {}".format(frame.hex()))
    if cmd != CMD_READ:
        raise ValueError("非读响应: cmd=0x{:02X}".format(cmd))
    raw = 0
    for byte in frame[3:3 + length]:
        raw = (raw << 8) | byte
    return {"slave_id": slave_id, "cmd": cmd, "raw": raw}


def raw_to_angle(raw, resolution=RESOLUTION, full_circle=FULL_CIRCLE):
    """原始计数值 -> 角度（deg）。"""
    return raw / resolution * full_circle


def build_response_frame(raw, slave_id=0x00, data_len=None):
    """按协议构造一帧响应（自检/假串口用）。data_len 可指定数据字节数。"""
    if data_len is None:
        data_len = max(1, (raw.bit_length() + 7) // 8)
    data = list(raw.to_bytes(data_len, "big"))
    frame = [slave_id, CMD_READ, data_len] + data
    return append_crc(frame)


# --------------------------------------------------------------------------
# 串口链路（真实 / 假）
# --------------------------------------------------------------------------
class SerialLink:
    """pyserial 封装：写请求、按超时读一帧。"""

    def __init__(self, port, baudrate):
        import serial  # 延迟导入，离线自检不需要
        self.serial = serial.Serial(port, baudrate, timeout=0.1)

    def query(self, query_bytes, timeout_s):
        self.serial.write(query_bytes)
        self.serial.timeout = timeout_s
        chunk = self.serial.read(64)
        return self._extract_frames(chunk)

    @staticmethod
    def _extract_frames(chunk):
        """从字节流中按 'ID CMD Len Data CRC' 扫描出合法帧（容忍分片/粘包/杂质）。"""
        frames = []
        i = 0
        while i + 3 <= len(chunk):
            length = chunk[i + 2]
            total = 3 + length + 2
            if i + total > len(chunk):
                break
            frame = chunk[i:i + total]
            try:
                parse_response(frame)
                frames.append(frame)
                i += total
            except ValueError:
                i += 1
        return frames

    def close(self):
        self.serial.close()


class FakeSerialLink:
    """假串口：按脚本依次返回响应帧（或 None 表示超时）。"""

    def __init__(self, script, slave_id=0x00):
        # script: [(t, raw) | (t, None)]，t 为产生该响应的模拟时刻
        self.script = list(script)
        self.slave_id = slave_id
        self.now = 0.0
        self.sent = []

    def query(self, query_bytes, timeout_s):
        self.now += timeout_s
        self.sent.append(query_bytes)
        while self.script and self.script[0][0] <= self.now:
            _, raw = self.script.pop(0)
            if raw is None:
                return []
            return [build_response_frame(raw, self.slave_id)]
        return []

    def close(self):
        pass


# --------------------------------------------------------------------------
# 落地判断（纯逻辑，可离线测试）
# --------------------------------------------------------------------------
class EncoderLandingJudge:
    """角度环缓冲 + 突变检测三态判断。

    feed() 仅在收到有效帧时调用；judge() 随时可查询三态。
    """

    def __init__(self, jump_threshold_deg=5.0, window_size=20,
                 min_baseline=3, stabilize_samples=5, stale_timeout_s=1.0):
        if jump_threshold_deg <= 0 or window_size < 2:
            raise ValueError("非法参数")
        self.jump_threshold = jump_threshold_deg
        self.window = deque(maxlen=window_size)     # 突变前基线
        self.post_jump = deque(maxlen=max(2, stabilize_samples))  # 突变后近段样本
        self.min_baseline = min_baseline
        self.stale_timeout_s = stale_timeout_s
        self.landed = False
        self.jump_detected = False
        self.last_angle = None
        self.last_read_time = None

    def feed(self, angle, t):
        """收到有效角度样本（deg）时调用。"""
        if not self.landed and not self.jump_detected:
            if len(self.window) >= self.min_baseline:
                baseline = sum(self.window) / len(self.window)
                # 基线需稳定（窗口跨度小于阈值）才算有效参照
                spread = max(self.window) - min(self.window)
                if spread < self.jump_threshold and abs(angle - baseline) > self.jump_threshold:
                    self.jump_detected = True
        if self.jump_detected:
            self.post_jump.append(angle)
        self.window.append(angle)
        self.last_angle = angle
        self.last_read_time = t

    def _stabilized_after_jump(self):
        """突变后近段样本重新固定（跨度小于阈值）即确认落地。"""
        if len(self.post_jump) < 2:
            return False
        spread = max(self.post_jump) - min(self.post_jump)
        return spread < self.jump_threshold

    def judge(self, now):
        """查询三态。返回 dict(normal, landed, not_landed, angle, reason)。"""
        if self.last_read_time is None or now - self.last_read_time > self.stale_timeout_s:
            return {"normal": False, "landed": False, "not_landed": False,
                    "angle": self.last_angle if self.last_angle is not None else 0.0,
                    "reason": "串口无数据"}
        if self.jump_detected and not self.landed and self._stabilized_after_jump():
            self.landed = True
        if self.landed:
            state = {"normal": True, "landed": True, "not_landed": False,
                     "angle": self.last_angle, "reason": "已检测到接触突变"}
        elif self.window:
            spread = max(self.window) - min(self.window)
            reason = "值固定未突变" if spread < self.jump_threshold else "值波动未检测到突变"
            state = {"normal": True, "landed": False, "not_landed": True,
                     "angle": self.last_angle, "reason": reason}
        else:
            state = {"normal": True, "landed": False, "not_landed": True,
                     "angle": self.last_angle, "reason": "样本不足"}
        return state


# --------------------------------------------------------------------------
# ROS 节点（内部低频轮询，持续发布当前状态）
# --------------------------------------------------------------------------
def run_node(args):
    import rospy
    from grasp_hexapod_msgs.msg import EncoderState

    rospy.init_node("encoder_status")
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baudrate = rospy.get_param("~baud", 115200)
    slave_id = int(rospy.get_param("~slave_id", 0))
    start_reg = int(rospy.get_param("~start_reg", 0))
    reg_count = int(rospy.get_param("~reg_count", 2))
    poll_hz = float(rospy.get_param("~poll_hz", 10.0))
    state_topic = rospy.get_param(
        "~state_topic", "/grasp_hexapod/encoder_state")
    jump_threshold = float(rospy.get_param("~jump_threshold", 5.0))
    window_size = int(rospy.get_param("~window_size", 20))
    stale_timeout = float(rospy.get_param("~stale_timeout_s", 1.0))

    judge = EncoderLandingJudge(jump_threshold_deg=jump_threshold,
                                window_size=window_size,
                                stale_timeout_s=stale_timeout)
    query = build_read_query(slave_id, start_reg, reg_count)
    link = SerialLink(port, baudrate)
    pub = rospy.Publisher(state_topic, EncoderState, queue_size=1)
    lock = __import__("threading").Lock()

    def poll(_event):
        frames = link.query(query, timeout_s=1.0 / poll_hz)
        with lock:
            for frame in frames:
                info = parse_response(frame)
                judge.feed(raw_to_angle(info["raw"]), rospy.get_time())
            state = judge.judge(rospy.get_time())
        msg = EncoderState()
        msg.header.stamp = rospy.Time.now()
        msg.normal = bool(state["normal"])
        msg.landed = bool(state["landed"])
        msg.not_landed = bool(state["not_landed"])
        msg.angle = state["angle"]
        msg.reason = state["reason"]
        pub.publish(msg)

    rospy.Timer(rospy.Duration(1.0 / poll_hz), poll)
    rospy.loginfo("编码器状态节点就绪: %s@%d slave=0x%02X 跳变阈值=%.1fdeg "
                  "（持续发布 %s，%gHz）",
                  port, baudrate, slave_id, jump_threshold,
                  state_topic, poll_hz)
    rospy.on_shutdown(link.close)
    rospy.spin()


# --------------------------------------------------------------------------
# 离线自检
# --------------------------------------------------------------------------
def selftest():
    # --- 1. 协议往返：CRC 向量 + 请求/响应解析 ---
    assert modbus_crc16(b"123456789") == 0x4B37, "CRC16 标准向量不符"
    query = build_read_query(0x00, 0, 2)
    assert len(query) == 8 and check_crc(query), "读请求帧构造错误"
    raw = 0x03FF  # 任意计数
    frame = build_response_frame(raw, 0x00)
    info = parse_response(frame)
    assert info["raw"] == raw, "响应解析 raw 不符"
    assert abs(raw_to_angle(raw) - raw / RESOLUTION * FULL_CIRCLE) < 1e-12
    bad = bytearray(frame)
    bad[0] ^= 0xFF
    try:
        parse_response(bytes(bad))
        raise AssertionError("CRC 损坏帧未被拒绝")
    except ValueError:
        pass
    print("[OK] 协议往返: CRC/请求/响应/损坏帧拒绝")

    # --- 2. 三态判断：值固定 -> 未落地 ---
    judge = EncoderLandingJudge(jump_threshold_deg=5.0)
    for i in range(20):
        judge.feed(120.0 + 0.1 * (i % 2), t=0.1 * i)
    state = judge.judge(1.0)
    assert state["normal"] and state["not_landed"] and not state["landed"], state
    print("[OK] 值固定: {} reason={}".format(
        "not_landed" if state["not_landed"] else "?", state["reason"]))

    # --- 3. 三态判断：突变 -> 稳定 -> 落地（锁定） ---
    for i in range(10):
        judge.feed(120.0, t=2.0 + 0.1 * i)
    judge.feed(150.0, t=3.1)   # 突变 +30deg
    for i in range(5):
        judge.feed(150.0, t=3.2 + 0.1 * i)
    state = judge.judge(4.0)
    assert state["normal"] and state["landed"] and not state["not_landed"], state
    for i in range(5):  # 突变后继续读到同值，仍锁定落地
        judge.feed(150.0, t=4.1 + 0.1 * i)
    assert judge.judge(5.0)["landed"]
    print("[OK] 突变落地锁定: reason={}".format(state["reason"]))

    # --- 4. 串口无数据 -> normal=false ---
    judge2 = EncoderLandingJudge(stale_timeout_s=1.0)
    judge2.feed(100.0, t=0.0)
    state = judge2.judge(now=5.0)
    assert not state["normal"] and "无数据" in state["reason"], state
    print("[OK] 串口无数据: reason={}".format(state["reason"]))

    # --- 5. 假串口端到端：固定值流 -> 服务三态未落地 ---
    script = [(0.1 * i, 80000) for i in range(30)]
    link = FakeSerialLink(script)
    judge3 = EncoderLandingJudge()
    t = 0.0
    for _ in range(10):
        frames = link.query(build_read_query(0, 0, 2), timeout_s=0.1)
        t += 0.1
        for frame in frames:
            judge3.feed(raw_to_angle(parse_response(frame)["raw"]), t)
    state = judge3.judge(t)
    assert state["normal"] and state["not_landed"], state
    # 追加突变段（先清掉预置脚本的剩余条目）
    link.script.clear()
    for i in range(10):
        link.script.append((t + 0.1, 100000))
    for _ in range(10):
        frames = link.query(build_read_query(0, 0, 2), timeout_s=0.1)
        t += 0.1
        for frame in frames:
            judge3.feed(raw_to_angle(parse_response(frame)["raw"]), t)
    assert judge3.judge(t)["landed"], "假串口突变后应确认落地"
    print("[OK] 假串口端到端: 固定值未落地 -> 突变后落地")

    print("selftest 全部通过")


def main():
    parser = argparse.ArgumentParser(description="编码器状态节点（三态判断，持续发布 /grasp_hexapod/encoder_state）")
    parser.add_argument("--selftest", action="store_true", help="离线自检（不依赖 ROS）")
    args, ros_args = parser.parse_known_args()

    if args.selftest:
        selftest()
        return

    run_node(args)


if __name__ == "__main__":
    main()
