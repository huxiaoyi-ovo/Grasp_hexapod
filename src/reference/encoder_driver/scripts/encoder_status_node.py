#!/usr/bin/env python3
"""编码器状态节点：节点内判断「落地 / 未落地」，持续发布当前状态。

与 reference/encoder_driver 的区别：
    - encoder_driver 持续发布 encoder_angle/encoder_raw，只读不判断；
    - 本节点复用同一串口协议（ID CMD Len Data CRC，Modbus RTU 读保持寄存器），
      内部低频轮询读取角度并做落地判断，**持续发布**当前状态到
      /grasp_hexapod/encoder_state（grasp_hexapod_msgs/EncoderState：
      landed / angle / reason），供行为树 is_landing_confirmed /
      sensor_health 订阅。

落地判据（两态，按角度范围）：最近有效角度 ∈ [90,180] deg -> 已落地；
否则未落地。阈值可配 ~landing_min/~landing_max；仅收到有效帧时发布状态
（无数据不发布，订阅方保持上次状态）。

用法：
    rosrun encoder_driver encoder_status_node.py _port:=/dev/ttyUSB0
    python3 encoder_status_node.py --selftest     # 离线自检（假串口，不依赖 ROS）
"""

import argparse
import time

# 编码器参数（与 encoder_driver/encoder_frame.hpp 一致）
RESOLUTION = 262144.0     # 18 位分辨率 2**18
FULL_CIRCLE = 360.0
CMD_READ = 0x03

# 落地角度范围（deg）：[LANDING_ANGLE_MIN, LANDING_ANGLE_MAX] 视为落地
LANDING_ANGLE_MIN = 90.0
LANDING_ANGLE_MAX = 180.0


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
    """角度范围两态判断：有效角度 ∈ [min,max] -> 已落地；否则未落地。

    无状态：只对收到的有效角度样本判定，无数据时不产生输出。
    """

    def __init__(self, landing_min=LANDING_ANGLE_MIN,
                 landing_max=LANDING_ANGLE_MAX):
        if landing_min >= landing_max:
            raise ValueError("落地角度范围非法: [{},{}]".format(
                landing_min, landing_max))
        self.landing_min = landing_min
        self.landing_max = landing_max

    def judge(self, angle):
        """按角度范围判定两态。返回 dict(landed, angle, reason)。"""
        landed = self.landing_min <= angle <= self.landing_max
        reason = ("已落地: 角度 {:.1f}° ∈ [{},{}]".format(
            angle, self.landing_min, self.landing_max) if landed
            else "未落地: 角度 {:.1f}° ∉ [{},{}]".format(
            angle, self.landing_min, self.landing_max))
        return {"landed": landed, "angle": angle, "reason": reason}


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
    landing_min = float(rospy.get_param("~landing_min", LANDING_ANGLE_MIN))
    landing_max = float(rospy.get_param("~landing_max", LANDING_ANGLE_MAX))

    judge = EncoderLandingJudge(landing_min=landing_min,
                                landing_max=landing_max)
    query = build_read_query(slave_id, start_reg, reg_count)
    link = SerialLink(port, baudrate)
    pub = rospy.Publisher(state_topic, EncoderState, queue_size=1)

    def poll(_event):
        frames = link.query(query, timeout_s=1.0 / poll_hz)
        for frame in frames:           # 仅收到有效帧时发布；无数据不发
            info = parse_response(frame)
            state = judge.judge(raw_to_angle(info["raw"]))
            msg = EncoderState()
            msg.header.stamp = rospy.Time.now()
            msg.landed = bool(state["landed"])
            msg.angle = state["angle"]
            msg.reason = state["reason"]
            pub.publish(msg)

    rospy.Timer(rospy.Duration(1.0 / poll_hz), poll)
    rospy.loginfo("编码器状态节点就绪: %s@%d slave=0x%02X 落地角度范围=[%.1f,%.1f]deg "
                  "（收到有效帧即发布 %s，%gHz）",
                  port, baudrate, slave_id, landing_min, landing_max,
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

    # --- 2. 角度在 [90,180] 内 -> 落地 ---
    judge = EncoderLandingJudge()
    state = judge.judge(120.0)
    assert state["landed"] and state["angle"] == 120.0, state
    print("[OK] 角度 120° ∈ [90,180] -> 落地: reason={}".format(state["reason"]))

    # --- 3. 角度越界 -> 未落地（下界/上界） ---
    assert not judge.judge(45.0)["landed"], "45° 应未落地"
    assert not judge.judge(200.0)["landed"], "200° 应未落地"
    # 边界值：90 / 180 视为落地
    assert judge.judge(90.0)["landed"], "90° 边界应落地"
    assert judge.judge(180.0)["landed"], "180° 边界应落地"
    print("[OK] 角度越界 -> 未落地；边界 90/180 -> 落地")

    # --- 4. 假串口端到端：角度 <90 未落地 -> 角度入范围落地 ---
    script = [(0.1 * i, 32768) for i in range(10)]   # 45° -> 未落地
    link = FakeSerialLink(script)
    judge3 = EncoderLandingJudge()
    t = 0.0
    last_state = None
    for _ in range(10):
        frames = link.query(build_read_query(0, 0, 2), timeout_s=0.1)
        t += 0.1
        for frame in frames:
            last_state = judge3.judge(
                raw_to_angle(parse_response(frame)["raw"]))
    assert last_state is not None and not last_state["landed"], last_state
    # 换成 135°（raw=98304）入范围段
    link.script.clear()
    for i in range(10):
        link.script.append((t + 0.1, 98304))
    for _ in range(10):
        frames = link.query(build_read_query(0, 0, 2), timeout_s=0.1)
        t += 0.1
        for frame in frames:
            last_state = judge3.judge(
                raw_to_angle(parse_response(frame)["raw"]))
    assert last_state["landed"], "角度入 [90,180] 后应确认落地"
    print("[OK] 假串口端到端: 45° 未落地 -> 135° 落地")

    print("selftest 全部通过")


def main():
    parser = argparse.ArgumentParser(description="编码器状态节点（角度范围两态判断，持续发布 /grasp_hexapod/encoder_state）")
    parser.add_argument("--selftest", action="store_true", help="离线自检（不依赖 ROS）")
    args, ros_args = parser.parse_known_args()

    if args.selftest:
        selftest()
        return

    run_node(args)


if __name__ == "__main__":
    main()
