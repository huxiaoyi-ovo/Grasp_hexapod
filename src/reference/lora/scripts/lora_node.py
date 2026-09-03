#!/usr/bin/env python3
"""LoRa 串口节点：地面命令（下行）与机器人回传（上行）的串口 ↔ ROS 桥接。

职责：
    - 下行（地面 → 机器人）：串口字节流 → 按 \r\n 分行 → 校验 → 去校验 →
      发布**净帧**到 /lora/command（std_msgs/String），供行为树
      WaitTaskCommand(⑤/⑲) / WaitDeployment(⑨) / WaitWinchHoisted(⑫/㉜) 消费。
    - 上行（机器人 → 地面）：订阅 /lora/status（净帧，行为树 ReportStatus
      发布）→ 追加校验 + \r\n → 写串口。

线协议（帧/校验/命令/回传）：
    帧格式     <帧体>*<校验>\r\n
               校验 = 帧体所有字节求和 mod 256 的两位大写十六进制；
               校验失败丢弃并计数（不发布），分片/粘包按 \r\n 重组。
    命令(下行) CMD,<目标>,<指令>[,<参数>...]        目标 HEX=六足
               RELEASE → ⑤ 释放任务；RECOVER → ⑲ 回收任务；
               DEPLOY → ⑨ 下放开始；HOIST_DONE → ⑫/㉜ 回收完成；其他透传。
    回传(上行) STA,<来源>,<状态>,<x>,<y>            来源 HEX
               状态 ∈ IDLE/LANDED/RELEASED/CLAMPED/DONE/FAILED；x,y 位置。

参数：
    ~port            串口设备，默认 /dev/lora
    ~baud            波特率，默认 115200（8N1）
    ~publish_topic   下行净帧话题，默认 /lora/command
    ~status_topic    上行净帧话题，默认 /lora/status
    ~enable_checksum 是否启用 *CK 校验，默认 true（false = 纯行兼容旧帧）
    ~verbose         打印收发帧与校验失败计数

用法：
    rosrun lora lora_node.py _port:=/dev/lora _baud:=115200
    python3 lora_node.py --selftest     # 离线自检（假串口，不依赖 ROS）
"""

import argparse
import threading
from collections import deque


class LoRaCodec:
    """串口帧编解码（可离线自检）。

    行分隔 \r\n；启用校验时每帧形如 '<body>*CK'，CK 为 body 字节求和
    mod 256 的两位大写十六进制。feed() 按块喂入字节流，返回完整合法净帧。
    """

    def __init__(self, enable_checksum=True):
        self.enable_checksum = bool(enable_checksum)
        self.buf = bytearray()
        self.dropped = 0     # 校验失败/坏帧计数
        self.lines = 0       # 收到的完整行数（含坏帧）

    @staticmethod
    def checksum(body):
        """帧体校验：字节求和 mod 256，两位大写十六进制。"""
        total = 0
        for byte in body:
            total = (total + byte) % 256
        return "{:02X}".format(total)

    @staticmethod
    def build_frame(body):
        """把净帧封装为串口行：body*CK\\r\\n。"""
        text = body if isinstance(body, str) else body.decode("utf-8", "ignore")
        return "{}*{}\r\n".format(text, LoRaCodec.checksum(text.encode("utf-8"))).encode("utf-8")

    def feed(self, chunk):
        """喂入一块字节，返回本次解出的净帧列表（已去校验、去 \r\n）。"""
        self.buf.extend(chunk if isinstance(chunk, bytes) else chunk.encode("utf-8"))
        frames = []
        while True:
            idx = self.buf.find(b"\n")
            if idx < 0:
                break
            line = bytes(self.buf[:idx])
            del self.buf[:idx + 1]
            self.lines += 1
            text = line.decode("utf-8", "ignore").rstrip("\r")
            frame = self._validate(text)
            if frame is not None:
                frames.append(frame)
        return frames

    def _validate(self, line):
        """校验单行；合法返回净帧 body，否则 None 并计数。"""
        if not line:
            return None
        if self.enable_checksum:
            if "*" not in line:
                self.dropped += 1
                return None
            body, ck = line.rsplit("*", 1)
            if not body or len(ck) != 2:
                self.dropped += 1
                return None
            if LoRaCodec.checksum(body.encode("utf-8")) != ck.upper():
                self.dropped += 1
                return None
            return body
        return line.rstrip("*").strip()


class SerialLink:
    """pyserial 封装：读字节流（非阻塞轮询式 read）、写一帧。"""

    def __init__(self, port, baudrate):
        import serial
        self.serial = serial.Serial(port, baudrate, timeout=0.05)
        self.serial.flushInput()

    def read_bytes(self, max_bytes=512):
        """读取当前可读字节；空返回 b''。"""
        return self.serial.read(max_bytes)

    def write_frame(self, frame_bytes):
        self.serial.write(frame_bytes)
        self.serial.flush()

    def close(self):
        try:
            self.serial.close()
        except Exception:  # noqa: BLE001
            pass


class FakeSerialLink:
    """假串口：设备侧读缓冲（inject 供下行）与主机侧写缓冲（供上行断言）。"""

    def __init__(self):
        self.device_read = deque()   # 地面 → 设备（下行原始字节流）
        self.host_written = []       # 设备 → 地面（上行原始字节流）

    def inject(self, raw):
        """模拟地面串口写入的字节（支持分片）。"""
        self.device_read.append(raw if isinstance(raw, bytes) else raw.encode("utf-8"))

    def read_bytes(self, max_bytes=512):
        if not self.device_read:
            return b""
        chunk = self.device_read.popleft()
        return chunk[:max_bytes]

    def write_frame(self, frame_bytes):
        self.host_written.append(frame_bytes)

    def close(self):
        pass


# --------------------------------------------------------------------------
# ROS 节点
# --------------------------------------------------------------------------
def run_node(args):
    import rospy
    from std_msgs.msg import String

    rospy.init_node("lora_node", anonymous=True)
    port = rospy.get_param("~port", "/dev/lora")
    baudrate = int(rospy.get_param("~baud", 115200))
    publish_topic = rospy.get_param("~publish_topic", "/lora/command")
    status_topic = rospy.get_param("~status_topic", "/lora/status")
    enable_checksum = bool(rospy.get_param("~enable_checksum", True))
    verbose = bool(rospy.get_param("~verbose", True))
    read_chunk = int(rospy.get_param("~read_chunk", 512))

    codec = LoRaCodec(enable_checksum=enable_checksum)
    link = SerialLink(port, baudrate)
    pub = rospy.Publisher(publish_topic, String, queue_size=10)
    stop = threading.Event()

    def on_status(msg):
        """上行：/lora/status 净帧 → 校验帧写串口。"""
        text = str(msg.data).strip()
        if not text:
            rospy.logwarn_throttle(5.0, "忽略空 /lora/status 消息")
            return
        if not (text.startswith("STA,") or not enable_checksum):
            # 约定上报以 STA, 开头；非 STA 且开校验时仍透传（告警一次）
            rospy.logwarn_throttle(5.0, "/lora/status 非 STA 帧: %s", text)
        frame = LoRaCodec.build_frame(text)
        try:
            link.write_frame(frame)
            if verbose:
                rospy.loginfo("[LoRa TX] %s", text)
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn_throttle(5.0, "LoRa 串口写失败: %s", exc)

    def reader():
        """下行：串口字节 → 净帧 → 发布 /lora/command。"""
        while not stop.is_set():
            try:
                chunk = link.read_bytes(read_chunk)
            except Exception as exc:  # noqa: BLE001
                rospy.logerr_throttle(5.0, "LoRa 串口读失败: %s", exc)
                continue
            if not chunk:
                continue
            frames = codec.feed(chunk)
            for body in frames:
                if verbose:
                    rospy.loginfo("[LoRa RX] %s", body)
                pub.publish(String(data=body))
            if verbose and codec.dropped and codec.lines:
                rospy.logwarn("LoRa 已丢弃 %d 帧（共 %d 行）", codec.dropped, codec.lines)

    rospy.Subscriber(status_topic, String, on_status, queue_size=10)
    rospy.loginfo("LoRa 节点就绪: %s@%d 校验=%s 下行=%s 上行=%s",
                  port, baudrate, enable_checksum, publish_topic, status_topic)
    rospy.on_shutdown(lambda: (stop.set(), link.close()))
    threading.Thread(target=reader, daemon=True).start()
    rospy.spin()


# --------------------------------------------------------------------------
# 离线自检
# --------------------------------------------------------------------------
def selftest():
    # --- 1. 校验和向量 ---
    assert LoRaCodec.checksum(b"ABC") == "C6", LoRaCodec.checksum(b"ABC")
    body = "CMD,HEX,RELEASE"
    ck = LoRaCodec.checksum(body.encode("utf-8"))
    frame = LoRaCodec.build_frame(body)
    assert frame.decode("utf-8") == "{}*{}\r\n".format(body, ck), frame
    print("[OK] 校验和向量与帧封装")

    # --- 2. 合法帧解析（整行 + 分片/粘包） ---
    codec = LoRaCodec(enable_checksum=True)
    frames = codec.feed(frame)  # 整行
    assert frames == [body], frames
    codec2 = LoRaCodec(enable_checksum=True)
    frames2 = codec2.feed(frame[:4])
    frames2 += codec2.feed(frame[4:-2])          # 分片
    frames2 += codec2.feed(frame[-2:] + b"CMD,HEX,DEPLOY*" )  # 粘包第二帧未结束
    assert frames2 == [body], frames2
    assert codec2.buf  # 残存半行
    frames2 += codec2.feed(
        LoRaCodec.checksum(b"CMD,HEX,DEPLOY").encode() + b"\r\n")
    assert frames2 == [body, "CMD,HEX,DEPLOY"], frames2
    print("[OK] 整行/分片/粘包解析")

    # --- 3. 坏帧拒绝 ---
    codec3 = LoRaCodec(enable_checksum=True)
    bad = bytearray(frame)
    bad[-4] ^= 0xFF  # 破坏校验字符
    frames3 = codec3.feed(bytes(bad))
    assert frames3 == [] and codec3.dropped == 1, (frames3, codec3.dropped)
    frames3 = codec3.feed(b"NOCHECKSUM\r\n")
    assert frames3 == [] and codec3.dropped == 2
    print("[OK] 坏校验/缺校验帧拒绝")

    # --- 4. 关闭校验的纯行兼容 ---
    codec4 = LoRaCodec(enable_checksum=False)
    frames4 = codec4.feed(b"CMD,HEX,RELEASE\r\nSTA,HEX,DONE,0.00,0.00\r\n")
    assert frames4 == ["CMD,HEX,RELEASE", "STA,HEX,DONE,0.00,0.00"], frames4
    print("[OK] 纯行兼容（无校验）")

    # --- 5. 假串口端到端：上行 STA 写串口 ---
    fake = FakeSerialLink()
    codec5 = LoRaCodec(enable_checksum=True)
    sta = "STA,HEX,LANDED,0.00,0.00"
    fake.write_frame(LoRaCodec.build_frame(sta))
    assert fake.host_written, "未写入"
    sent = fake.host_written[-1].decode("utf-8").rstrip("\r\n")
    body5, ck5 = sent.rsplit("*", 1)
    assert body5 == sta and ck5 == LoRaCodec.checksum(sta.encode("utf-8")), sent
    print("[OK] 上行 STA 帧写串口（含校验）")

    # --- 6. 假串口端到端：下行 CMD 读串口（分片）→ 净帧 ---
    codec6 = LoRaCodec(enable_checksum=True)
    full6 = LoRaCodec.build_frame("CMD,HEX,DEPLOY")
    fake.inject(full6[:7])
    fake.inject(full6[7:])
    got = []
    while fake.device_read or codec6.buf:
        chunk = fake.read_bytes()
        if not chunk:
            break
        got.extend(codec6.feed(chunk))
    assert got == ["CMD,HEX,DEPLOY"], got
    print("[OK] 下行 CMD 帧读串口（分片）→ 净帧")

    print("selftest 全部通过")


def main():
    parser = argparse.ArgumentParser(description="LoRa 串口节点（/lora@115200 ↔ ROS）")
    parser.add_argument("--selftest", action="store_true", help="离线自检（不依赖 ROS）")
    args, _ = parser.parse_known_args()
    if args.selftest:
        selftest()
        return
    run_node(args)


if __name__ == "__main__":
    main()
