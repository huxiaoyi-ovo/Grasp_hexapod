#!/usr/bin/env python3
# coding=utf-8
"""重构后节点验证：串口读取 + GGA 解析 + NavSatFix 发布。

  - 单元验证 parse_gga / 度分转换 / RTK 状态映射
  - 端到端验证：FakeSerial 模拟 GPS，节点读取 "GPGGA 1" 请求应答的 GGA 并发布 NavSatFix

用法: python3 run_serial_node.py          # 只跑单元验证
      python3 run_serial_node.py e2e      # 端到端（配合外部 rostopic echo /fix）
      python3 run_serial_node.py unit
"""
import os
import sys
import threading
import time

PKG_SRC = os.path.join(os.path.dirname(__file__), "..", "src")


def gga_with_ck(body):
    ck = 0
    for ch in body[1:]:
        ck ^= ord(ch)
    return body + "*%02X" % ck


# GGA 样例：纬度 31°01.2345678'N，经度 121°26.1234567'E，质量1，海拔12.3M，大地水准面差-2.0M
VALID_GGA_BODY = "$GPGGA,083700.00,3101.2345678,N,12126.1234567,E,1,08,1.0,12.3,M,-2.0,M,,"
VALID_GGA = gga_with_ck(VALID_GGA_BODY)
BAD_CK_GGA = "$GPGGA,083700.00,3101.2345678,N,12126.1234567,E,1,08,1.0,12.3,M,-2.0,M,,*00"
SHORT_GGA = "$GPGGA,083700.00,3101,N"
RMC_SENTENCE = "$GPRMC,083700.00,A,3101.234,N,12126.123,E,0.0,0.0,260826,,,D*47"
RTK_GGA_BODY = "$GPGGA,083700.00,3101.2345678,N,12126.1234567,E,4,10,0.8,12.3,M,-2.0,M,,"
SW_GGA_BODY = "$GPGGA,083700.00,3101.2345678,S,12126.1234567,W,1,08,1.0,12.3,M,-2.0,M,,"

# GSTH 样例：lat_std=1.2, lon_std=1.5, alt_std=2.0（米，1σ）
GSTH_BODY = "$GNGSTH,083700.00,0.450,1.200,1.500,127.6430,1.2,1.5,2.0"
GSTH = gga_with_ck(GSTH_BODY)  # 同一校验和算法
# 手册示例（校验和 *0F）
MANUAL_GSTH = "$GNGSTH,055543.00,0.45,0.01,0.01,127.6430,0.010,0.010,0.019*0F"


class FakeSerial:
    """进程内模拟串口：node_buf 是 GPS->节点，host_buf 是节点->GPS。"""
    def __init__(self, *args, **kwargs):
        self.is_open = True
        self.node_buf = bytearray()
        self.host_buf = bytearray()

    @property
    def in_waiting(self):
        return len(self.node_buf)

    def read(self, n=1):
        if not self.node_buf:
            return b""
        d = bytes(self.node_buf[:n])
        del self.node_buf[:n]
        return d

    def write(self, data):
        if isinstance(data, str):
            data = data.encode("utf-8")
        self.host_buf.extend(data)
        return len(data)

    def close(self):
        self.is_open = False


def run_unit_tests():
    sys.path.insert(0, PKG_SRC)
    import gps_ntrip_node as mod

    # 合法 GGA：度分转十进 + 椭球高
    r = mod.parse_gga(VALID_GGA)
    assert r is not None, "合法 GGA 应解析成功"
    assert abs(r["lat"] - (31 + 1.2345678 / 60.0)) < 1e-9, "纬度度分转换错误: %r" % r
    assert abs(r["lon"] - (121 + 26.1234567 / 60.0)) < 1e-9, "经度度分转换错误: %r" % r
    assert abs(r["alt"] - (12.3 + (-2.0))) < 1e-9, "椭球高=MSL+geoid 错误: %r" % r
    assert r["quality"] == 1

    # 校验和错误 / 字段不足 / 非 GGA
    assert mod.parse_gga(BAD_CK_GGA) is None, "校验和错误应拒绝"
    assert mod.parse_gga(SHORT_GGA) is None, "字段不足应拒绝"
    assert mod.parse_gga(RMC_SENTENCE) is None, "非 GGA 语句应拒绝"

    # RTK 质量 4 -> GBAS(地基增强)
    rtk = mod.parse_gga(gga_with_ck(RTK_GGA_BODY))
    assert rtk is not None and rtk["quality"] == 4
    assert mod.fix_status_from_quality(4) == mod.NavSatStatus.STATUS_GBAS_FIX
    assert mod.fix_status_from_quality(5) == mod.NavSatStatus.STATUS_GBAS_FIX
    assert mod.fix_status_from_quality(1) == mod.NavSatStatus.STATUS_FIX
    assert mod.fix_status_from_quality(0) == mod.NavSatStatus.STATUS_NO_FIX

    # 南纬/西经取负
    sw = mod.parse_gga(gga_with_ck(SW_GGA_BODY))
    assert sw is not None and sw["lat"] < 0 and sw["lon"] < 0, "半球符号错误: %r" % sw

    # ---- GSTH 解析 ----
    r = mod.parse_gsth(GSTH)
    assert r == {"lat_std": 1.2, "lon_std": 1.5, "alt_std": 2.0}, "GSTH 解析错误: %r" % r
    # 手册示例校验和
    assert mod.parse_gsth(MANUAL_GSTH) == {"lat_std": 0.010, "lon_std": 0.010, "alt_std": 0.019}
    # 坏校验和 / 字段不足 / 非 GSTH / 负值
    bad_ck = "$GNGSTH,083700.00,0.450,1.200,1.500,127.6430,1.2,1.5,2.0*00"
    assert mod.parse_gsth(bad_ck) is None, "GSTH 坏校验和应拒绝"
    assert mod.parse_gsth("$GNGSTH,083700.00,0.45") is None, "GSTH 字段不足应拒绝"
    assert mod.parse_gsth(VALID_GGA) is None, "GGA 不是 GSTH"
    neg_gsth = gga_with_ck("$GNGSTH,083700.00,0.450,1.200,1.500,127.6430,-1.0,1.5,2.0")
    assert mod.parse_gsth(neg_gsth) is None, "GSTH 负标准差应拒绝"

    print("[unit] parse_gga / 度分转换 / RTK 状态映射 / parse_gsth 全部通过")


def run_end2end():
    import serial as serial_mod
    fake = FakeSerial()
    serial_mod.Serial = lambda *a, **k: fake  # 节点创建串口时返回共享 FakeSerial

    sys.path.insert(0, PKG_SRC)
    import gps_ntrip_node as mod

    def feeder():
        while True:
            if b"GPGGA 1" in fake.host_buf:
                del fake.host_buf[:]  # 消费请求，防止重复应答
                # 模拟外部已配置 GPGSTH 1 的接收机：应答 GGA + 周期输出 GSTH
                fake.node_buf.extend((VALID_GGA + "\r\n").encode("utf-8"))
                fake.node_buf.extend((GSTH + "\r\n").encode("utf-8"))
            time.sleep(0.02)

    threading.Thread(target=feeder, daemon=True).start()
    print("[e2e] 节点启动中（主线程，rospy 信号正常）...", flush=True)
    print("[e2e] 请另开终端执行: rostopic echo /fix -n 3", flush=True)
    print("[e2e] 期望值: lat=31.02057613 lon=121.43539095 alt=10.30", flush=True)
    print("[e2e] 期望协方差(σ², ENU): [2.25,0,0, 0,1.44,0, 0,0,4.0] type=2 (DIAGONAL_KNOWN)", flush=True)
    mod.gps_serial_node()  # 阻塞运行，由外部 timeout 结束


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "unit"
    if mode == "unit":
        run_unit_tests()
    elif mode == "e2e":
        run_end2end()
    else:
        print(__doc__)
