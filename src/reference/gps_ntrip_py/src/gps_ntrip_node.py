#!/usr/bin/env python3
# coding=utf-8
"""GPS 串口数据读取与 NMEA 解析节点（已去除 NTRIP 客户端部分）。

功能：
  - 打开 GPS 串口，周期请求 GGA 数据（发送 "GPGGA 1"）
  - 读取串口 NMEA 语句，解析 GPGGA / GNGGA（位置、定位质量）与
    GNGSTH（各轴 1σ 误差标准差）
  - 将位置发布为 sensor_msgs/NavSatFix；GSTH 的 lat/lon/alt 标准差
    以 σ² 填入 position_covariance 对角（ENU 顺序，type=DIAGONAL_KNOWN）

注意：
  GSTH 使能命令（如 "GPGSTH 1"）不在本节点内发送，需在外部配置
  （串口工具 / 接收机保存配置后上电即输出），节点只负责解析。

参数（可通过 launch 或 rosrun 传参，均带默认值）：
  ~port       串口设备，默认 /dev/ttyUSB0
  ~baudrate   波特率，默认 115200
  ~fix_topic  NavSatFix 发布话题，默认 /fix（robot_localization、rviz 等默认订阅此话题）
  ~frame_id   坐标系，默认 gps（接收机天线位置）
  ~ask_period GGA 请求周期（秒），默认 3.0

示例：
  rosrun gps_ntrip_py gps_ntrip_node.py _port:=/dev/ttyUSB1 _baudrate:=9600
  rostopic echo /fix
"""
import rospy
import serial
from sensor_msgs.msg import NavSatFix, NavSatStatus

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 115200
DEFAULT_FIX_TOPIC = "/fix"
DEFAULT_FRAME_ID = "gps"
DEFAULT_ASK_PERIOD = 3.0  # 秒，周期向模块请求一次 GGA
ASK_GGA = b"GPGGA 1\r\n"
MAX_BUFFER = 4096  # 串口缓冲上限，防止无换行时无限增长


def nmea_checksum_ok(sentence):
    """校验 NMEA 语句的 *XX 校验和。返回 True 表示合法（或没有校验和段）。"""
    if "*" not in sentence:
        return True
    body, _, ck = sentence.partition("*")
    ck = ck.strip()
    if len(ck) < 2:
        return False
    calc = 0
    for ch in body[1:]:  # 跳过开头的 $
        calc ^= ord(ch)
    try:
        return int(ck[:2], 16) == calc
    except ValueError:
        return False


def parse_ddm(ddm, hemisphere):
    """NMEA 度分格式 (DDMM.MMMMM) 转十进制，南纬/西经取负。失败返回 None。"""
    if not ddm or not hemisphere:
        return None
    try:
        value = float(ddm)
    except ValueError:
        return None
    degrees = int(value // 100)
    minutes = value - degrees * 100
    decimal = degrees + minutes / 60.0
    if hemisphere in ("S", "W"):
        decimal = -decimal
    return decimal


def fix_status_from_quality(quality):
    """GGA 定位质量字段 -> NavSatStatus.status。"""
    if quality == 0:
        return NavSatStatus.STATUS_NO_FIX
    if quality == 1:
        return NavSatStatus.STATUS_FIX
    if quality == 2:
        return NavSatStatus.STATUS_SBAS_FIX
    if quality in (4, 5):
        # 4=RTK 固定解, 5=RTK 浮点解 -> 地基增强
        return NavSatStatus.STATUS_GBAS_FIX
    return NavSatStatus.STATUS_NO_FIX


def parse_gga(data_str):
    """解析一条 GPGGA/GNGGA 语句。

    返回 dict（quality/lat/lon/alt），解析失败返回 None。
    altitude 为 WGS84 椭球高 = 海拔高(MSL) + 大地水准面差距(geoid separation)。
    """
    if not (data_str.startswith("$GPGGA") or data_str.startswith("$GNGGA")):
        return None
    fields = data_str.split(",")
    if len(fields) < 14:
        rospy.logwarn("GGA 字段数不足: %s", data_str.strip())
        return None
    if not nmea_checksum_ok(data_str):
        rospy.logwarn("GGA 校验和错误: %s", data_str.strip())
        return None
    try:
        quality = int(fields[6])
        lat = parse_ddm(fields[2], fields[3])
        lon = parse_ddm(fields[4], fields[5])
        if lat is None or lon is None:
            return None
        msl_alt = float(fields[9]) if fields[9] else 0.0
        geoid = float(fields[11]) if len(fields) > 11 and fields[11] else 0.0
    except (ValueError, IndexError):
        rospy.logwarn("GGA 字段解析失败: %s", data_str.strip())
        return None
    return {"quality": quality, "lat": lat, "lon": lon, "alt": msl_alt + geoid}


def parse_gsth(data_str):
    """解析 $GNGSTH（从天线计算的伪距观测误差信息）。

    $GNGSTH,utc,rms,smjr_std,smnr_std,orient,lat_std,lon_std,alt_std*CK
    返回 dict(lat_std, lon_std, alt_std)（单位米，1σ），失败返回 None。
    """
    if not data_str.startswith("$GNGSTH"):
        return None
    if not nmea_checksum_ok(data_str):
        rospy.logwarn("GSTH 校验和错误: %s", data_str.strip())
        return None
    # GSTH 以数字结尾，校验和紧贴最后字段（无逗号分隔），先剥离
    body = data_str.split("*")[0]
    fields = body.split(",")
    if len(fields) < 9:
        rospy.logwarn("GSTH 字段数不足: %s", data_str.strip())
        return None
    try:
        lat_std = float(fields[6])
        lon_std = float(fields[7])
        alt_std = float(fields[8])
    except (ValueError, IndexError):
        rospy.logwarn("GSTH 字段解析失败: %s", data_str.strip())
        return None
    if lat_std < 0 or lon_std < 0 or alt_std < 0:
        return None
    return {"lat_std": lat_std, "lon_std": lon_std, "alt_std": alt_std}


def gps_serial_node():
    rospy.init_node("gps_ntrip_node", anonymous=True)

    port = rospy.get_param("~port", DEFAULT_PORT)
    baudrate = int(rospy.get_param("~baudrate", DEFAULT_BAUDRATE))
    fix_topic = rospy.get_param("~fix_topic", DEFAULT_FIX_TOPIC)
    frame_id = rospy.get_param("~frame_id", DEFAULT_FRAME_ID)
    ask_period = float(rospy.get_param("~ask_period", DEFAULT_ASK_PERIOD))

    pub = rospy.Publisher(fix_topic, NavSatFix, queue_size=10)

    # 打开串口
    sp = serial.Serial(port, baudrate, timeout=1)
    rospy.loginfo("GPS serial opened: %s @ %d baud, publish NavSatFix -> %s", port, baudrate, fix_topic)

    rate = rospy.Rate(100)
    buffer = bytearray()
    last_ask = rospy.get_time()
    sp.write(ASK_GGA)  # 启动时请求一次 GGA

    # 最近一次 GSTH 的 ENU 方差 (east, north, up)，None 表示尚未收到
    cov_enu = None

    while not rospy.is_shutdown():
        # 周期请求 GGA（适配需要轮询的模块）
        now = rospy.get_time()
        if now - last_ask >= ask_period:
            try:
                sp.write(ASK_GGA)
                last_ask = now
            except Exception as e:
                rospy.logerr("Failed to request GGA: %s", e)

        # 读取串口原始数据
        try:
            if sp.in_waiting > 0:
                data = sp.read(sp.in_waiting)
                if isinstance(data, str):  # py2 兼容，py3 下 read 返回 bytes
                    data = data.encode("utf-8")
                buffer.extend(data)
                if len(buffer) > MAX_BUFFER:
                    rospy.logwarn("Serial buffer overflow, dropping oldest data")
                    del buffer[: len(buffer) - MAX_BUFFER]
        except Exception as e:
            rospy.logerr("Failed to read serial: %s", e)

        # 按行提取并解析
        while True:
            idx = buffer.find(b"\n")
            if idx == -1:
                break
            line = bytes(buffer[:idx]).rstrip(b"\r")
            del buffer[: idx + 1]
            if line.startswith(b"$"):
                data_str = line.decode("ascii", errors="ignore")
                if data_str.startswith("$GNGSTH"):
                    # GSTH：更新各轴 1σ -> ENU 方差（东≈经度误差，北≈纬度误差，天≈高程误差）
                    gsth = parse_gsth(data_str)
                    if gsth is not None:
                        cov_enu = (gsth["lon_std"] ** 2, gsth["lat_std"] ** 2, gsth["alt_std"] ** 2)
                        rospy.loginfo(
                            "GSTH: lat_std=%.3f lon_std=%.3f alt_std=%.3f m",
                            gsth["lat_std"], gsth["lon_std"], gsth["alt_std"],
                        )
                    continue
                result = parse_gga(data_str)
                if result is not None:
                    fix = NavSatFix()
                    fix.header.stamp = rospy.Time.now()
                    fix.header.frame_id = frame_id
                    fix.status.status = fix_status_from_quality(result["quality"])
                    fix.status.service = NavSatStatus.SERVICE_GPS
                    fix.latitude = result["lat"]
                    fix.longitude = result["lon"]
                    fix.altitude = result["alt"]
                    if cov_enu is not None and (cov_enu[0] > 0 or cov_enu[1] > 0 or cov_enu[2] > 0):
                        fix.position_covariance = [
                            cov_enu[0], 0.0, 0.0,
                            0.0, cov_enu[1], 0.0,
                            0.0, 0.0, cov_enu[2],
                        ]
                        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                    else:
                        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                    pub.publish(fix)
                    rospy.loginfo(
                        "NavSatFix: lat=%.7f lon=%.7f alt=%.2fm fix=%d | %s",
                        fix.latitude, fix.longitude, fix.altitude,
                        result["quality"], data_str.strip(),
                    )

        rate.sleep()

    if sp.is_open:
        sp.close()


if __name__ == "__main__":
    try:
        gps_serial_node()
    except rospy.ROSInterruptException:
        pass
    except serial.SerialException as e:
        rospy.logfatal("Cannot open serial port: %s", e)
