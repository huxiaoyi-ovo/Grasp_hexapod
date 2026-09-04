#!/usr/bin/env python3
"""传感器健康监控节点：单点订阅全部传感器话题，按频率/新鲜度统一判定。

补全现有传感器包（IMU/_gps 驱动、舵机关节反馈、相机状态等只发数据、
不做判断）缺失的数据判断层。编码器不走本节点（改由 encoder_status_node
持续发布 /grasp_hexapod/encoder_state 状态 topic，判据在编码器节点内）。

发布 /grasp_hexapod/sensor_health（grasp_hexapod_msgs/SensorHealthArray），
行为树 IsSensorDataOk/WaitSensorsReady 单点消费。

判据（每传感器，多话题时取最差）：
    online   收到过消息（通信在线）；
    fresh    age_s <= max_age 且（min_freq 配置时，warmup 后 freq_hz >= min_freq）；
    freq_hz  滑动时间窗（默认 2s）内实测频率；
    age_s    距最近一条消息的时长；
    reason   人类可读异常（如 "超时 0.8s"、"频率 2Hz < 5Hz"）。

监测范围控制（config/sensor_health.yaml 或 ~config 参数）：
    话题级 enabled: false 或传感器级 enabled: false 均可不订阅、不监测、
    不进健康报告（IsSensorDataOk/WaitSensorsReady 只检查报告内条目，
    禁用即放行）；缺省 enabled=true。

用法：
    rosrun grasp_hexapod_bt sensor_health_monitor.py
    rosrun grasp_hexapod_bt sensor_health_monitor.py _config:=<yaml>
    python3 sensor_health_monitor.py --selftest     # 离线自检（不依赖 ROS）
"""

import argparse
from collections import deque

DEFAULT_WINDOW_S = 2.0   # 频率估计滑动窗
DEFAULT_WARMUP_S = 2.0   # 启动预热（期间不判 min_freq）


# --------------------------------------------------------------------------
# 纯逻辑（可离线测试）
# --------------------------------------------------------------------------
class HealthTracker:
    """单话题健康跟踪：记录到达时刻滑窗，按需计算 online/freq/age。

    online 语义为"链路曾经建立"（启动门禁 WaitSensorsReady 用）；
    停发后报"超时"（不新鲜）而非"离线"。
    """

    def __init__(self, max_age, min_freq=0.0, window_s=DEFAULT_WINDOW_S,
                 label=None):
        if max_age <= 0:
            raise ValueError("max_age 必须为正")
        self.max_age = max_age
        self.min_freq = min_freq
        self.window_s = window_s
        self.label = label or ""
        self.events = deque()        # 滑窗内到达时刻
        self.last_event_time = None  # 最近一次到达（不受窗口淘汰影响）
        self.ever_received = False

    def feed(self, now):
        """收到一条消息时调用。"""
        self.events.append(now)
        self.last_event_time = now
        self.ever_received = True

    def snapshot(self, now):
        """计算该话题健康状态。"""
        # 淘汰滑窗外事件
        while self.events and now - self.events[0] > self.window_s:
            self.events.popleft()
        if not self.ever_received:
            return {"online": False, "fresh": False, "freq_hz": 0.0,
                    "age_s": float("inf"), "reason": "离线/无消息"}
        age_s = now - self.last_event_time
        # 滑窗内频率：事件数 / 窗时长（窗口未满时用实际跨度）
        if self.events:
            span = now - self.events[0]
            duration = max(min(span, self.window_s), 1e-6)
            freq_hz = len(self.events) / duration
        else:
            freq_hz = 0.0
        reasons = []
        fresh = True
        if age_s > self.max_age:
            fresh = False
            reasons.append("超时 {:.1f}s".format(age_s))
        if self.min_freq > 0 and freq_hz < self.min_freq:
            fresh = False
            reasons.append("频率 {:+.0f}Hz < {}Hz".format(
                int(freq_hz), int(self.min_freq)))
        return {"online": True, "fresh": fresh, "freq_hz": freq_hz,
                "age_s": age_s, "reason": "; ".join(reasons)}


class SensorHealthMonitor:
    """一个传感器（可含多个话题）的健康汇总：多话题取最差。"""

    def __init__(self, name, topics_cfg, window_s=DEFAULT_WINDOW_S):
        """topics_cfg: {topic: {"max_age": float, "min_freq": float,
                                "enabled": bool(可选, 缺省 True)}}；
        也可含传感器级开关 {"enabled": bool}（非话题键，作用于整个传感器）。
        禁用的话题不注册跟踪器；被禁用部分不出现在健康报告中。"""
        self.name = name
        self.enabled = bool(topics_cfg.get("enabled", True))
        self.trackers = {}
        if not self.enabled:
            return
        for topic, cfg in topics_cfg.items():
            if topic == "enabled" or not isinstance(cfg, dict):
                continue
            if not cfg.get("enabled", True):
                continue
            self.trackers[topic] = HealthTracker(
                max_age=cfg["max_age"],
                min_freq=cfg.get("min_freq", 0.0),
                window_s=window_s,
                label=topic,
            )
        self.started_at = None

    def start(self, now):
        self.started_at = now

    def feed(self, topic, now):
        if topic in self.trackers:
            self.trackers[topic].feed(now)

    def snapshot(self, now):
        """返回 SensorHealth 字段 dict（不含 name）。"""
        if not self.trackers:
            return {"online": False, "fresh": False, "freq_hz": 0.0,
                    "age_s": float("inf"), "reason": "未配置话题"}
        parts = [tracker.snapshot(now) for tracker in self.trackers.values()]
        online = all(p["online"] for p in parts)
        fresh = all(p["fresh"] for p in parts)
        freq_hz = min(p["freq_hz"] for p in parts)
        finite_ages = [p["age_s"] for p in parts if p["age_s"] != float("inf")]
        age_s = max(finite_ages) if finite_ages else float("inf")
        reasons = []
        warmup = self.started_at is None or now - self.started_at < DEFAULT_WARMUP_S
        for topic, part in zip(self.trackers, parts):
            if part["reason"]:
                reasons.append("{}({})".format(topic, part["reason"]))
        # 预热期仅报年龄异常，不报频率
        if warmup:
            fresh = all(
                p["fresh"] or ("频率" in p["reason"] and "超时" not in p["reason"])
                for p in parts)
            reasons = [r for r in reasons if "频率" not in r]
        reason = "; ".join(reasons)
        return {"online": online, "fresh": fresh, "freq_hz": freq_hz,
                "age_s": age_s, "reason": reason}


class SensorHealthRegistry:
    """全部传感器的注册与汇总（离线/在线共用核心逻辑）。"""

    def __init__(self, config, window_s=DEFAULT_WINDOW_S):
        """config: {sensor_name: {topic: {"max_age":…, "min_freq":…,
                  "enabled":…}}}；enabled 可选（缺省 True），全部话题被禁用
                  或传感器级 enabled=false 的传感器不注册、不进报告。"""
        self.monitors = {}
        for name, topics_cfg in config.items():
            monitor = SensorHealthMonitor(name, topics_cfg, window_s=window_s)
            if monitor.trackers:
                self.monitors[name] = monitor

    def start(self, now):
        for monitor in self.monitors.values():
            monitor.start(now)

    def feed(self, sensor, topic, now):
        if sensor in self.monitors:
            self.monitors[sensor].feed(topic, now)

    def snapshot(self, now):
        """返回 {name: {online, fresh, freq_hz, age_s, reason}}。"""
        return {name: monitor.snapshot(now)
                for name, monitor in self.monitors.items()}


# --------------------------------------------------------------------------
# 默认配置（话题集与 config/sensor_health.yaml 一致、全量启用；yaml 是实机
# 配置，可用 enabled 停用条目。模拟节点 bt_mock_world 用本配置全开仿真）
# --------------------------------------------------------------------------
DEFAULT_CONFIG = {
    "imu": {
        "/grasp_hexapod/imu": {"max_age": 0.2, "min_freq": 30.0},
    },
    "gps": {
        "/fix": {"max_age": 1.0, "min_freq": 1.0},
    },
    "rtk": {
        "/grasp_hexapod/navigation/base_pose": {"max_age": 0.5},
        "/grasp_hexapod/navigation/xiaolan_pose": {"max_age": 0.5},
        "/grasp_hexapod/navigation/pv_boundary": {"max_age": 0.5},
    },
    "servo": {
        "/lf_pos": {"max_age": 0.5, "min_freq": 10.0},
        "/lm_pos": {"max_age": 0.5, "min_freq": 10.0},
        "/lb_pos": {"max_age": 0.5, "min_freq": 10.0},
        "/rf_pos": {"max_age": 0.5, "min_freq": 10.0},
        "/rm_pos": {"max_age": 0.5, "min_freq": 10.0},
        "/rb_pos": {"max_age": 0.5, "min_freq": 10.0},
    },
    "stereo": {
        "/grasp_hexapod/stereo_ok": {"max_age": 1.0},
    },
    "mono": {
        # 真实链路 dock_tag_system.launch：usb_cam 驱动 10Hz + apriltag 检测
        # （tag_detections 无 tag 也按帧率发布空数组，可作全链路心跳）
        "/dock_camera/image_raw": {"max_age": 0.5, "min_freq": 5.0},
        "/dock/tag_detections": {"max_age": 0.5, "min_freq": 5.0},
    },
}


# --------------------------------------------------------------------------
# ROS 节点（薄封装）
# --------------------------------------------------------------------------
def run_node(args):
    import os
    import rospy
    import yaml
    from grasp_hexapod_msgs.msg import SensorHealth, SensorHealthArray

    rospy.init_node("sensor_health_monitor")
    config_path = rospy.get_param(
        "~config",
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "config", "sensor_health.yaml"))
    publish_hz = float(rospy.get_param("~publish_hz", 5.0))

    if os.path.isfile(config_path):
        with open(config_path) as f:
            config = yaml.safe_load(f)["sensors"]
    else:
        rospy.logwarn("配置 %s 不存在，使用内置默认", config_path)
        config = DEFAULT_CONFIG

    registry = SensorHealthRegistry(config)
    pub = rospy.Publisher("/grasp_hexapod/sensor_health", SensorHealthArray,
                          queue_size=2)

    def subscribe_all():
        now = rospy.get_time()
        registry.start(now)
        for sensor, monitor in registry.monitors.items():
            for topic in monitor.trackers:
                rospy.Subscriber(
                    topic, rospy.AnyMsg,
                    lambda msg, s=sensor, t=topic: registry.feed(
                        s, t, rospy.get_time()),
                queue_size=1)
        rospy.loginfo("传感器健康监控就绪: %d 个传感器, %d 路话题, 配置 %s",
                      len(registry.monitors),
                      sum(len(m.trackers) for m in registry.monitors.values()),
                      config_path)

    def publish(_event):
        now = rospy.get_time()
        msg = SensorHealthArray()
        msg.header.stamp = rospy.Time.now()
        for name, state in sorted(registry.snapshot(now).items()):
            health = SensorHealth()
            health.name = name
            health.online = bool(state["online"])
            health.fresh = bool(state["fresh"])
            health.freq_hz = state["freq_hz"]
            health.age_s = state["age_s"] if state["age_s"] != float("inf") else 999.0
            health.reason = state["reason"]
            msg.sensors.append(health)
        pub.publish(msg)

    subscribe_all()
    rospy.Timer(rospy.Duration(1.0 / publish_hz), publish)
    rospy.spin()


# --------------------------------------------------------------------------
# 离线自检
# --------------------------------------------------------------------------
def selftest():
    # --- 1. 正常频率流：online/fresh，频率估计准确 ---
    cfg = {"imu": {"/imu": {"max_age": 0.2, "min_freq": 30.0}}}
    registry = SensorHealthRegistry(cfg)
    registry.start(0.0)
    for i in range(100):           # 50Hz
        registry.feed("imu", "/imu", 0.02 * i)
    state = registry.snapshot(2.0 - 1e-9)
    assert state["imu"]["online"] and state["imu"]["fresh"], state
    assert 45.0 < state["imu"]["freq_hz"] < 50.1, state
    print("[OK] 正常频率流: fresh={} freq={:.0f}Hz".format(
        state["imu"]["fresh"], state["imu"]["freq_hz"]))

    # --- 2. 停发 -> 超时不新鲜 ---
    state = registry.snapshot(2.5)  # 最后事件 t=1.96，age≈0.54 > 0.2
    assert not state["imu"]["fresh"] and "超时" in state["imu"]["reason"], state
    print("[OK] 停发超时: reason={}".format(state["imu"]["reason"]))

    # --- 3. 低频 -> 频率不足（预热期后） ---
    cfg3 = {"cam": {"/cam": {"max_age": 1.0, "min_freq": 5.0}}}
    registry3 = SensorHealthRegistry(cfg3)
    registry3.start(0.0)
    for i in range(30):            # 2Hz
        registry3.feed("cam", "/cam", 0.5 * i)
    state3 = registry3.snapshot(16.0)
    assert not state3["cam"]["fresh"] and "频率" in state3["cam"]["reason"], state3
    print("[OK] 低频不新鲜: reason={}".format(state3["cam"]["reason"]))

    # --- 4. 预热期内低频不判罚（只判超时/离线） ---
    registry3b = SensorHealthRegistry(cfg3)
    registry3b.start(0.0)
    for i in range(6):             # 预热期内 2Hz
        registry3b.feed("cam", "/cam", 0.5 * i)
    state3b = registry3b.snapshot(1.5)
    assert state3b["cam"]["fresh"], state3b
    print("[OK] 预热期低频不判罚: fresh={}".format(state3b["cam"]["fresh"]))

    # --- 5. 多话题取最差：一路停发 -> 整个传感器不新鲜 ---
    cfg5 = {"rtk": {"/pose_a": {"max_age": 0.5}, "/pose_b": {"max_age": 0.5}}}
    registry5 = SensorHealthRegistry(cfg5)
    registry5.start(0.0)
    for i in range(20):
        registry5.feed("rtk", "/pose_a", 0.1 * i)
        registry5.feed("rtk", "/pose_b", 0.1 * i)
    for i in range(20):            # pose_b 停发，pose_a 继续
        registry5.feed("rtk", "/pose_a", 2.0 + 0.1 * i)
    state5 = registry5.snapshot(4.0)
    assert state5["rtk"]["online"] and not state5["rtk"]["fresh"], state5
    assert "/pose_b" in state5["rtk"]["reason"], state5
    print("[OK] 多话题取最差: reason={}".format(state5["rtk"]["reason"]))

    # --- 6. 完全离线 ---
    cfg6 = {"mono": {"/mono": {"max_age": 1.0}}}
    registry6 = SensorHealthRegistry(cfg6)
    registry6.start(0.0)
    state6 = registry6.snapshot(5.0)
    assert not state6["mono"]["online"] and not state6["mono"]["fresh"], state6
    print("[OK] 完全离线: reason={}".format(state6["mono"]["reason"]))

    # --- 7. 默认配置可加载 ---
    registry7 = SensorHealthRegistry(DEFAULT_CONFIG)
    assert set(registry7.monitors) == {"imu", "gps", "rtk", "servo", "stereo", "mono"}
    print("[OK] 默认配置: {} 路话题".format(
        sum(len(m.trackers) for m in registry7.monitors.values())))

    # --- 8. enabled 开关：话题级/传感器级禁用不订阅、不进报告 ---
    cfg8 = {
        "cam": {"/on": {"max_age": 1.0},
                "/off": {"max_age": 1.0, "enabled": False}},
        "lidar": {"enabled": False, "/scan": {"max_age": 1.0}},
    }
    registry8 = SensorHealthRegistry(cfg8)
    registry8.start(0.0)
    assert set(registry8.monitors) == {"cam"}, registry8.monitors.keys()
    assert set(registry8.monitors["cam"].trackers) == {"/on"}
    registry8.feed("cam", "/on", 0.1)
    state8 = registry8.snapshot(0.2)
    assert set(state8) == {"cam"} and state8["cam"]["online"], state8
    print("[OK] enabled 开关: 话题级/传感器级禁用不监测不报告")

    print("selftest 全部通过")


def main():
    parser = argparse.ArgumentParser(description="传感器健康监控（频率/新鲜度判断层）")
    parser.add_argument("--selftest", action="store_true", help="离线自检（不依赖 ROS）")
    args, _ = parser.parse_known_args()

    if args.selftest:
        selftest()
        return

    run_node(args)


if __name__ == "__main__":
    main()
