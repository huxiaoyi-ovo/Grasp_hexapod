#!/usr/bin/env python3
"""模拟反馈节点：**唯一的仿真来源**——树节点/运行器不仿真；当真实节点缺失时，
单独运行本节点，按 config/real_bt.yaml 对 simulate:true 的接口模拟发布话题/应答服务
（标准名），使行为树在“部分实机 + 部分模拟”下仍能完成完整任务。

run_real_bt.py 为纯真实运行器，不含仿真；如需补缺，请另开本节点（一个进程即可提供
多个接口的模拟）。

LoRa：**话题级仿真（需要）**——本节点按时间线注入 /lora/command（CMD 帧）并观测
/lora/status（STA）；**串口读取不仿真**——真实串口收发由 reference/lora 的
lora_node.py 负责（sim 不读/不写串口）。接真实地面站时把 yaml 里 lora_command
simulate 关掉即可由 lora_node 接管。

可模拟接口（标准名）：
    话题(发布/应答)  sensor_health /grasp_hexapod/sensor_health
                    encoder_state /grasp_hexapod/encoder_state
                    fix          /fix
                    lora_command  /lora/command    （话题级注入，串口不仿真）
                    lora_status   /lora/status     （观测 STA，串口不仿真）
                    remote_cmd    /grasp_hexapod/remote_cmd
    服务(应答)      switch_mode   /grasp_hexapod/switch_mode
                    gripper_act   /grasp_hexapod/gripper_act

模拟语义：时间线（landing/mode_* 完成时刻）、夹爪 clamp_fail/open_fail 注入、
RTK 协方差 cov_bad_windows、模式拒绝 switch_fail_mode —— 复用 bt_mock_world 的
ModeWorld/DEFAULT_TIMELINE（避免重复实现）。

LoRa 不仿真：/lora/command、/lora/status 由真实节点 reference/lora（lora_node.py）
提供，sim_feedback 不发布 LoRa（配置中 lora_* 固定 simulate:false）。

用法：
    rosrun grasp_hexapod_bt sim_feedback.py                       # 默认 config 全部模拟
    rosrun grasp_hexapod_bt sim_feedback.py _config:=<yaml>       # 指定接口开关
    python3 sim_feedback.py --selftest                            # 离线自检（不依赖 ROS）
"""

import argparse
import os
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import bt_mock_world
from bt_mock_world import ModeWorld, DEFAULT_TIMELINE, parse_cov_bad

DEFAULT_CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "config", "real_bt.yaml")

# 全部可模拟接口（缺省 simulate 开关）
DEFAULT_INTERFACES = {
    "sensor_health": {"simulate": True, "topic": "/grasp_hexapod/sensor_health"},
    "encoder_state": {"simulate": True, "topic": "/grasp_hexapod/encoder_state"},
    "fix": {"simulate": True, "topic": "/fix"},
    "lora_command": {"simulate": True, "topic": "/lora/command"},
    "lora_status": {"simulate": True, "topic": "/lora/status"},
    "remote_cmd": {"simulate": True, "topic": "/grasp_hexapod/remote_cmd"},
    "switch_mode": {"simulate": True, "service": "/grasp_hexapod/switch_mode"},
    "gripper_act": {"simulate": True, "service": "/grasp_hexapod/gripper_act"},
}

SIM_DEFAULTS = {
    "mission": "recover",
    "remote_test": "",
    "landing_t": None,          # None -> 用 timeline["landing"]
    "deploy": None,             # None -> 用 timeline["deploy"]
    "winch_done": None,         # None -> 用 timeline["winch_done"]
    "clamp_fail": False,
    "open_fail": False,
    "switch_fail_mode": "",
    "cov_bad_windows": [],
    "sensor_bad": "",
}


def load_config(path=None):
    """读取 real_bt.yaml；缺失时用内置默认（全 simulate）。返回 (interfaces, sim)。"""
    import yaml
    path = path or DEFAULT_CONFIG_PATH
    interfaces = {k: dict(v) for k, v in DEFAULT_INTERFACES.items()}
    sim = dict(SIM_DEFAULTS)
    if os.path.isfile(path):
        with open(path) as f:
            cfg = yaml.safe_load(f) or {}
        interfaces.update(cfg.get("interfaces", {}))
        sim.update(cfg.get("simulation", {}))
    else:
        print("[sim_feedback] 配置不存在，使用内置默认（可模拟接口全开，含 LoRa 话题级；串口不仿真）: %s" % path)
    return interfaces, sim


class SimClock:
    """给 ModeWorld 用的“节点句柄”：now/mode_done_at/失败注入。"""

    def __init__(self, sim, start_wall=None):
        import rospy
        self.rospy = rospy
        self.sim = sim
        self.timeline = dict(DEFAULT_TIMELINE)
        for key, val in (("landing", sim.get("landing_t")),
                         ("deploy", sim.get("deploy")),
                         ("winch_done", sim.get("winch_done")),
                         ("home_cmd", sim.get("home_cmd"))):
            if val is not None:
                self.timeline[key] = float(val)
        self.start = rospy.get_time() if start_wall is None else start_wall
        self.clamp_fail = bool(sim.get("clamp_fail", False))
        self.open_fail = bool(sim.get("open_fail", False))
        self.switch_fail_mode = sim.get("switch_fail_mode", "")

    @property
    def now(self):
        return self.rospy.get_time() - self.start

    def mode_done_at(self, key):
        return self.timeline.get(key, 1e9)


def activate(interfaces=None, sim=None, verbose=True):
    """在当前已 init 的 rospy 节点上，为 simulate:true 的接口挂载模拟源。

    返回 SimRuntime（便于 run_real_bt 关闭/查询）。
    """
    import rospy
    from std_msgs.msg import String
    from sensor_msgs.msg import NavSatFix
    from grasp_hexapod_msgs.msg import (SensorHealthArray, SensorHealth,
                                        EncoderState, RemoteCmd)
    from grasp_hexapod_msgs.srv import (SwitchMode, SwitchModeResponse,
                                        GripperAct, GripperActResponse)
    from sensor_health_monitor import DEFAULT_CONFIG as HEALTH_CONFIG

    interfaces = interfaces or {}
    sim = sim or {}
    clock = SimClock(sim)
    mode_world = ModeWorld(clock)
    rt = SimRuntime()
    rt.clock = clock
    rt.mode_world = mode_world
    log = rospy.loginfo

    # ---- sensor_health：全健康（频率新鲜） ----
    if interfaces.get("sensor_health", {}).get("simulate"):
        names = list(HEALTH_CONFIG.keys())
        topic = interfaces["sensor_health"]["topic"]

        pub_health = rospy.Publisher(topic, SensorHealthArray, queue_size=2)

        def _pub_health(_pub=pub_health):
            msg = SensorHealthArray()
            msg.header.stamp = rospy.Time.now()
            for name in sorted(names):
                h = SensorHealth()
                h.name = name
                h.online = True
                h.fresh = True
                h.freq_hz = 50.0
                h.age_s = 0.02
                msg.sensors.append(h)
            _pub.publish(msg)

        rt.timers.append(rospy.Timer(rospy.Duration(0.5), lambda _e: _pub_health()))
        rt.pubs.append(pub_health)
        log("[sim] sensor_health 模拟发布（全健康）@%s", topic)

    # ---- encoder_state：not_landed -> landed（时间线） ----
    if interfaces.get("encoder_state", {}).get("simulate"):
        topic = interfaces["encoder_state"]["topic"]

        pub_enc = rospy.Publisher(topic, EncoderState, queue_size=2)

        def _pub_enc(_pub=pub_enc):
            landed = clock.now >= clock.mode_done_at("landing")
            m = EncoderState()
            m.header.stamp = rospy.Time.now()
            m.landed = bool(landed)
            m.angle = 135.0 if landed else 45.0
            m.reason = "已落地" if landed else "未落地"
            _pub.publish(m)

        rt.timers.append(rospy.Timer(rospy.Duration(0.1), lambda _e: _pub_enc()))
        rt.pubs.append(pub_enc)
        log("[sim] encoder_state 模拟发布 @%s（落地 t=%s）",
            topic, clock.mode_done_at("landing"))

    # ---- fix：RTK 协方差（cov_bad_windows 内 9.0，否则 0.01） ----
    if interfaces.get("fix", {}).get("simulate"):
        topic = interfaces["fix"]["topic"]
        windows = parse_cov_bad(sim.get("cov_bad_windows") or "")

        pub_fix = rospy.Publisher(topic, NavSatFix, queue_size=2)

        def _pub_fix(_pub=pub_fix):
            f = NavSatFix()
            f.header.stamp = rospy.Time.now()
            f.status.status = 4
            cov = 9.0 if any(s <= clock.now <= e for s, e in windows) else 0.01
            f.position_covariance = [cov, 0, 0, 0, cov, 0, 0, 0, cov]
            f.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            _pub.publish(f)

        rt.timers.append(rospy.Timer(rospy.Duration(0.1), lambda _e: _pub_fix()))
        rt.pubs.append(pub_fix)
        log("[sim] fix 模拟发布 @%s（cov_bad=%s）", topic, windows)

    # ---- lora_command：**话题级**时间线注入 CMD（串口读取不仿真） ----
    if interfaces.get("lora_command", {}).get("simulate"):
        topic = interfaces["lora_command"]["topic"]
        pub_lora = rospy.Publisher(topic, String, queue_size=5)
        rt.pubs.append(pub_lora)
        rt._lora = {"pub": pub_lora, "cls": String, "sent": set()}
        mission = sim.get("mission", "recover")

        def _tick_lora(_e=None):
            t = clock.now
            due = []
            if "task_cmd" not in rt._lora["sent"] and t >= 2.0:
                due.append(("task_cmd", "CMD,HEX,{},NOW".format(mission.upper())))
            if "deploy" not in rt._lora["sent"] and t >= clock.mode_done_at("deploy"):
                due.append(("deploy", "CMD,HEX,DEPLOY,NOW"))
            if "winch" not in rt._lora["sent"] and t >= clock.mode_done_at("winch_done"):
                due.append(("winch", "CMD,HEX,HOIST_DONE,NOW"))
            if "home_cmd" not in rt._lora["sent"] and t >= clock.mode_done_at("home_cmd"):
                due.append(("home_cmd", "CMD,HEX,HOME,NOW"))
            for key, frame in due:
                rt._lora["sent"].add(key)
                rt._lora["pub"].publish(rt._lora["cls"](data=frame))
                log("[sim lora TX] %s", frame)

        rt.timers.append(rospy.Timer(rospy.Duration(0.1), _tick_lora))
        log("[sim] lora_command 话题级注入 @%s（mission=%s；串口读取不仿真）",
            topic, mission)

    # ---- lora_status：观测 STA（串口不仿真，真实 lora_node 才写串口） ----
    if interfaces.get("lora_status", {}).get("simulate"):
        topic = interfaces["lora_status"]["topic"]

        def on_sta(msg):
            rospy.loginfo("[sim lora RX] %s", str(msg.data).strip())

        rospy.Subscriber(topic, String, on_sta, queue_size=10)
        log("[sim] lora_status 观测 @%s（不写串口）", topic)

    # ---- remote_cmd：remote_test 目标持续发布（否则不发，读侧回退 idle） ----
    if interfaces.get("remote_cmd", {}).get("simulate"):
        topic = interfaces["remote_cmd"]["topic"]
        pub_remote = rospy.Publisher(topic, RemoteCmd, queue_size=2)
        rt.pubs.append(pub_remote)
        target = sim.get("remote_test", "")
        if target:
            def _pub_remote(_e=None, _pub=pub_remote):
                m = RemoteCmd()
                m.mode = target
                _pub.publish(m)
            rt.timers.append(rospy.Timer(rospy.Duration(0.2), _pub_remote))
            log("[sim] remote_cmd 模拟发布：mode=%s", target)
        rt._remote_pub = pub_remote

    # ---- switch_mode 服务：最终结果（阻塞式语义的模拟实现） ----
    if interfaces.get("switch_mode", {}).get("simulate"):
        srv = interfaces["switch_mode"]["service"]

        def on_switch(req):
            mode = req.target_mode
            if mode not in ModeWorld.MODE_NAMES:
                return SwitchModeResponse(success=False,
                                          message="未知模式 {}".format(mode))
            if mode == clock.switch_fail_mode:
                return SwitchModeResponse(success=False,
                                          message="模式 {} 被拒绝".format(mode))
            if mode_world.active_mode != mode:
                mode_world.active_mode = mode
                rt.switch_log.append((clock.now, mode))
                log("[sim] switch_mode -> %s（t=%.2fs）", mode, clock.now)
            # 阻塞至该模式终态（与真实阻塞式 mode_server 语义一致），
            # 期间不占用本服务线程外的资源；行为树侧同步等待最终结果。
            while not rospy.is_shutdown():
                state, message = mode_world.query_state(mode)
                if state in ("SUCCESS", "FAILED"):
                    break
                rospy.sleep(0.05)
            return SwitchModeResponse(success=state == "SUCCESS", message=message)

        rt.serv_switch = rospy.Service(srv, SwitchMode, on_switch)
        log("[sim] switch_mode 服务模拟：%s", srv)

    # ---- gripper_act 服务：open/clamp ----
    if interfaces.get("gripper_act", {}).get("simulate"):
        srv = interfaces["gripper_act"]["service"]

        def on_gripper(req):
            if req.action not in ("open", "clamp"):
                return GripperActResponse(success=False, message="未知 action")
            mode = "release" if req.action == "open" else "dock"
            if req.action == "clamp" and clock.clamp_fail:
                return GripperActResponse(
                    success=False, message="夹爪受限/open复位后仍失败")
            if req.action == "open" and clock.open_fail:
                return GripperActResponse(
                    success=False, message="夹爪离线/松开超时")
            rt.gripper_calls.append((clock.now, mode, req.action))
            return GripperActResponse(success=True, message="ok")

        rt.serv_gripper = rospy.Service(srv, GripperAct, on_gripper)
        log("[sim] gripper_act 服务模拟：%s", srv)

    return rt


class SimRuntime:
    """activate() 返回的运行句柄：发布器/服务/定时器/开关记录。"""

    def __init__(self):
        self.clock = None
        self.mode_world = None
        self.pubs = []
        self.timers = []
        self.serv_switch = None
        self.serv_gripper = None
        self.switch_log = []
        self.gripper_calls = []


def main():
    parser = argparse.ArgumentParser(description="模拟反馈节点（按 yaml 逐接口模拟）")
    parser.add_argument("--selftest", action="store_true", help="离线自检（不依赖 ROS）")
    args, _ = parser.parse_known_args()
    if args.selftest:
        selftest()
        return
    run()


def run():
    import rospy
    rospy.init_node("sim_feedback", anonymous=True)
    interfaces, sim = load_config(rospy.get_param("~config", DEFAULT_CONFIG_PATH))
    # rosparam 覆盖（与 bt_mock_world 同风格）
    for key in ("mission", "remote_test", "clamp_fail", "open_fail",
                "switch_fail_mode", "cov_bad_windows", "landing_t"):
        v = rospy.get_param("~" + key, None)
        if v is not None:
            sim[key] = v
    rt = activate(interfaces, sim)
    rospy.loginfo("sim_feedback 就绪：模拟接口 %s（LoRa 为话题级仿真，串口读取不仿真）",
                  [k for k, v in interfaces.items() if v.get("simulate")])
    rospy.spin()


def selftest():
    """离线：时间线/模式状态机/夹爪注入一致性（不依赖 ROS）。"""
    class FakeClock:
        """无 ROS 的 SimClock 替代（供 ModeWorld 直接测试）。"""
        clamp_fail = False
        open_fail = False
        sim = dict(SIM_DEFAULTS)

        def __init__(self, timeline_extra=None):
            self.timeline = dict(DEFAULT_TIMELINE)
            if timeline_extra:
                self.timeline.update(timeline_extra)
            self.now = 10.0

        def mode_done_at(self, key):
            return self.timeline.get(key, 1e9)

    clock = FakeClock()
    world = ModeWorld(clock)
    assert world.query_state("home") == ("SUCCESS", "")       # home done @5
    assert world.query_state("release") == ("RUNNING", "")    # release done @11
    clock.now = 12.0
    assert world.query_state("release") == ("SUCCESS", "")
    print("[OK] 时间线/模式状态机（复用 bt_mock_world.ModeWorld）")

    # 夹爪失败注入（dock 完成前触发）
    clock2 = FakeClock()
    clock2.clamp_fail = True
    clock2.now = 64.0
    world2 = ModeWorld(clock2)
    assert world2.query_state("dock") == ("FAILED", "夹爪受限/open复位后仍失败")
    print("[OK] dock 夹爪失败注入")

    # yaml 缺省接口清单可加载（LoRa 为话题级仿真；串口读取不仿真）
    interfaces, sim = load_config("/nonexistent.yaml")
    assert "lora_command" in interfaces and "lora_status" in interfaces
    assert all(v["simulate"] for v in interfaces.values())
    assert sim["mission"] == "recover"
    print("[OK] 配置缺省：可模拟接口全 simulate=true（含 LoRa 话题级；串口读取不仿真）")

    print("selftest 全部通过")


if __name__ == "__main__":
    main()
