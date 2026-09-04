#!/usr/bin/env python3
"""行为树联调模拟节点（统一模式执行版）：模拟模式/夹爪/传感器世界并 tick 行为树。

与 scripts/hexapod_bt.py 配套（hexapod_bt 为 ROS-free 纯逻辑，本节点实现其
BridgeContext 并模拟全部模式/服务）：
    - 所有运动/任务动作 = 模式（home/walk/climb/dock/spin_search/release/
      approach/tag_nav）；BT 一个模式一个 RunMode 节点，统一走
      ~/switch_mode（SwitchMode.srv）——服务自动执行该模式完整连续性流程，
      响应即【最终结果】（success=最终成功 + message=最终问题）。
    - 夹爪夹紧/松开 = 新建 ~/gripper_act（GripperAct.srv），由 release/dock
      模式内部调用，不在行为树中体现；到位结果折入 switch_mode 最终结果。
    - 编码器 = 持续发布 /grasp_hexapod/encoder_state（EncoderState topic，
      替代原服务），is_landing_confirmed/sensor_health 订阅它。
    - 主链不含遥控；遥控独立测试链入口：_remote_test:=mode（build_remote_test_tree）。
    - 传感器健康用真判据代码 sensor_health_monitor（发布即喂）。

用法：
    rosrun grasp_hexapod_bt bt_mock_world.py _mission:=recover    # 回收
    rosrun grasp_hexapod_bt bt_mock_world.py _mission:=release    # 释放
    rosrun grasp_hexapod_bt bt_mock_world.py _drop_sensor:=imu    # 拨传感器
    rosrun grasp_hexapod_bt bt_mock_world.py _rtk_cov_bad:=13-15  # RTK协方差停走
    rosrun grasp_hexapod_bt bt_mock_world.py _clamp_fail:=true    # dock夹紧失败
    rosrun grasp_hexapod_bt bt_mock_world.py _remote_test:=dock   # 遥控测试链
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import argparse

import hexapod_bt
import sensor_health_monitor
from sensor_health_monitor import SensorHealthRegistry, DEFAULT_CONFIG


DEFAULT_TIMELINE = {
    "task_cmd": 2.0,       # ⑤/⑲ 任务命令
    "deploy": 7.0,         # ⑨ 绞盘下放开始
    "landing": 9.0,        # ⑩/㉔ 编码器确认落地
    "winch_done": 80.0,    # ⑫/㉜ 绞盘回收完成
    "home_cmd": 82.0,      # 恢复初始命令（CMD,HEX,HOME,NOW）
    # 各模式完成时刻（模拟 control 端执行耗时）
    "mode_home": 5.0,          # 回到初始姿态(含复位)
    "mode_release": 11.0,      # 释放小蓝（夹爪 open 到位）
    "mode_spin_search": 13.0,  # 自转搜索（感知发现小蓝）
    "mode_approach": 22.0,     # 粗导航到可视 tag
    "mode_tag_nav": 24.0,      # tag 精导航到攀爬点
    "mode_climb": 45.0,        # 攀爬 C1→C35
    "mode_dock": 65.0,         # 对接（导引+抬腿+夹爪clamp）
}


def parse_cov_bad(spec):
    if not spec:
        return []
    windows = []
    for part in str(spec).split(","):
        if "-" in part:
            s, e = part.split("-")
            windows.append((float(s), float(e)))
        else:
            t = float(part)
            windows.append((t, t + 3.0))
    return windows


class ModeWorld:
    """模拟模式执行器：switch_mode 幂等切换；最终结果由时间线/夹爪驱动。

    release/dock 模式内部调用夹爪 ~/gripper_act（open/clamp），到位→SUCCESS；
    受限/离线/失败 → FAILED + 问题（见 ~clamp_fail / ~open_fail 注入）。
    """

    MODE_NAMES = ("home", "walk", "climb", "dock", "spin_search",
                  "release", "approach", "tag_nav")

    def __init__(self, node):
        self.node = node
        self.active_mode = None

    def query_state(self, mode):
        """返回 (state, message)：state ∈ RUNNING / SUCCESS / FAILED。"""
        # 夹爪失败注入 → 对应模式在完成前 FAILED
        if mode == "dock" and self.node.clamp_fail:
            t_done = self.node.mode_done_at("mode_dock")
            if self.node.now >= t_done - 2.0:
                return ("FAILED", "夹爪受限/open复位后仍失败")
        if mode == "release" and self.node.open_fail:
            t_done = self.node.mode_done_at("mode_release")
            if self.node.now >= t_done - 2.0:
                return ("FAILED", "夹爪离线/松开超时")
        if self.node.now >= self.node.mode_done_at("mode_" + mode):
            return ("SUCCESS", "")
        return ("RUNNING", "")


class MockRosBridge(hexapod_bt.BridgeContext):
    """按时间线实现桥接契约（话题/服务语义与 BT_INTERFACES.md 一致）。"""

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.hold_log = []

    @property
    def t(self):
        return self.node.now

    def _done(self, key):
        return self.t >= self.node.timeline[key]

    # ---- 传感器 ----
    def sensor_health(self):
        report = self.node.registry.snapshot(self.t)
        state = self.node.encoder_state()
        if state is None:          # 编码器拨断：无帧 -> 离线
            report["encoder"] = {"online": False, "fresh": False,
                                 "freq_hz": 0.0, "age_s": 99.0,
                                 "reason": "无 encoder_state 帧"}
        else:
            report["encoder"] = {"online": True, "fresh": True,
                                 "freq_hz": 0.0, "age_s": 0.0, "reason": ""}
        return report

    def is_landing_confirmed(self):
        state = self.node.encoder_state()
        if state is None:
            return False        # 无数据：视为未落地，继续等待
        return state["landed"]

    def rtk_covariance_ok(self):
        return not any(s <= self.t <= e for s, e in self.node.cov_bad_windows)

    def hold_motion(self, reason):
        # 段首打印，避免刷屏
        if not self.hold_log or self.t - self.hold_log[-1][0] > 0.35:
            self.node.log("  [HOLD] t={:6.2f}s 原因={}（停走等待恢复）".format(self.t, reason))
        self.hold_log.append((self.t, reason))

    # ---- 模式执行（统一 ~/switch_mode，返回最终结果三态） ----
    def switch_mode(self, target_mode):
        return self.node.switch_mode_service(target_mode)

    # ---- 任务/LoRa ----
    def receive_task_command(self):
        if not self._done("task_cmd"):
            return None
        return self.node.mission

    def wait_deployment(self, dt):
        return self._done("deploy")

    def wait_winch_hoisted(self, dt):
        return self._done("winch_done")

    def wait_home_cmd(self, dt):
        return self._done("home_cmd")

    def report_status(self, status, dt=0.0):
        super().report_status(status, dt)
        self.node.publish_status(status)

    # ---- 遥控（仅测试链） ----
    def read_remote_cmd(self):
        return {"mode": self.node.remote_test or "idle",
                "vx": 0.0, "vy": 0.0, "vyaw": 0.0,
                "reset_edge": False, "enable_edge": False,
                "climb_edge": False, "dock_edge": False}


def main():
    parser = argparse.ArgumentParser(description="行为树联调模拟节点（统一模式切换版，无硬件）")
    parser.add_argument("--selftest", action="store_true", help="离线自检（不依赖 ROS）")
    args, _ = parser.parse_known_args()
    if args.selftest:
        selftest()
        return
    run_node()


def run_node():
    import rospy
    from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32
    from sensor_msgs.msg import Imu, JointState, Joy, NavSatFix
    from std_msgs.msg import Bool, String
    from grasp_hexapod_msgs.srv import (SwitchMode, SwitchModeResponse,
                                        GripperAct, GripperActResponse)
    from grasp_hexapod_msgs.msg import (EncoderState, SensorHealthArray,
                                        BtStateArray, BtNodeState)
    from py_trees.common import Status

    rospy.init_node("bt_mock_world")
    cleanup_stale_params()

    class MockWorld:
        def __init__(self):
            self.now = 0.0
            self.start = rospy.get_time()
            self.mission = rospy.get_param("~mission", "recover")
            assert self.mission in ("release", "recover")
            self.remote_test = rospy.get_param("~remote_test", "")
            if self.remote_test:
                assert self.remote_test in ModeWorld.MODE_NAMES
            self.drop_sensor = rospy.get_param("~drop_sensor", "")
            self.drop_at = float(rospy.get_param("~drop_at", 6.0))
            self.clamp_fail = bool(rospy.get_param("~clamp_fail", False))
            self.open_fail = bool(rospy.get_param("~open_fail", False))
            self.switch_fail_mode = rospy.get_param("~switch_fail_mode", "")
            self.cov_bad_windows = parse_cov_bad(rospy.get_param("~rtk_cov_bad", ""))
            self.timeline = dict(DEFAULT_TIMELINE)
            for key in DEFAULT_TIMELINE:
                param = rospy.get_param("~" + key, None)
                if param is not None:
                    self.timeline[key] = float(param)
            self.tick_hz = float(rospy.get_param("~tick_hz", 30.0))
            self.verbose = bool(rospy.get_param("~verbose", True))
            self.mode_world = ModeWorld(self)
            self.switch_log = []
            self.gripper_calls = []
            self.registry = SensorHealthRegistry(DEFAULT_CONFIG)
            self.registry.start(0.0)

            # ---- 发布器 ----
            self.pub_imu = rospy.Publisher("/grasp_hexapod/imu", Imu, queue_size=5)
            self.pub_fix = rospy.Publisher("/fix", NavSatFix, queue_size=5)
            self.pub_base = rospy.Publisher(
                "/grasp_hexapod/navigation/base_pose", PoseStamped, queue_size=5)
            self.pub_xiaolan = rospy.Publisher(
                "/grasp_hexapod/navigation/xiaolan_pose", PoseStamped, queue_size=5)
            self.pub_boundary = rospy.Publisher(
                "/grasp_hexapod/navigation/pv_boundary", PolygonStamped, queue_size=5)
            self.pub_joints = {}
            for leg in ("lf", "lm", "lb", "rf", "rm", "rb"):
                self.pub_joints[leg] = rospy.Publisher(
                    "/{}_pos".format(leg), JointState, queue_size=5)
            self.pub_stereo = rospy.Publisher("/grasp_hexapod/stereo_ok", Bool, queue_size=5)
            self.pub_mono = rospy.Publisher("/grasp_hexapod/mono_ok", Bool, queue_size=5)
            self.pub_encoder = rospy.Publisher(
                "/grasp_hexapod/encoder_state", EncoderState, queue_size=2)
            self.pub_joy = rospy.Publisher("/joy", Joy, queue_size=5)
            self.pub_lora_cmd = rospy.Publisher("/lora/command", String, queue_size=5)
            self.pub_status = rospy.Publisher("/lora/status", String, queue_size=5)
            self.pub_health = rospy.Publisher(
                "/grasp_hexapod/sensor_health", SensorHealthArray, queue_size=2)
            self.pub_state = rospy.Publisher("/grasp_hexapod/state", String, queue_size=5)
            self.pub_bt = rospy.Publisher("/grasp_hexapod/bt_state", BtStateArray,
                                          queue_size=5)

            # ---- 订阅（观测） ----
            rospy.Subscriber("/lora/status", String, self.on_status, queue_size=5)

            # ---- 服务：模式 / 夹爪 ----
            self.srv_switch = rospy.Service(
                "~switch_mode", SwitchMode, self.on_switch_mode)
            self.srv_gripper = rospy.Service(
                "~gripper_act", GripperAct, self.on_gripper_act)

            # ---- 行为树 ----
            self.bridge = MockRosBridge(self)
            if self.remote_test:
                self.tree = hexapod_bt.build_remote_test_tree(self.bridge)
                rospy.loginfo("遥控测试链: 直接切换 %s", self.remote_test)
            else:
                self.tree = hexapod_bt.build_hexapod_tree(self.bridge)
            self.last_print = 0.0
            self.finished = False
            self.lora_sent = set()

        def mode_done_at(self, key):
            return self.timeline.get(key, 1e9)

        def log(self, text):
            rospy.loginfo("%s", text)

        # ---- 模式服务实现：返回该模式最终执行结果三态 ----
        def switch_mode_service(self, target_mode):
            """统一模式执行：进入并自动执行；返回 (state, message)。

            state ∈ RUNNING（执行中）/ SUCCESS（成功）/ FAILED（失败+问题）。
            幂等：已在目标模式返回其当前状态、不重复触发。
            """
            if target_mode not in ModeWorld.MODE_NAMES:
                return ("FAILED", "未知模式 {}".format(target_mode))
            if target_mode == self.switch_fail_mode:
                return ("FAILED", "模式 {} 被拒绝".format(target_mode))
            if self.mode_world.active_mode != target_mode:
                self.mode_world.active_mode = target_mode
                self.switch_log.append((self.now, target_mode))
                self.log("  [MODE] t={:6.2f}s switch_mode -> {}".format(self.now, target_mode))
                # release/dock 模式内部触发夹爪动作（松开/夹紧，不在树中体现）
                if target_mode == "release":
                    self.call_gripper_in_mode("open", mode="release")
                elif target_mode == "dock":
                    self.call_gripper_in_mode("clamp", mode="dock")
            return self.mode_world.query_state(target_mode)

        def on_switch_mode(self, req):
            state, message = self.switch_mode_service(req.target_mode)
            # 服务对外返回【最终结果】：success = 模式最终成功（阻塞式语义）
            success = state == "SUCCESS"
            return SwitchModeResponse(success=success, message=message)

        # ---- 夹爪服务（模式内部调用；~gripper_act） ----
        def call_gripper_in_mode(self, action, mode):
            success, message = self.gripper_act_impl(action, mode)
            self.log("  [GRIP] t={:6.2f}s {} 模式内部 {} -> {} {}".format(
                self.now, mode, action,
                "到位" if success else "失败", message or ""))

        def gripper_act_impl(self, action, mode):
            self.gripper_calls.append((self.now, mode, action))
            if action == "clamp" and self.clamp_fail and mode == "dock":
                return False, "夹爪受限/open复位后仍失败"
            if action == "open" and self.open_fail and mode == "release":
                return False, "夹爪离线/松开超时"
            return True, "ok"

        def on_gripper_act(self, req):
            if req.action not in ("open", "clamp"):
                return GripperActResponse(success=False, message="未知 action")
            mode = "release" if req.action == "open" else "dock"
            success, message = self.gripper_act_impl(req.action, mode)
            return GripperActResponse(success=success, message=message)

        # ---- 编码器（落地判断，持续发布 EncoderState topic；拨断则不发布） ----
        def encoder_state(self):
            if self.drop_sensor == "encoder" and self.now >= self.drop_at:
                return None
            landed = self.now >= self.timeline["landing"]
            return {"landed": landed,
                    "angle": 135.0 if landed else 45.0,
                    "reason": "已落地" if landed else "未落地"}

        # ---- 原始传感器发布（发布即喂健康注册表） ----
        def publish_sensors(self):
            stamp = rospy.Time.now()
            if self.drop_sensor != "imu" or self.now < self.drop_at:
                msg = Imu()
                msg.header.stamp = stamp
                msg.header.frame_id = "base_link"
                self.pub_imu.publish(msg)
                self.registry.feed("imu", "/grasp_hexapod/imu", self.now)
            if self.drop_sensor != "gps" or self.now < self.drop_at:
                fix = NavSatFix()
                fix.header.stamp = stamp
                fix.status.status = 4
                cov = 9.0 if any(s <= self.now <= e for s, e in self.cov_bad_windows) else 0.01
                fix.position_covariance = [cov, 0, 0, 0, cov, 0, 0, 0, cov]
                fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                self.pub_fix.publish(fix)
                self.registry.feed("gps", "/fix", self.now)
            if self.drop_sensor != "rtk" or self.now < self.drop_at:
                pose = PoseStamped()
                pose.header.stamp = stamp
                pose.header.frame_id = "pv_map"
                self.pub_base.publish(pose)
                self.pub_xiaolan.publish(pose)
                boundary = PolygonStamped()
                boundary.header.stamp = stamp
                boundary.header.frame_id = "pv_map"
                boundary.polygon.points = [Point32(0.0, 0.0, 0.0)]
                self.pub_boundary.publish(boundary)
                for topic in ("/grasp_hexapod/navigation/base_pose",
                              "/grasp_hexapod/navigation/xiaolan_pose",
                              "/grasp_hexapod/navigation/pv_boundary"):
                    self.registry.feed("rtk", topic, self.now)
            if self.drop_sensor != "servo" or self.now < self.drop_at:
                for leg, pub in self.pub_joints.items():
                    joint = JointState()
                    joint.header.stamp = stamp
                    joint.name = ["j1", "j2", "j3"]
                    joint.position = [0.0, 0.5, -1.0]
                    pub.publish(joint)
                    self.registry.feed("servo", "/{}_pos".format(leg), self.now)
            for name in ("stereo", "mono"):
                if self.drop_sensor != name or self.now < self.drop_at:
                    pub = self.pub_stereo if name == "stereo" else self.pub_mono
                    pub.publish(Bool(data=True))
                    self.registry.feed(name, "/grasp_hexapod/{}_ok".format(name), self.now)
            # 编码器状态持续发布（topic，替代原服务；拨断则不发布）
            enc = self.encoder_state()
            if enc is not None:
                enc_msg = EncoderState()
                enc_msg.header.stamp = stamp
                enc_msg.landed = bool(enc["landed"])
                enc_msg.angle = enc["angle"]
                enc_msg.reason = enc["reason"]
                self.pub_encoder.publish(enc_msg)
            # /lora/command 时间线
            for key, frame in (("task_cmd", "CMD,HEX,{},NOW".format(self.mission.upper())),
                               ("deploy", "CMD,HEX,DEPLOY,NOW"),
                               ("winch_done", "CMD,HEX,HOIST_DONE,NOW"),
                               ("home_cmd", "CMD,HEX,HOME,NOW")):
                if key not in self.lora_sent and self.now >= self.timeline[key]:
                    self.lora_sent.add(key)
                    self.pub_lora_cmd.publish(String(data=frame))
                    self.log("  [LoRa->] t={:6.2f}s {}".format(self.now, frame))

        def publish_health(self):
            from grasp_hexapod_msgs.msg import SensorHealth
            msg = SensorHealthArray()
            msg.header.stamp = rospy.Time.now()
            for name, state in sorted(self.registry.snapshot(self.now).items()):
                health = SensorHealth()
                health.name = name
                health.online = bool(state["online"])
                health.fresh = bool(state["fresh"])
                health.freq_hz = state["freq_hz"]
                health.age_s = state["age_s"] if state["age_s"] != float("inf") else 999.0
                health.reason = state["reason"]
                msg.sensors.append(health)
            self.pub_health.publish(msg)

        def publish_status(self, status):
            frame = "STA,HEX,{},0.00,0.00".format(status)
            self.pub_status.publish(String(data=frame))
            self.log("  [LoRa<-] t={:6.2f}s {}".format(self.now, frame))

        def on_status(self, message):
            pass

        def publish_bt_state(self):
            """发布 /grasp_hexapod/bt_state（≤5Hz；状态变化或终态立即发）。"""
            mission_status = (self.bridge.status_log[-1]
                              if self.bridge.status_log else "")
            tree_label = "遥控测试链" if self.remote_test else "主链"
            snap = hexapod_bt.snapshot_tree(
                self.tree, mission_status=mission_status, tree_name=tree_label)
            key = (snap["root_status"], snap["mission_status"],
                   snap["active_phase"])
            state = self.tree.status
            if (state != Status.RUNNING or key != getattr(self, "_bt_key", None)
                    or self.now - getattr(self, "_bt_t", -1.0) >= 0.2):
                msg = BtStateArray()
                msg.header.stamp = rospy.Time.now()
                msg.tree_name = snap["tree_name"]
                msg.root_status = snap["root_status"]
                msg.mission_status = snap["mission_status"]
                msg.active_phase = snap["active_phase"]
                msg.active_feedback = snap["active_feedback"]
                for n in snap["nodes"]:
                    entry = BtNodeState()
                    entry.name = n["name"]
                    entry.status = n["status"]
                    entry.feedback = n["feedback"]
                    entry.depth = n["depth"]
                    entry.is_leaf = n["is_leaf"]
                    msg.nodes.append(entry)
                self.pub_bt.publish(msg)
                self._bt_key = key
                self._bt_t = self.now

        # ---- 主循环 ----
        def tick(self, _event=None):
            if self.finished:
                return
            self.now = rospy.get_time() - self.start
            self.publish_sensors()
            if int(self.now * 5) != int(self.last_print * 5):
                self.publish_health()
            dt = 1.0 / self.tick_hz
            for node in self.tree.iterate():
                node.dt = dt
            self.tree.tick_once()
            state = self.tree.status
            if self.verbose and int(self.now * 1) != int(self.last_print * 1):
                running = [n for n in self.tree.iterate()
                           if n.status == Status.RUNNING and not n.children]
                tip = running[-1] if running else None
                if tip:
                    self.log("t={:6.2f}s [{}] {}".format(
                        self.now, tip.name, tip.feedback_message))
                self.last_print = self.now
            self.pub_state.publish(String(data=state.name))
            self.publish_bt_state()
            if state != Status.RUNNING:
                self.finished = True
                self.log("=" * 60)
                self.log("行为树终态: {}   状态上报顺序: {}".format(
                    state.name, self.bridge.status_log))
                expected = {
                    "release": ["RELEASED", "RESET_DONE", "DONE"],
                    "recover": ["LANDED", "CLAMPED", "RESET_DONE", "DONE"],
                }.get(self.mission, [])
                if state == Status.SUCCESS and self.bridge.status_log == expected:
                    self.log("[通过] {} 任务按预期完成".format(self.mission))
                elif state == Status.SUCCESS and self.bridge.status_log[-1:] == ["FAILED"]:
                    self.log("[按预期] 任务失败回退（拨动传感器/受限等场景）")
                else:
                    self.log("[注意] 顺序 {} 与预期 {} 不符".format(
                        self.bridge.status_log, expected))
                modes = [m for _, m in self.switch_log]
                self.log("模式切换序列: {}（{} 次）".format(modes, len(modes)))
                rospy.signal_shutdown("mock world finished")

    world = MockWorld()
    rospy.Timer(rospy.Duration(1.0 / world.tick_hz), world.tick)
    rospy.loginfo("模拟世界就绪: mission=%s remote=%s drop_sensor=%s cov_bad=%s "
                  "clamp_fail=%s tick=%gHz（落地 t=%gs）",
                  world.mission, world.remote_test or "无", world.drop_sensor or "无",
                  world.cov_bad_windows, world.clamp_fail, world.tick_hz,
                  world.timeline["landing"])
    rospy.spin()


def cleanup_stale_params(argv=None):
    """删除本节点私有命名空间下、本次 argv 未显式设置的陈旧参数。"""
    import rospy
    argv = list(sys.argv[1:] if argv is None else argv)
    argv_keys = set()
    for arg in argv:
        if arg.startswith("_") and ":=" in arg:
            argv_keys.add(arg[1:].split(":=", 1)[0])
    prefix = rospy.get_name() + "/"
    for name in rospy.get_param_names():
        if name.startswith(prefix):
            key = name[len(prefix):]
            if key not in argv_keys:
                try:
                    rospy.delete_param(name)
                except Exception:  # noqa: BLE001
                    pass


def selftest():
    """不依赖 ROS 的离线自检：时间线/模式状态机一致性。"""
    assert parse_cov_bad("13-16") == [(13.0, 16.0)]
    assert parse_cov_bad("") == []
    print("[OK] rtk_cov_bad 解析")

    class FakeNode:
        now = 10.0
        mission = "recover"
        remote_test = ""
        clamp_fail = False
        open_fail = False
        switch_fail_mode = ""
        cov_bad_windows = []
        timeline = dict(DEFAULT_TIMELINE)

        def mode_done_at(self, key):
            return self.timeline.get(key, 1e9)

        def encoder_state(self):
            landed = self.now >= self.timeline["landing"]
            return {"landed": landed,
                    "angle": 135.0 if landed else 45.0,
                    "reason": "已落地" if landed else "未落地"}

        def publish_status(self, status):
            pass

        def log(self, _text):
            pass

        def switch_mode_service(self, mode):
            return ("SUCCESS", "")

    world = ModeWorld(FakeNode())
    # t=10：home(5) SUCCESS、release(11) RUNNING
    assert world.query_state("home") == ("SUCCESS", "")
    assert world.query_state("release") == ("RUNNING", "")
    print("[OK] 模式状态机时间线语义")
    print("selftest 全部通过")


if __name__ == "__main__":
    main()
