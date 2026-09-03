#!/usr/bin/env python3
"""六足行为树 · 实机运行入口（纯真实桥接，**不含任何仿真**）。

原则：树节点不仿真；若某个真实节点缺失需要模拟回退，请**单独运行**
sim_feedback.py（按 config/real_bt.yaml 对 simulate:true 的接口模拟发布/
应答）。本脚本只做三件事：订阅标准名话题 → 调标准名服务 → tick 行为树。

标准名接口（契约见 src/docs/BT_INTERFACES.md）：
    话题：/grasp_hexapod/sensor_health、/grasp_hexapod/encoder_state、
          /fix、/lora/command、/lora/status、/grasp_hexapod/remote_cmd
    服务：/grasp_hexapod/switch_mode（阻塞式，返回最终 success/message）、
          /grasp_hexapod/gripper_act（release/dock 模式内部调用）

依赖的真实节点/服务：
    - encoder_driver/encoder_status_node → /grasp_hexapod/encoder_state
    - grasp_hexapod_bt/sensor_health_monitor → /grasp_hexapod/sensor_health
    - GPS/RTK 驱动 → /fix
    - lora（reference/lora）→ /lora/command、/lora/status
    - 控制栈 mode_server → /grasp_hexapod/switch_mode（真实执行端，尚未实现）
    - remote_control（遥控测试链用）→ /grasp_hexapod/remote_cmd
    （任一缺失且未用 sim_feedback 模拟 → 树将等待（如传感器上线）或失败回退）

用法：
    rosrun grasp_hexapod_bt run_real_bt.py                     # 主链任务
    rosrun grasp_hexapod_bt run_real_bt.py _loop:=false        # 任务结束即退出
    rosrun grasp_hexapod_bt run_real_bt.py _remote_test:=dock  # 遥控测试链

配套（独立进程，按需运行）：
    rosrun grasp_hexapod_bt sim_feedback.py                    # 缺啥补啥的仿真
"""

import argparse
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import hexapod_bt
from py_trees.common import Status

SWITCH_MODE_SERVICE = "/grasp_hexapod/switch_mode"


class RosBridgeContext(hexapod_bt.BridgeContext):
    """真实机器桥接：只订阅/调用标准名话题与服务（无仿真逻辑）。"""

    def __init__(self, node):
        super().__init__()
        self.n = node
        self._lock = threading.Lock()
        self._encoder = None
        self._sensor_health = None
        self._fix = None
        self._remote = None
        self._task = None
        self._deploy_done = False
        self._winch_done = False

    # ---- 话题缓存 ----
    def on_sensor_health(self, msg):
        with self._lock:
            self._sensor_health = msg

    def on_encoder_state(self, msg):
        with self._lock:
            self._encoder = {"normal": bool(msg.normal),
                             "landed": bool(msg.landed),
                             "not_landed": bool(msg.not_landed),
                             "angle": msg.angle, "reason": msg.reason}

    def on_fix(self, msg):
        with self._lock:
            self._fix = msg

    def on_remote_cmd(self, msg):
        with self._lock:
            self._remote = msg

    def on_lora_command(self, msg):
        text = str(msg.data).strip()
        fields = text.split(",")
        if len(fields) < 3 or fields[0] != "CMD":
            self.n.logwarn("忽略非 CMD 帧: %s", text)
            return
        op = fields[2].upper()
        with self._lock:
            if op == "RELEASE":
                self._task = "release"
            elif op == "RECOVER":
                self._task = "recover"
            elif op == "DEPLOY":
                self._deploy_done = True
            elif op == "HOIST_DONE":
                self._winch_done = True
            else:
                self.n.loginfo("未知指令透传: %s", text)

    # ---- 传感器 ----
    def sensor_health(self):
        with self._lock:
            array = self._sensor_health
            enc = self._encoder
        report = {}
        if array is not None:
            for h in array.sensors:
                report[h.name] = {"online": bool(h.online),
                                  "fresh": bool(h.fresh),
                                  "freq_hz": h.freq_hz,
                                  "age_s": h.age_s,
                                  "reason": h.reason}
        if enc is not None:
            report["encoder"] = {"online": enc["normal"], "fresh": enc["normal"],
                                 "freq_hz": 0.0, "age_s": 0.0,
                                 "reason": "" if enc["normal"] else enc["reason"]}
        else:
            report["encoder"] = {"online": False, "fresh": False, "freq_hz": 0.0,
                                 "age_s": 99.0,
                                 "reason": "无 /grasp_hexapod/encoder_state"}
        return report

    def is_landing_confirmed(self):
        with self._lock:
            enc = self._encoder
        if enc is None:
            return False
        if not enc["normal"]:
            return None
        return enc["landed"]

    def rtk_covariance_ok(self):
        with self._lock:
            fix = self._fix
        if fix is None:
            self.n.logwarn_throttle(30.0, "未收到 /fix，协方差监护默认放行")
            return True
        diag = [fix.position_covariance[i * 3 + i] for i in range(3)]
        return max(diag) <= self.n.rtk_max_cov

    def hold_motion(self, reason):
        self.n.logwarn_throttle(5.0, "[HOLD] 请求停走 reason=%s"
                                      "（协方差超限，依赖模式执行端停走）", reason)

    # ---- 模式执行（阻塞式服务，响应即最终结果） ----
    def switch_mode(self, target_mode):
        import rospy
        if rospy.is_shutdown():
            return ("RUNNING", "shutdown")
        if target_mode not in hexapod_bt.MODE_LABELS:
            return ("FAILED", "未知模式 {}".format(target_mode))
        proxy = self.n.switch_proxy()
        if proxy is None:
            return ("FAILED", "{} 不可用（需实机 mode_server 或 sim_feedback）"
                    .format(SWITCH_MODE_SERVICE))
        try:
            resp = proxy(target_mode)
        except Exception as exc:  # noqa: BLE001
            if rospy.is_shutdown():
                return ("RUNNING", "shutdown")
            self.n.logerr("%s(%s) 调用失败: %s",
                          SWITCH_MODE_SERVICE, target_mode, exc)
            return ("FAILED", "switch 调用失败: {}".format(exc))
        return ("SUCCESS" if resp.success else "FAILED", resp.message)

    # ---- 任务 / LoRa ----
    def receive_task_command(self):
        with self._lock:
            task = self._task
            self._task = None
        return task

    def wait_deployment(self, dt):
        with self._lock:
            done = self._deploy_done
            self._deploy_done = False
        return done

    def wait_winch_hoisted(self, dt):
        with self._lock:
            done = self._winch_done
            self._winch_done = False
        return done

    def report_status(self, status, dt=0.0):
        super().report_status(status, dt)
        x, y = self.n.pos_xy()
        if x is None:
            frame = "STA,HEX,{},0.00,0.00".format(status)
        else:
            frame = "STA,HEX,{},{:.2f},{:.2f}".format(status, float(x), float(y))
        self.n.pub_status.publish(self.n.String(data=frame))
        self.n.loginfo("[LoRa TX] %s", frame)

    # ---- 遥控（仅测试链） ----
    def read_remote_cmd(self):
        with self._lock:
            msg = self._remote
        if msg is None:
            return {"mode": "idle", "vx": 0.0, "vy": 0.0, "vyaw": 0.0,
                    "reset_edge": False, "enable_edge": False,
                    "climb_edge": False, "dock_edge": False}
        return {"mode": msg.mode, "vx": msg.vx, "vy": msg.vy, "vyaw": msg.vyaw,
                "reset_edge": bool(msg.reset_edge),
                "enable_edge": bool(msg.enable_edge),
                "climb_edge": bool(msg.climb_edge),
                "dock_edge": bool(msg.dock_edge)}


def main():
    parser = argparse.ArgumentParser(description="六足行为树实机运行入口（纯真实，无仿真）")
    parser.add_argument("--selftest", action="store_true",
                        help="离线校验模块可导入（不依赖 ROS）")
    args, _ = parser.parse_known_args()
    if args.selftest:
        print("[OK] run_real_bt 模块导入正常（纯真实桥接；仿真请单独运行 sim_feedback.py）")
        return
    run()


def run():
    import rospy
    from std_msgs.msg import String
    from sensor_msgs.msg import NavSatFix
    from grasp_hexapod_msgs.msg import SensorHealthArray, EncoderState, RemoteCmd
    from grasp_hexapod_msgs.srv import SwitchMode

    rospy.init_node("run_real_bt", anonymous=True)
    rate_hz = float(rospy.get_param("~rate", 30.0))
    loop = bool(rospy.get_param("~loop", True))
    remote_test = rospy.get_param("~remote_test", "")
    log_tip_s = float(rospy.get_param("~log_tip_s", 1.0))
    rtk_max_cov = float(rospy.get_param("~rtk_max_covariance", 0.04))
    verbose = bool(rospy.get_param("~verbose", True))
    pos_x = rospy.get_param("~x", None)
    pos_y = rospy.get_param("~y", None)

    # ---- 真实桥接（统一标准名）：node 句柄 = 普通对象 + 闭包方法 ----
    node = type("Node", (), {})()
    node.String = String
    node.rtk_max_cov = rtk_max_cov
    node.pub_status = rospy.Publisher("/lora/status", String, queue_size=10)
    node.pos_xy = lambda: (pos_x, pos_y)
    node.loginfo = lambda *a: rospy.loginfo(*a)
    node.logwarn = lambda *a: rospy.logwarn(*a)
    node.logerr = lambda *a: rospy.logerr(*a)
    node.logwarn_throttle = lambda t, *a: rospy.logwarn_throttle(t, *a)

    def _switch_proxy():
        try:
            return rospy.ServiceProxy(SWITCH_MODE_SERVICE, SwitchMode)
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn("创建 %s 代理失败: %s", SWITCH_MODE_SERVICE, exc)
            return None

    node.switch_proxy = _switch_proxy

    bridge = RosBridgeContext(node)
    rospy.Subscriber("/grasp_hexapod/sensor_health", SensorHealthArray,
                     bridge.on_sensor_health, queue_size=5)
    rospy.Subscriber("/grasp_hexapod/encoder_state", EncoderState,
                     bridge.on_encoder_state, queue_size=5)
    rospy.Subscriber("/fix", NavSatFix, bridge.on_fix, queue_size=5)
    rospy.Subscriber("/lora/command", String, bridge.on_lora_command,
                     queue_size=20)
    rospy.Subscriber("/grasp_hexapod/remote_cmd", RemoteCmd,
                     bridge.on_remote_cmd, queue_size=5)

    rospy.sleep(0.5)   # 等真实节点首帧
    while not rospy.is_shutdown():
        bridge.status_log = []
        if remote_test:
            tree = hexapod_bt.build_remote_test_tree(bridge)
            rospy.loginfo("行为树启动: 遥控测试链 %s（loop=%s）", remote_test, loop)
        else:
            tree = hexapod_bt.build_hexapod_tree(bridge)
            rospy.loginfo("行为树启动: 主链（loop=%s，等待 LoRa 任务命令）", loop)
        last_tip = time.time()
        finished = False
        while not rospy.is_shutdown() and not finished:
            dt = 1.0 / rate_hz
            for node in tree.iterate():
                node.dt = dt
            tree.tick_once()
            state = tree.status
            if verbose and time.time() - last_tip >= log_tip_s:
                last_tip = time.time()
                running = [n for n in tree.iterate()
                           if n.status == Status.RUNNING and not n.children]
                tip = running[-1] if running else None
                if tip is not None:
                    rospy.loginfo("运行中 [%s] %s", tip.name, tip.feedback_message)
            if state != Status.RUNNING:
                rospy.loginfo("行为树终态: %s  状态上报: %s",
                              state.name, bridge.status_log)
                finished = True
            else:
                time.sleep(dt)
        if not loop or rospy.is_shutdown():
            break
        time.sleep(1.0)


if __name__ == "__main__":
    main()
