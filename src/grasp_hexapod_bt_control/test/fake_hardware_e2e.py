"""bt_control_node 端到端验证（伪舵机板 + 伪夹爪服务）。

覆盖链路：
1. WAIT_B 安全门：首次 home 之前请求 walk 被拒绝（"call home first"）；
   release 免门控，WAIT_B 下也能直接开夹爪。
2. switch_mode(home)：先松夹爪再回正到 HOLD（夹爪结果即响应）；发布
   /grasp_hexapod/mode_status（home/success）。
3. switch_mode(walk) + /cmd_vel：步态目标持续变化；/cmd_vel 停发后安全停步，
   walk 以 "cmd_vel lost" 结束。
4. 抢占：walk 进行中调用 switch_mode(home)，walk 立即以 "preempted by home"
   结束，home 正常完成。
5. HOLD 租约：walk 进行中发布 /grasp_hexapod/hold_motion 心跳，目标原地冻结
   （仍持续发布保持上力），心跳停止后从冻结点恢复推进（PauseGate 语义）。
6. approach 假模式（~approach_fake 默认 true）：约 5 秒后返回成功。
7. switch_mode(release)：内部调用夹爪 open（与 dock 成功后的 clamp 走同一
   actuateGripper 服务链路）。

注意：switch_mode 是"提交请求 → 等控制循环终态"的阻塞服务，控制循环只在新
反馈帧上推进，因此伪舵机板必须全程持续发布 /<leg>_pos。

用法（需要 roscore，不需要真实舵机/夹爪）：
    python3 test/fake_hardware_e2e.py
脚本自行启动/停止 bt_control_node（rosrun，全部默认参数即可）。
"""
import subprocess
import sys
import threading
import time
from contextlib import contextmanager

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from grasp_hexapod_msgs.msg import ModeStatus
from grasp_hexapod_msgs.srv import GripperAct, SwitchMode
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

LEG_NAMES = ("lb", "lf", "lm", "rb", "rf", "rm")
POSE_RATE_HZ = 30.0


class FakeHardware:
    """双板伺服仿真 + 伪夹爪服务；反馈线程全程持续发布。"""

    def __init__(self):
        rospy.init_node("fake_hardware_bt_e2e")
        self.lock = threading.Lock()
        self.q_des = np.full((6, 3), 0.02)
        self.legs_seen = set()
        self.gripper_calls = []
        self.walk_targets = []
        self.record_targets = False
        self.pos_pubs = {
            leg: rospy.Publisher(f"/{leg}_pos", JointState, queue_size=1)
            for leg in LEG_NAMES
        }
        for leg in LEG_NAMES:
            rospy.Subscriber(
                f"/{leg}_des", Float64MultiArray,
                lambda msg, l=leg: self.des_cb(l, msg), queue_size=1)
        rospy.Service("/grasp_hexapod/gripper_act", GripperAct,
                      self.gripper_cb)
        self.pose_thread = threading.Thread(target=self.pose_loop, daemon=True)
        self.pose_thread.start()
        self.mode_statuses = []
        rospy.Subscriber("/grasp_hexapod/mode_status", ModeStatus,
                         self.mode_status_cb, queue_size=5)

    def mode_status_cb(self, msg):
        with self.lock:
            self.mode_statuses.append((msg.mode, msg.status, msg.run))

    def des_cb(self, leg, msg):
        assert len(msg.data) == 10, f"bad msg length {len(msg.data)}"
        self.legs_seen.add(leg)
        with self.lock:
            self.q_des[LEG_NAMES.index(leg)] = msg.data[1:4]
            if self.record_targets:
                self.walk_targets.append(self.q_des.copy())

    def gripper_cb(self, request):
        with self.lock:
            self.gripper_calls.append(request.action)
        rospy.sleep(0.1)  # 模拟夹爪到位验证耗时
        return True, f"fake {request.action} ok"

    def pose_loop(self):
        rate = rospy.Rate(POSE_RATE_HZ)
        while not rospy.is_shutdown():
            with self.lock:
                snapshot = self.q_des.copy()
            stamp = rospy.Time.now()
            for index, leg in enumerate(LEG_NAMES):
                message = JointState()
                message.header.stamp = stamp
                message.position = list(snapshot[index])
                self.pos_pubs[leg].publish(message)
            rate.sleep()

    def wait_until(self, predicate, timeout_s, description):
        deadline = time.time() + timeout_s
        while time.time() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            rospy.sleep(0.05)
        raise AssertionError(f"timeout waiting for: {description}")


class CmdVelPublisher:
    """后台 30Hz 发布前进速度；stop 后停发（触发 cmd_vel lost）。"""

    def __init__(self, vx_forward=0.1):
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.vx = vx_forward
        self.running = True
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        rate = rospy.Rate(30)
        while self.running and not rospy.is_shutdown():
            message = Twist()
            message.linear.x = self.vx
            self.pub.publish(message)
            rate.sleep()

    def stop(self):
        self.running = False
        self.thread.join(timeout=5)


def call_mode(proxy, mode, timeout_s=60.0):
    """线程内阻塞调用 switch_mode，返回 (success, message)；超时返回 (None,"")。"""
    result = {}

    def worker():
        try:
            response = proxy(mode)
            result["success"] = response.success
            result["message"] = response.message
        except Exception as exc:  # noqa: BLE001
            result["success"] = None
            result["message"] = f"exception: {exc}"

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    thread.join(timeout_s)
    if thread.is_alive():
        return None, "timeout"
    return result.get("success"), result.get("message", "")


@contextmanager
def record_targets(fake):
    with fake.lock:
        fake.walk_targets = []
        fake.record_targets = True
    try:
        yield
    finally:
        with fake.lock:
            fake.record_targets = False


def main():
    node = subprocess.Popen(
        ["rosrun", "grasp_hexapod_bt_control", "bt_control_node"],
        stdout=sys.stdout, stderr=sys.stderr)
    try:
        fake = FakeHardware()
        switch = rospy.ServiceProxy("/grasp_hexapod/switch_mode", SwitchMode)
        switch.wait_for_service(timeout=15)
        rospy.loginfo("switch_mode service up")

        # 1. WAIT_B 安全门：home 之前 walk 必须被拒；release 免门控可直接开夹爪。
        ok, message = call_mode(switch, "walk", timeout_s=15)
        assert ok is False, f"WAIT_B walk should be rejected, got {ok}"
        assert "call home first" in message, message
        ok, message = call_mode(switch, "release", timeout_s=15)
        assert ok is True, f"WAIT_B release should succeed: {message}"
        assert fake.gripper_calls == ["open"], fake.gripper_calls
        rospy.loginfo("PASS 1 WAIT_B gate: walk rejected; release opened gripper")

        # 2. home：先松夹爪（免门控 release 已开过一次，这是第二次）再回正；
        #    此后节点开始发布 /<leg>_des；mode_status 出现 home 终态。
        ok, message = call_mode(switch, "home", timeout_s=60)
        assert ok is True, f"home failed: {message}"
        assert fake.gripper_calls.count("open") >= 2, fake.gripper_calls
        fake.wait_until(lambda: len(fake.legs_seen) == len(LEG_NAMES), 15,
                        "targets echoed to all 6 /<leg>_des topics")
        fake.wait_until(
            lambda: any(m == "home" and s == "success"
                        for m, s, _ in fake.mode_statuses),
            5, "mode_status home success")
        rospy.loginfo("PASS 2 home: %s (gripper=%s)", message,
                      fake.gripper_calls)

        # 3. walk + /cmd_vel：目标变化；停发 /cmd_vel 后安全停步结束。
        walk_result = {}

        def walk_worker():
            walk_result["success"], walk_result["message"] = call_mode(
                switch, "walk", timeout_s=60)

        cmd = CmdVelPublisher(vx_forward=0.1)
        with record_targets(fake):
            walk_thread = threading.Thread(target=walk_worker, daemon=True)
            walk_thread.start()
            time.sleep(1.5)  # 步态行进窗口
            cmd.stop()       # 停发速度 → walk 以 cmd_vel lost 结束
            walk_thread.join(timeout=30)
            assert not walk_thread.is_alive(), "walk did not finish"
        assert walk_result["success"] is False, walk_result
        assert "cmd_vel lost" in walk_result["message"], walk_result
        assert set(LEG_NAMES) <= fake.legs_seen, fake.legs_seen
        targets = np.array(fake.walk_targets)
        span = (np.ptp(targets.reshape(targets.shape[0], -1), axis=0).max()
                if targets.size else 0.0)
        assert span > 1e-3, f"walk targets did not move (span={span})"
        rospy.loginfo("PASS 3 walk+cmd_vel: %s; target span=%.4f rad",
                      walk_result["message"], span)

        # 4. 抢占：walk 中请求 home，walk 立即结束，home 完成。
        preempt_result = {}

        def preempt_walk_worker():
            preempt_result["success"], preempt_result["message"] = call_mode(
                switch, "walk", timeout_s=60)

        cmd = CmdVelPublisher(vx_forward=0.1)
        walk_thread = threading.Thread(target=preempt_walk_worker, daemon=True)
        walk_thread.start()
        time.sleep(0.8)
        ok, message = call_mode(switch, "home", timeout_s=60)
        cmd.stop()
        walk_thread.join(timeout=30)
        assert ok is True, f"preempting home failed: {message}"
        assert preempt_result["success"] is False, preempt_result
        assert "preempted by home" in preempt_result["message"], preempt_result
        rospy.loginfo("PASS 4 preempt: walk -> %s", preempt_result["message"])

        # 5. HOLD 租约：心跳期间目标原地冻结（仍发布），停止后恢复推进。
        hold_result = {}

        def hold_walk_worker():
            hold_result["success"], hold_result["message"] = call_mode(
                switch, "walk", timeout_s=60)

        cmd = CmdVelPublisher(vx_forward=0.1)
        with record_targets(fake):
            walk_thread = threading.Thread(target=hold_walk_worker, daemon=True)
            walk_thread.start()
            time.sleep(1.0)  # 正常行进

            hold_pub = rospy.Publisher("/grasp_hexapod/hold_motion", String,
                                       queue_size=5)
            beat_deadline = time.time() + 2.5

            def beat():
                rate = rospy.Rate(30)
                while time.time() < beat_deadline and not rospy.is_shutdown():
                    hold_pub.publish(String(data="paused"))
                    rate.sleep()

            beat_thread = threading.Thread(target=beat, daemon=True)
            beat_thread.start()
            time.sleep(1.0)  # 等租约生效
            with fake.lock:
                frozen_start = len(fake.walk_targets)
            time.sleep(1.0)  # 冻结窗口（心跳期内）
            with fake.lock:
                frozen = np.array(fake.walk_targets[frozen_start:])
            frozen_span = (float(np.ptp(frozen.reshape(frozen.shape[0], -1),
                                        axis=0).max())
                           if frozen.size else 1.0)
            assert len(frozen) > 10, \
                "no targets published during hold (servo must stay powered)"
            assert frozen_span == 0.0, \
                f"targets changed during hold lease: {frozen_span}"
            beat_thread.join(timeout=5)
            time.sleep(0.6)  # 租约过期 → resume
            with fake.lock:
                resumed_start = len(fake.walk_targets)
            time.sleep(1.0)
            with fake.lock:
                resumed = np.array(fake.walk_targets[resumed_start:])
            resumed_span = (float(np.ptp(resumed.reshape(resumed.shape[0], -1),
                                         axis=0).max())
                            if resumed.size else 0.0)
            assert resumed_span > 1e-4, \
                f"walk did not resume after hold lease expired (span={resumed_span})"
            cmd.stop()  # 结束 walk：cmd_vel lost
            walk_thread.join(timeout=30)
            assert not walk_thread.is_alive(), "walk did not finish"
        assert hold_result["success"] is False, hold_result
        assert "cmd_vel lost" in hold_result["message"], hold_result
        rospy.loginfo(
            "PASS 5 hold lease: frozen span=0.0 (%d frames), resumed "
            "span=%.4f rad", len(frozen), resumed_span)

        # 6. approach 假模式（~approach_fake 默认 true）：约 5s 后返回成功。
        t0 = time.time()
        ok, message = call_mode(switch, "approach", timeout_s=30)
        elapsed = time.time() - t0
        assert ok is True, f"fake approach failed: {message}"
        assert "fake approach completed" in message, message
        assert 4.0 <= elapsed <= 10.0, f"fake approach duration={elapsed:.1f}s"
        rospy.loginfo("PASS 6 fake approach: %s (%.1fs)", message, elapsed)

        # 7. release：内部调用夹爪 open（dock clamp 同链路）。
        ok, message = call_mode(switch, "release", timeout_s=30)
        assert ok is True, f"release failed: {message}"
        assert fake.gripper_calls.count("open") >= 3, fake.gripper_calls
        rospy.loginfo("PASS 7 release: %s (gripper=%s)", message,
                      fake.gripper_calls)

        rospy.loginfo("E2E OK: all 7 scenarios passed")
    finally:
        node.terminate()
        node.wait(timeout=10)


if __name__ == "__main__":
    main()
