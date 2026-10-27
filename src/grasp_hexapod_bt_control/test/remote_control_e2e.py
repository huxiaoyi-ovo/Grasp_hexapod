"""remote_control_node 端到端验证（伪 switch_mode/gripper_act 服务）。

覆盖链路：
1. /joy 左摇杆 → /cmd_vel（平面三自由度，linear.z 恒为 0）。
2. /joy 失效 → /cmd_vel 归零。
3. 按键沿 → switch_mode：A=walk B=home X=climb Y=dock。
4. 方向键轴 → gripper_act：左=clamp 右=open，回中后可重复触发。

用法（需要 roscore）：
    python3 test/remote_control_e2e.py
脚本自行启动/停止 remote_control_node。
"""
import subprocess
import sys
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from grasp_hexapod_msgs.srv import GripperAct, SwitchMode
from sensor_msgs.msg import Joy

MAX_LINEAR = 0.20
MAX_YAW = 0.50


def main():
    node = subprocess.Popen(
        ["rosrun", "grasp_hexapod_bt_control", "remote_control_node",
         f"_max_linear_speed:={MAX_LINEAR}", f"_max_yaw_rate:={MAX_YAW}"],
        stdout=sys.stdout, stderr=sys.stderr)
    try:
        rospy.init_node("fake_remote_e2e")
        lock = threading.Lock()
        switch_calls = []
        gripper_calls = []
        cmd_messages = []
        gripper_proxy_holder = {}

        def switch_cb(request):
            with lock:
                switch_calls.append(request.target_mode)
            return True, f"fake {request.target_mode} ok"

        def gripper_cb(request):
            with lock:
                gripper_calls.append(request.action)
            return True, f"fake {request.action} ok"

        def cmd_cb(message):
            with lock:
                cmd_messages.append(message)

        rospy.Service("/grasp_hexapod/switch_mode", SwitchMode, switch_cb)
        rospy.Service("/grasp_hexapod/gripper_act", GripperAct, gripper_cb)
        rospy.Subscriber("/cmd_vel", Twist, cmd_cb, queue_size=5)
        joy_pub = rospy.Publisher("/joy", Joy, queue_size=1)
        rate = rospy.Rate(30)

        def joy(axes, buttons):
            message = Joy()
            message.axes = list(axes) + [0.0] * max(0, 8 - len(axes))
            message.buttons = list(buttons) + [0] * max(0, 11 - len(buttons))
            for _ in range(3):  # 发几帧，保证按下沿被锁存
                joy_pub.publish(message)
                rate.sleep()

        # 等节点订阅者就绪。
        deadline = time.time() + 10
        while joy_pub.get_num_connections() == 0 and time.time() < deadline:
            rate.sleep()

        # 1. 摇杆前进 0.5 → linear.x=0.1，y=0，z=0；角速度=0.25。
        joy([0.0, 0.5, 0.0, 0.5], [])
        deadline = time.time() + 2
        sample = None
        while time.time() < deadline:
            with lock:
                if cmd_messages:
                    sample = cmd_messages[-1]
            if sample is not None and abs(sample.linear.x - 0.1) < 1e-6:
                break
            rate.sleep()
        assert sample is not None, "no /cmd_vel received"
        assert abs(sample.linear.x - 0.1) < 1e-6, sample.linear.x
        assert abs(sample.linear.y) < 1e-6, sample.linear.y
        assert abs(sample.linear.z) < 1e-6, "cmd_vel must not carry linear.z"
        assert abs(sample.angular.z - 0.25) < 1e-6, sample.angular.z
        rospy.loginfo("PASS 1 cmd_vel mapping: x=%.3f y=%.3f z=%.3f w=%.3f",
                      sample.linear.x, sample.linear.y, sample.linear.z,
                      sample.angular.z)

        # 2. 停发 /joy → 失效归零。
        deadline = time.time() + 3
        zeroed = False
        while time.time() < deadline:
            with lock:
                if cmd_messages and cmd_messages[-1].linear.x == 0.0 \
                        and cmd_messages[-1].angular.z == 0.0:
                    zeroed = True
                    break
            rate.sleep()
        assert zeroed, "/cmd_vel did not zero after joy loss"
        rospy.loginfo("PASS 2 joy loss zeroing")

        # 3. 按键沿 → switch_mode。
        joy([], [1])  # A=walk
        joy([], [0, 1])  # B=home
        joy([], [0, 0, 1])  # X=climb
        joy([], [0, 0, 0, 1])  # Y=dock
        deadline = time.time() + 3
        while time.time() < deadline:
            with lock:
                if len(switch_calls) >= 4:
                    break
            rate.sleep()
        with lock:
            recorded = list(switch_calls)
        assert recorded == ["walk", "home", "climb", "dock"], recorded
        rospy.loginfo("PASS 3 button->switch_mode: %s", recorded)

        # 4. 方向键（axes[6]）→ 夹爪：右=+1 open，左=-1 clamp，回中后可重复。
        joy([0.0] * 6 + [1.0], [])   # dpad 右
        time.sleep(0.5)
        joy([0.0] * 6 + [-1.0], [])  # dpad 左
        time.sleep(0.5)
        joy([0.0] * 8, [])           # 回中
        time.sleep(0.3)
        joy([0.0] * 6 + [1.0], [])   # 再次右 → 可重复触发
        deadline = time.time() + 3
        while time.time() < deadline:
            with lock:
                if gripper_calls == ["open", "clamp", "open"]:
                    break
            rate.sleep()
        with lock:
            gripper_recorded = list(gripper_calls)
        assert gripper_recorded == ["open", "clamp", "open"], gripper_recorded
        rospy.loginfo("PASS 4 dpad->gripper: %s", gripper_recorded)

        rospy.loginfo("REMOTE E2E OK: all 4 scenarios passed")
    finally:
        node.terminate()
        node.wait(timeout=10)


if __name__ == "__main__":
    main()
