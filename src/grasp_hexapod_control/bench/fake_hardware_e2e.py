"""仿真两块舵机板：30Hz 发布 /<leg>_pos 回应 <leg>_des 指令，并注入 /joy。"""
import numpy as np
import rospy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64MultiArray

rospy.init_node("fake_hardware")
LEG_NAMES = ("lb", "lf", "lm", "rb", "rf", "rm")
q = np.full((6, 3), 0.02)
des_pubs = {}
pos_pubs = {}
for leg in LEG_NAMES:
    pos_pubs[leg] = rospy.Publisher(f"/{leg}_pos", JointState, queue_size=1)
    des_pubs[leg] = rospy.Subscriber(
        f"/{leg}_des", Float64MultiArray,
        lambda msg, l=leg: q_des_cb(l, msg), queue_size=1)

latest = {}

def q_des_cb(leg, msg):
    latest[leg] = np.array(msg.data[1:4])
    assert len(msg.data) == 10, f"bad msg length {len(msg.data)}"

joy_pub = rospy.Publisher("/joy", Joy, queue_size=1)
rate = rospy.Rate(30)
seq = 0
while not rospy.is_shutdown() and seq < 240:
    for i, leg in enumerate(LEG_NAMES):
        m = JointState()
        m.header.stamp = rospy.Time.now()
        m.position = list(q[i])
        pos_pubs[leg].publish(m)
    if seq == 60:
        # 松开摇杆：B(1) 按下沿 -> 回站
        j = Joy()
        j.axes = [0.0] * 8
        j.buttons = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        joy_pub.publish(j)
    if seq == 65 or seq == 200:
        j = Joy()
        j.axes = [0.0] * 8
        j.buttons = [0] * 11
        joy_pub.publish(j)
    if seq == 100:
        # A(0) 使能
        j = Joy()
        j.axes = [0.0] * 8
        j.buttons = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        joy_pub.publish(j)
    if seq == 105:
        j = Joy()
        j.axes = [0.0] * 8
        j.buttons = [0] * 11
        joy_pub.publish(j)
    if seq >= 110:
        q += 0.0001
    rate.sleep()
    seq += 1
rospy.loginfo("received des legs: %s", sorted(latest.keys()))
assert set(latest.keys()) == set(LEG_NAMES), "not all legs received commands!"
rospy.loginfo("E2E OK: 6 legs commanded, sample lb=%s", np.round(latest["lb"], 4))
