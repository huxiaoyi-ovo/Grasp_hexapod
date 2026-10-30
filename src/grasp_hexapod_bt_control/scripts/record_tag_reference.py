#!/usr/bin/env python3
"""Teach-in helper: average N fresh AprilTag detections and print yaml lines.

Place the robot at the desired final climb-start pose in front of the tag, then
run:  rosrun grasp_hexapod_bt_control record_tag_reference.py _tag_id:=0
Prints tag_ref_x_m / tag_ref_z_m / tag_ref_yaw_rad for config/phase1_visual.yaml.

Frame: camera optical (x right, y down, z forward).  yaw uses the same
definition as the controller: yaw = atan2(R(2,0), R(0,0)) with R the rotation
of the tag frame in the camera frame (tag x axis = image +x, independent of
the tag z convention).  Each yaw is converted before averaging; the mean is circular.
"""
import math
import sys

import rospy
from apriltag_ros.msg import AprilTagDetectionArray


MAX_X_AXIS_OUT_OF_PLANE = 0.5  # same limit as kMaxTagXAxisOutOfPlane in tag_guidance.h


def tag_yaw(qx, qy, qz, qw):
    """Return yaw, or None for an unusable quaternion / bogus pose."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(n) or n < 1e-6:
        return None
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    r00 = 1.0 - 2.0 * (y * y + z * z)
    r10 = 2.0 * (x * y + z * w)
    r20 = 2.0 * (x * z - y * w)
    if not all(map(math.isfinite, (r00, r10, r20))) or abs(r10) > MAX_X_AXIS_OUT_OF_PLANE:
        return None  # bogus / heavily rolled detection
    return math.atan2(r20, r00)


def main():
    rospy.init_node("record_tag_reference")
    tag_id = rospy.get_param("~tag_id", 0)
    count = rospy.get_param("~samples", 30)
    topic = rospy.get_param("~topic", "/xiaolan/tag_detections")
    max_age = rospy.get_param("~max_age_s", 0.25)
    frame = rospy.get_param("~frame", "camera_color_optical_frame")
    timeout = rospy.get_param("~timeout_s", 30.0)
    samples = []
    last_stamp = [0.0]

    def callback(msg):
        for d in msg.detections:
            if len(d.id) != 1 or d.id[0] != tag_id:
                continue
            header = d.pose.header if d.pose.header.stamp.to_sec() > 0.0 else msg.header
            stamp = header.stamp.to_sec()
            if header.frame_id != frame or stamp <= last_stamp[0]:
                return
            if rospy.Time.now().to_sec() - stamp > max_age:
                return
            p = d.pose.pose.pose.position
            q = d.pose.pose.pose.orientation
            yaw = tag_yaw(q.x, q.y, q.z, q.w)
            if yaw is None or not (p.z > 0.0) or not all(map(math.isfinite, (p.x, p.z))):
                return
            last_stamp[0] = stamp
            samples.append((p.x, p.z, yaw))
            return

    rospy.Subscriber(topic, AprilTagDetectionArray, callback, queue_size=1)
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown() and len(samples) < count:
        if rospy.Time.now() > deadline:
            rospy.logerr("only %d/%d fresh detections of id %d on %s", len(samples), count, tag_id, topic)
            sys.exit(1)
        rate.sleep()
    if len(samples) < count:
        sys.exit(1)

    n = float(len(samples))
    mx = sum(s[0] for s in samples) / n
    mz = sum(s[1] for s in samples) / n
    myaw = math.atan2(sum(math.sin(s[2]) for s in samples) / n,
                      sum(math.cos(s[2]) for s in samples) / n)
    sx = math.sqrt(sum((s[0] - mx) ** 2 for s in samples) / n)
    sz = math.sqrt(sum((s[1] - mz) ** 2 for s in samples) / n)
    print("# %d samples of tag id %d; std x=%.4f m z=%.4f m (camera optical frame)" % (len(samples), tag_id, sx, sz))
    print("tag_ref_x_m: %.4f" % mx)
    print("tag_ref_z_m: %.4f" % mz)
    print("tag_ref_yaw_rad: %.4f" % myaw)


if __name__ == "__main__":
    main()
