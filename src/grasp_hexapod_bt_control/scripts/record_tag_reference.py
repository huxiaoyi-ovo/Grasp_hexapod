#!/usr/bin/env python3
"""Teach-in helper: average N fresh AprilTag detections and print yaml lines.

Preferred (hardware 2026-11-27: the camera still sees the tag at ~0.15 m when
the front legs touch Xiaolan, so the tag is visible at P0): place the robot at
the climb start pose P0 in the walking stance that will be used, then run
  rosrun grasp_hexapod_bt_control record_tag_reference.py _tag_id:=<ID> _samples:=30
(_offset_from_p0_m defaults to 0). Fallback when the tag cannot be seen at P0:
walk straight back from P0 to where it is clearly visible, measure the back-off
distance D with a tape and pass _offset_from_p0_m:=<D>; then tag_p0_z_m =
mean taught depth - D (virtual depth at P0; an error in D transfers 1:1 to the
stopping position).
Prints tag_ref_x_m / tag_p0_z_m / tag_ref_yaw_rad for config/phase1_visual.yaml
and the sample standard deviations.

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
    # 0 = taught at P0 itself (hardware 2026-11-27: the tag stays visible down to
    # ~0.15 m, i.e. at P0). > 0 only when teaching from a measured back-off D.
    offset = rospy.get_param("~offset_from_p0_m", 0.0)
    if isinstance(offset, bool) or not isinstance(offset, (int, float)) \
            or not math.isfinite(offset) or offset < 0.0:
        rospy.logerr("~offset_from_p0_m must be >= 0 (0 = robot at P0; else measured back-off D in metres)")
        sys.exit(1)
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
    # Sample standard deviation (n - 1); yaw spread is measured around the circular mean.
    def std(values):
        return math.sqrt(sum(v * v for v in values) / (len(values) - 1)) if len(values) > 1 else 0.0

    sx = std([s[0] - mx for s in samples])
    sz = std([s[1] - mz for s in samples])
    syaw = std([math.atan2(math.sin(s[2] - myaw), math.cos(s[2] - myaw)) for s in samples])
    print("# %d samples of tag id %d at back-off D=%.4f m from P0 (camera optical frame)" % (len(samples), tag_id, offset))
    print("# sample std: x=%.4f m  z=%.4f m  yaw=%.4f rad  (mean taught depth z=%.4f m)" % (sx, sz, syaw, mz))
    print("tag_ref_x_m: %.4f" % mx)
    print("tag_p0_z_m: %.4f" % (mz - offset))
    print("tag_ref_yaw_rad: %.4f" % myaw)


if __name__ == "__main__":
    main()
