#!/usr/bin/env python3
"""Publish docking mechanical transforms from dock_system.yaml."""

from pathlib import Path
import sys

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_matrix

scripts_dir = Path(__file__).resolve().parent
if not (scripts_dir / "dock_mode.py").exists():
    import rospkg

    scripts_dir = (
        Path(rospkg.RosPack().get_path("grasp_hexapod_control"))
        / "scripts"
    )
sys.path.insert(0, str(scripts_dir))

from dock_mode import invert_transform, load_dock_system


def _message(parent, child, pose, stamp):
    message = TransformStamped()
    message.header.stamp = stamp
    message.header.frame_id = parent
    message.child_frame_id = child
    message.transform.translation.x = float(pose[0, 3])
    message.transform.translation.y = float(pose[1, 3])
    message.transform.translation.z = float(pose[2, 3])
    quaternion = quaternion_from_matrix(pose)
    message.transform.rotation.x = float(quaternion[0])
    message.transform.rotation.y = float(quaternion[1])
    message.transform.rotation.z = float(quaternion[2])
    message.transform.rotation.w = float(quaternion[3])
    return message


def main():
    rospy.init_node("dock_static_tf")
    config_path = str(rospy.get_param("~dock_system_config", "")).strip()
    lock_frame = str(rospy.get_param("~lock_frame", "dock_lock_center"))
    camera_frame = str(rospy.get_param(
        "~camera_frame", "dock_camera_optical_frame"
    ))
    pin_frame_prefix = str(rospy.get_param(
        "~pin_frame_prefix", "dock_pin_from_tag_"
    ))
    dock_system = load_dock_system(config_path or None)
    stamp = rospy.Time.now()
    transforms = [
        _message(
            lock_frame,
            camera_frame,
            dock_system["lock_from_camera"],
            stamp,
        )
    ]
    for tag_id in dock_system["tag_ids"]:
        transforms.append(_message(
            dock_system["tag_frames"][tag_id],
            "{}{}".format(pin_frame_prefix, tag_id),
            invert_transform(dock_system["pin_from_tag"][tag_id]),
            stamp,
        ))
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transforms)
    rospy.loginfo(
        "Published %d docking static TFs from %s",
        len(transforms), dock_system["path"],
    )
    rospy.spin()


if __name__ == "__main__":
    main()
