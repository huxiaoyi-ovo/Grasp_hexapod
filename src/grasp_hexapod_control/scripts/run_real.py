#!/usr/bin/env python3
"""抓取六足实机ROS入口与外部感知接口。

功能：
    订阅协作者发布的六足位姿、小蓝位姿、光伏板边界和落板确认信息，
    统一转换为NavigationState。后续在同一60 Hz循环中接入舵机反馈、
    GraspController和LX-15D目标下发。
输入：
    PoseStamped、PolygonStamped和Bool；所有导航几何必须位于pv_map。
输出：
    当前仅维护ROS无关的NavigationState；舵机闭环尚未接通。
边界：
    本文件只做ROS/硬件适配，不选择接近侧、不规划步态、不计算运动学。
"""

from pathlib import Path
import sys

import numpy as np
import rospy
from geometry_msgs.msg import PolygonStamped, PoseStamped
from std_msgs.msg import Bool

# catkin生成的启动转发脚本不自动加入本包scripts目录。
sys.path.insert(0, str(Path(__file__).resolve().parent))

from utils import NavigationState, polygon_area


PV_FRAME = "pv_map"


def pose_to_transform(pose):
    """把geometry_msgs/Pose转换为4×4齐次变换。"""

    quaternion = np.array(
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
        dtype=np.float64,
    )
    quaternion_norm = np.linalg.norm(quaternion)
    if quaternion_norm < 1e-12:
        return None
    quaternion /= quaternion_norm
    x, y, z, w = quaternion

    rotation = np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = [
        pose.position.x,
        pose.position.y,
        pose.position.z,
    ]
    return transform


class NavigationRosAdapter:
    """把协作者ROS话题汇总成控制器使用的NavigationState。"""

    def __init__(self):
        self.max_pose_age = float(
            rospy.get_param("~max_pose_age", 0.5)
        )
        self.base_stamp = 0.0
        self.xiaolan_stamp = 0.0
        self.base_valid = False
        self.xiaolan_valid = False
        self.boundary_valid = False
        self.landing_confirmed = False
        self.pv_from_base = np.eye(4, dtype=np.float64)
        self.pv_from_xiaolan = np.eye(4, dtype=np.float64)
        self.pv_boundary = np.empty((0, 2), dtype=np.float64)

        rospy.Subscriber(
            "~base_pose",
            PoseStamped,
            self._base_pose_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "~xiaolan_pose",
            PoseStamped,
            self._xiaolan_pose_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "~pv_boundary",
            PolygonStamped,
            self._boundary_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "~landing_confirmed",
            Bool,
            self._landing_callback,
            queue_size=1,
        )

    @staticmethod
    def _is_pv_frame(frame_id):
        return frame_id.lstrip("/") == PV_FRAME

    def _base_pose_callback(self, message):
        if not self._is_pv_frame(message.header.frame_id):
            self.base_valid = False
            rospy.logwarn_throttle(
                2.0,
                "base_pose must use frame_id=%s",
                PV_FRAME,
            )
            return
        transform = pose_to_transform(message.pose)
        if transform is None:
            self.base_valid = False
            return
        self.pv_from_base = transform
        self.base_stamp = message.header.stamp.to_sec()
        self.base_valid = True

    def _xiaolan_pose_callback(self, message):
        if not self._is_pv_frame(message.header.frame_id):
            self.xiaolan_valid = False
            rospy.logwarn_throttle(
                2.0,
                "xiaolan_pose must use frame_id=%s",
                PV_FRAME,
            )
            return
        transform = pose_to_transform(message.pose)
        if transform is None:
            self.xiaolan_valid = False
            return
        self.pv_from_xiaolan = transform
        self.xiaolan_stamp = message.header.stamp.to_sec()
        self.xiaolan_valid = True

    def _boundary_callback(self, message):
        if not self._is_pv_frame(message.header.frame_id):
            self.boundary_valid = False
            rospy.logwarn_throttle(
                2.0,
                "pv_boundary must use frame_id=%s",
                PV_FRAME,
            )
            return
        self.pv_boundary = np.array(
            [[point.x, point.y] for point in message.polygon.points],
            dtype=np.float64,
        ).reshape(-1, 2)
        self.boundary_valid = (
            len(self.pv_boundary) >= 3
            and abs(polygon_area(self.pv_boundary)) >= 1e-9
        )

    def _landing_callback(self, message):
        self.landing_confirmed = bool(message.data)

    def snapshot(self):
        """返回当前导航快照；位姿过期或字段缺失时valid=False。"""

        now = rospy.Time.now().to_sec()
        pose_stamp = min(self.base_stamp, self.xiaolan_stamp)
        pose_age = now - pose_stamp
        pose_fresh = (
            pose_stamp > 0.0
            and 0.0 <= pose_age <= self.max_pose_age
        )
        valid = (
            self.base_valid
            and self.xiaolan_valid
            and self.boundary_valid
            and pose_fresh
        )
        return NavigationState(
            stamp=pose_stamp,
            valid=valid,
            landing_confirmed=self.landing_confirmed,
            pv_from_base=self.pv_from_base,
            pv_from_xiaolan=self.pv_from_xiaolan,
            pv_boundary=self.pv_boundary,
        )


def main():
    rospy.init_node("grasp_hexapod_control")
    adapter = NavigationRosAdapter()
    rate = rospy.Rate(60.0)

    rospy.loginfo(
        "Navigation interface ready; controller-to-servo loop is not connected"
    )
    while not rospy.is_shutdown():
        navigation = adapter.snapshot()
        if not navigation.valid:
            rospy.logwarn_throttle(
                2.0,
                "Waiting for fresh pv_map navigation inputs",
            )

        # TODO: 读取18个舵机位置 -> controller.update(...)
        #       -> 关节/舵机方向与raw映射 -> 同步下发LX-15D。
        rate.sleep()


if __name__ == "__main__":
    main()
