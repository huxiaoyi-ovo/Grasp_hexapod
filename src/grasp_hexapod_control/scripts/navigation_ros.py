"""把导航ROS话题整理为控制器使用的NavigationState。"""

from threading import Lock

import numpy as np
import rospy
from geometry_msgs.msg import PolygonStamped, PoseStamped
from std_msgs.msg import Bool

from utils import NavigationState


PV_FRAME = "pv_map"


def pose_to_transform(pose):
    """把ROS位置和四元数转换为pv_map_from_body齐次变换。"""

    quaternion = np.array(
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
        dtype=np.float64,
    )
    norm = np.linalg.norm(quaternion)
    if norm == 0.0:
        return None
    x, y, z, w = quaternion / norm

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = [
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
    ]
    transform[:3, 3] = [
        pose.position.x,
        pose.position.y,
        pose.position.z,
    ]
    if not np.isfinite(transform).all():
        return None
    return transform


class NavigationRosInput:
    """缓存导航输入；控制循环调用snapshot()读取同一份状态。"""

    def __init__(self):
        self.max_age = float(rospy.get_param("~max_pose_age", 0.5))
        self.lock = Lock()
        self.base_stamp = 0.0
        self.xiaolan_stamp = 0.0
        self.pv_from_base = None
        self.pv_from_xiaolan = None
        self.pv_boundary = np.empty((0, 2), dtype=np.float64)
        self.landing_confirmed = False

        self.subscribers = [
            rospy.Subscriber(
                rospy.get_param(
                    "~base_pose_topic",
                    "/grasp_hexapod/navigation/base_pose",
                ),
                PoseStamped,
                self._base_callback,
                queue_size=1,
            ),
            rospy.Subscriber(
                rospy.get_param(
                    "~xiaolan_pose_topic",
                    "/grasp_hexapod/navigation/xiaolan_pose",
                ),
                PoseStamped,
                self._xiaolan_callback,
                queue_size=1,
            ),
            rospy.Subscriber(
                rospy.get_param(
                    "~pv_boundary_topic",
                    "/grasp_hexapod/navigation/pv_boundary",
                ),
                PolygonStamped,
                self._boundary_callback,
                queue_size=1,
            ),
            rospy.Subscriber(
                rospy.get_param(
                    "~landing_topic",
                    "/grasp_hexapod/landing_confirmed",
                ),
                Bool,
                self._landing_callback,
                queue_size=1,
            ),
        ]

    @staticmethod
    def _valid_frame(message):
        if message.header.frame_id.lstrip("/") == PV_FRAME:
            return True
        rospy.logwarn_throttle(2.0, "Navigation frame must be pv_map")
        return False

    def _base_callback(self, message):
        if not self._valid_frame(message):
            return
        transform = pose_to_transform(message.pose)
        if transform is None:
            return
        with self.lock:
            self.pv_from_base = transform
            self.base_stamp = message.header.stamp.to_sec()

    def _xiaolan_callback(self, message):
        if not self._valid_frame(message):
            return
        transform = pose_to_transform(message.pose)
        if transform is None:
            return
        with self.lock:
            self.pv_from_xiaolan = transform
            self.xiaolan_stamp = message.header.stamp.to_sec()

    def _boundary_callback(self, message):
        if not self._valid_frame(message):
            return
        boundary = np.array(
            [[point.x, point.y] for point in message.polygon.points],
            dtype=np.float64,
        ).reshape(-1, 2)
        if not np.isfinite(boundary).all():
            return
        with self.lock:
            self.pv_boundary = boundary

    def _landing_callback(self, message):
        with self.lock:
            self.landing_confirmed = bool(message.data)

    def snapshot(self):
        """返回当前导航状态；两帧位姿过期时valid=False。"""

        now = rospy.Time.now().to_sec()
        with self.lock:
            stamp = min(self.base_stamp, self.xiaolan_stamp)
            valid = (
                self.pv_from_base is not None
                and self.pv_from_xiaolan is not None
                and len(self.pv_boundary) >= 3
                and stamp > 0.0
                and 0.0 <= now - stamp <= self.max_age
            )
            return NavigationState(
                stamp=stamp,
                valid=valid,
                landing_confirmed=self.landing_confirmed,
                pv_from_base=(
                    np.eye(4)
                    if self.pv_from_base is None
                    else self.pv_from_base.copy()
                ),
                pv_from_xiaolan=(
                    np.eye(4)
                    if self.pv_from_xiaolan is None
                    else self.pv_from_xiaolan.copy()
                ),
                pv_boundary=self.pv_boundary.copy(),
            )
