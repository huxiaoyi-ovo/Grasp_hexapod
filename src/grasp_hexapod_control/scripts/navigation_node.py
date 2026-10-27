#!/usr/bin/env python3
"""把机器人/小蓝 NavSatFix 与机器人 Imu 转成 pv_map 导航位姿。

该节点只发布定位结果，不向控制器或舵机发送运动命令。标定文件未明确
``installation_calibrated: true`` 时拒绝启动，避免占位数据进入实机导航。
"""

from pathlib import Path
import sys
from threading import Lock

import numpy as np
import rospy
import yaml
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus


scripts_dir = Path(__file__).resolve().parent
if not (scripts_dir / "utils").is_dir():
    import rospkg

    scripts_dir = Path(
        rospkg.RosPack().get_path("grasp_hexapod_control")
    ) / "scripts"
sys.path.insert(0, str(scripts_dir))

from utils import package_config_path
from utils.navigation import (
    NavigationCalibration,
    RelativePoseEstimator,
    quaternion_from_rotation,
)


class RtkImuNavigationNode:
    """验证异步传感器快照并发布同时间戳的导航位姿。"""

    def __init__(self):
        config_path = Path(rospy.get_param(
            "~config", str(package_config_path("navigation_rtk_imu.yaml"))
        ))
        with config_path.open() as file:
            config = yaml.safe_load(file)
        if not isinstance(config, dict):
            raise ValueError("navigation config must contain a mapping")
        self.calibration = NavigationCalibration.from_mapping(config)
        self.estimator = RelativePoseEstimator(self.calibration)
        self.frame_id = str(config.get("panel_frame", "pv_map")).lstrip("/")
        if self.frame_id != "pv_map":
            raise ValueError("panel_frame must be pv_map")

        self.max_fix_age = float(config.get("max_fix_age_s", 0.5))
        self.max_fix_skew = float(config.get("max_fix_skew_s", 0.2))
        self.max_imu_age = float(config.get("max_imu_age_s", 0.2))
        self.max_sensor_skew = float(config.get("max_sensor_skew_s", 0.2))
        self.max_horizontal_std = float(
            config.get("max_horizontal_std_m", 0.05)
        )
        self.require_known_covariance = bool(
            config.get("require_known_covariance", True)
        )
        self.require_gbas_fix = bool(config.get("require_gbas_fix", True))
        self.require_known_imu_orientation_covariance = bool(
            config.get("require_known_imu_orientation_covariance", True)
        )
        self.max_imu_orientation_std = np.deg2rad(float(
            config.get("max_imu_orientation_std_deg", 5.0)
        ))
        self.publish_rate_hz = float(config.get("publish_rate_hz", 10.0))
        if min(
            self.max_fix_age,
            self.max_fix_skew,
            self.max_imu_age,
            self.max_sensor_skew,
            self.max_horizontal_std,
            self.max_imu_orientation_std,
            self.publish_rate_hz,
        ) <= 0.0:
            raise ValueError("navigation timing, accuracy and rate values must be positive")

        self.lock = Lock()
        self.robot_fix = None
        self.xiaolan_fix = None
        self.imu = None
        self.last_published_stamps = np.zeros(3, dtype=np.float64)
        self.base_pub = rospy.Publisher(
            rospy.get_param(
                "~base_pose_topic", "/grasp_hexapod/navigation/base_pose"
            ),
            PoseStamped,
            queue_size=1,
        )
        self.xiaolan_pub = rospy.Publisher(
            rospy.get_param(
                "~xiaolan_pose_topic", "/grasp_hexapod/navigation/xiaolan_pose"
            ),
            PoseStamped,
            queue_size=1,
        )
        self.boundary_pub = rospy.Publisher(
            rospy.get_param(
                "~pv_boundary_topic", "/grasp_hexapod/navigation/pv_boundary"
            ),
            PolygonStamped,
            queue_size=1,
            latch=True,
        )
        self.subscribers = [
            rospy.Subscriber(
                rospy.get_param("~robot_fix_topic", "/grasp_hexapod/rtk/fix"),
                NavSatFix,
                self._robot_fix_callback,
                queue_size=1,
            ),
            rospy.Subscriber(
                rospy.get_param(
                    "~xiaolan_fix_topic", "/grasp_hexapod/xiaolan/rtk/fix"
                ),
                NavSatFix,
                self._xiaolan_fix_callback,
                queue_size=1,
            ),
            rospy.Subscriber(
                rospy.get_param("~imu_topic", "/grasp_hexapod/imu"),
                Imu,
                self._imu_callback,
                queue_size=1,
            ),
        ]
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate_hz), self._publish
        )
        rospy.loginfo(
            "RTK/IMU navigation ready: frame=%s config=%s",
            self.frame_id,
            config_path,
        )

    def _robot_fix_callback(self, message):
        with self.lock:
            self.robot_fix = message

    def _xiaolan_fix_callback(self, message):
        with self.lock:
            self.xiaolan_fix = message

    def _imu_callback(self, message):
        with self.lock:
            self.imu = message

    def _fix_valid(self, message, now):
        if message is None or message.status.status == NavSatStatus.STATUS_NO_FIX:
            return False, "RTK has no fix"
        if (
            self.require_gbas_fix
            and message.status.status < NavSatStatus.STATUS_GBAS_FIX
        ):
            return False, "RTK is not a GBAS/RTK solution"
        stamp = message.header.stamp.to_sec()
        values = np.array(
            [message.latitude, message.longitude, message.altitude],
            dtype=np.float64,
        )
        if stamp <= 0.0 or not np.isfinite(values).all():
            return False, "RTK timestamp or geodetic position is invalid"
        if not 0.0 <= now - stamp <= self.max_fix_age:
            return False, "RTK is stale or future-dated"
        covariance_known = (
            message.position_covariance_type
            != NavSatFix.COVARIANCE_TYPE_UNKNOWN
        )
        if self.require_known_covariance and not covariance_known:
            return False, "RTK covariance is unknown"
        if covariance_known:
            covariance = np.asarray(
                message.position_covariance, dtype=np.float64
            ).reshape(3, 3)
            horizontal_variance = np.diag(covariance)[:2]
            if (
                not np.isfinite(horizontal_variance).all()
                or (horizontal_variance < 0.0).any()
                or np.sqrt(np.max(horizontal_variance))
                > self.max_horizontal_std
            ):
                return False, "RTK horizontal uncertainty exceeds limit"
        return True, ""

    def _imu_valid(self, message, now):
        if message is None:
            return False, "IMU is missing"
        stamp = message.header.stamp.to_sec()
        quaternion = np.array(
            [
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
                message.orientation.w,
            ],
            dtype=np.float64,
        )
        if (
            stamp <= 0.0
            or not np.isfinite(quaternion).all()
            or np.linalg.norm(quaternion) <= np.finfo(np.float64).eps
        ):
            return False, "IMU timestamp or quaternion is invalid"
        if not 0.0 <= now - stamp <= self.max_imu_age:
            return False, "IMU is stale or future-dated"
        covariance = np.asarray(
            message.orientation_covariance,
            dtype=np.float64,
        ).reshape(3, 3)
        covariance_known = covariance[0, 0] >= 0.0
        if self.require_known_imu_orientation_covariance and not covariance_known:
            return False, "IMU orientation covariance is unknown"
        if covariance_known:
            orientation_variance = np.diag(covariance)
            if (
                not np.isfinite(orientation_variance).all()
                or (orientation_variance < 0.0).any()
                or np.sqrt(np.max(orientation_variance))
                > self.max_imu_orientation_std
            ):
                return False, "IMU orientation uncertainty exceeds limit"
        return True, ""

    @staticmethod
    def _pose_message(transform, stamp, frame_id):
        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.pose.position.x = float(transform[0, 3])
        message.pose.position.y = float(transform[1, 3])
        message.pose.position.z = float(transform[2, 3])
        quaternion = quaternion_from_rotation(transform[:3, :3])
        message.pose.orientation.x = float(quaternion[0])
        message.pose.orientation.y = float(quaternion[1])
        message.pose.orientation.z = float(quaternion[2])
        message.pose.orientation.w = float(quaternion[3])
        return message

    def _boundary_message(self, stamp):
        message = PolygonStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.polygon.points = [
            Point32(x=float(point[0]), y=float(point[1]), z=0.0)
            for point in self.calibration.pv_boundary_xy_m
        ]
        return message

    def _publish(self, _event):
        with self.lock:
            robot_fix = self.robot_fix
            xiaolan_fix = self.xiaolan_fix
            imu = self.imu
        now = rospy.Time.now().to_sec()
        for message, label in (
            (robot_fix, "robot"),
            (xiaolan_fix, "xiaolan"),
        ):
            valid, reason = self._fix_valid(message, now)
            if not valid:
                rospy.logwarn_throttle(2.0, "%s navigation input invalid: %s", label, reason)
                return
        imu_valid, reason = self._imu_valid(imu, now)
        if not imu_valid:
            rospy.logwarn_throttle(2.0, "robot navigation input invalid: %s", reason)
            return
        fix_skew = abs(
            robot_fix.header.stamp.to_sec()
            - xiaolan_fix.header.stamp.to_sec()
        )
        if fix_skew > self.max_fix_skew:
            rospy.logwarn_throttle(
                2.0,
                "RTK pair skew %.3fs exceeds %.3fs",
                fix_skew,
                self.max_fix_skew,
            )
            return
        input_stamps = np.array(
            [
                robot_fix.header.stamp.to_sec(),
                xiaolan_fix.header.stamp.to_sec(),
                imu.header.stamp.to_sec(),
            ],
            dtype=np.float64,
        )
        sensor_skew = float(np.max(input_stamps) - np.min(input_stamps))
        if sensor_skew > self.max_sensor_skew:
            rospy.logwarn_throttle(
                2.0,
                "RTK/IMU snapshot skew %.3fs exceeds %.3fs",
                sensor_skew,
                self.max_sensor_skew,
            )
            return
        if not (input_stamps > self.last_published_stamps).all():
            return
        try:
            pv_from_base, pv_from_xiaolan = self.estimator.estimate(
                [robot_fix.latitude, robot_fix.longitude, robot_fix.altitude],
                [xiaolan_fix.latitude, xiaolan_fix.longitude, xiaolan_fix.altitude],
                [
                    imu.orientation.x,
                    imu.orientation.y,
                    imu.orientation.z,
                    imu.orientation.w,
                ],
            )
        except ValueError as error:
            rospy.logwarn_throttle(2.0, "navigation estimate rejected: %s", error)
            return
        common_stamp_sec = float(np.min(input_stamps))
        common_stamp = rospy.Time.from_sec(common_stamp_sec)
        self.base_pub.publish(
            self._pose_message(pv_from_base, common_stamp, self.frame_id)
        )
        self.xiaolan_pub.publish(
            self._pose_message(pv_from_xiaolan, common_stamp, self.frame_id)
        )
        self.boundary_pub.publish(self._boundary_message(common_stamp))
        self.last_published_stamps = input_stamps


def main():
    rospy.init_node("grasp_hexapod_rtk_imu_navigation")
    RtkImuNavigationNode()
    rospy.spin()


if __name__ == "__main__":
    main()
