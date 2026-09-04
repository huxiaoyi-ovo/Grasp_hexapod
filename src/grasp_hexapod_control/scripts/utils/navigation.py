"""RTK/IMU 到光伏板局部坐标系的纯计算导航工具。

输入：WGS84 经纬高、IMU 四元数和实测安装标定。
输出：``pv_map`` 中的 ``base_link``、``xiaolan_frame`` 齐次变换。
边界：不订阅 ROS、不规划足端、不判断接触或承载。
"""

from dataclasses import dataclass

import numpy as np


WGS84_A_M = 6378137.0
WGS84_E2 = 6.69437999014e-3


def _finite_vector(value, size, name):
    vector = np.asarray(value, dtype=np.float64).reshape(-1)
    if vector.shape != (size,) or not np.isfinite(vector).all():
        raise ValueError(f"{name} must contain {size} finite values")
    return vector


def _finite_scalar(value, name):
    try:
        scalar = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be finite") from error
    if not np.isfinite(scalar):
        raise ValueError(f"{name} must be finite")
    return scalar


def rotation_from_rpy(roll, pitch, yaw):
    """返回 ``R_parent_from_child``，角度单位 rad，采用 ZYX 顺序。"""

    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def rotation_from_quaternion(quaternion_xyzw):
    """把 ``[x,y,z,w]`` 四元数转换为旋转矩阵。"""

    quaternion = _finite_vector(quaternion_xyzw, 4, "quaternion_xyzw")
    norm = np.linalg.norm(quaternion)
    if norm <= np.finfo(np.float64).eps:
        raise ValueError("quaternion_xyzw must have non-zero norm")
    x, y, z, w = quaternion / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def quaternion_from_rotation(rotation):
    """把正交旋转矩阵转换为 ``[x,y,z,w]`` 四元数。"""

    rotation = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    if not np.isfinite(rotation).all():
        raise ValueError("rotation must be finite")
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        quaternion = np.array(
            [
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = 2.0 * np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
            quaternion = np.array(
                [
                    0.25 * scale,
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[2, 1] - rotation[1, 2]) / scale,
                ]
            )
        elif index == 1:
            scale = 2.0 * np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
            quaternion = np.array(
                [
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    0.25 * scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    (rotation[0, 2] - rotation[2, 0]) / scale,
                ]
            )
        else:
            scale = 2.0 * np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
            quaternion = np.array(
                [
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    0.25 * scale,
                    (rotation[1, 0] - rotation[0, 1]) / scale,
                ]
            )
    quaternion /= np.linalg.norm(quaternion)
    return quaternion


def geodetic_to_ecef(geodetic_deg_m):
    """WGS84 ``[lat_deg, lon_deg, altitude_m]`` 转 ECEF 米坐标。"""

    latitude_deg, longitude_deg, altitude_m = _finite_vector(
        geodetic_deg_m, 3, "geodetic_deg_m"
    )
    if not -90.0 <= latitude_deg <= 90.0:
        raise ValueError("latitude must be in [-90, 90] degrees")
    if not -180.0 <= longitude_deg <= 180.0:
        raise ValueError("longitude must be in [-180, 180] degrees")
    latitude = np.deg2rad(latitude_deg)
    longitude = np.deg2rad(longitude_deg)
    sin_latitude = np.sin(latitude)
    prime_vertical = WGS84_A_M / np.sqrt(
        1.0 - WGS84_E2 * sin_latitude * sin_latitude
    )
    return np.array(
        [
            (prime_vertical + altitude_m) * np.cos(latitude) * np.cos(longitude),
            (prime_vertical + altitude_m) * np.cos(latitude) * np.sin(longitude),
            (prime_vertical * (1.0 - WGS84_E2) + altitude_m) * sin_latitude,
        ],
        dtype=np.float64,
    )


def ecef_to_enu_rotation(origin_geodetic_deg_m):
    """返回 ``R_enu_from_ecef``。"""

    latitude_deg, longitude_deg, _ = _finite_vector(
        origin_geodetic_deg_m, 3, "origin_geodetic_deg_m"
    )
    latitude = np.deg2rad(latitude_deg)
    longitude = np.deg2rad(longitude_deg)
    sin_latitude, cos_latitude = np.sin(latitude), np.cos(latitude)
    sin_longitude, cos_longitude = np.sin(longitude), np.cos(longitude)
    return np.array(
        [
            [-sin_longitude, cos_longitude, 0.0],
            [-sin_latitude * cos_longitude, -sin_latitude * sin_longitude, cos_latitude],
            [cos_latitude * cos_longitude, cos_latitude * sin_longitude, sin_latitude],
        ],
        dtype=np.float64,
    )


@dataclass(frozen=True)
class NavigationCalibration:
    """一次实测安装对应的固定坐标与杆臂标定。"""

    panel_origin_geodetic_deg_m: np.ndarray
    panel_yaw_from_east_rad: float
    enu_from_imu_reference_rotation: np.ndarray
    imu_from_base_rotation: np.ndarray
    robot_rtk_antenna_in_base_m: np.ndarray
    panel_from_xiaolan_rotation: np.ndarray
    xiaolan_rtk_antenna_in_xiaolan_m: np.ndarray
    pv_boundary_xy_m: np.ndarray

    @classmethod
    def from_mapping(cls, mapping):
        """从 YAML 字典加载并严格拒绝占位或非有限标定。"""

        if mapping.get("installation_calibrated") is not True:
            raise ValueError(
                "navigation calibration must set installation_calibrated: true"
            )
        panel_origin = _finite_vector(
            mapping.get("panel_origin_geodetic_deg_m"),
            3,
            "panel_origin_geodetic_deg_m",
        )
        panel_yaw_deg = _finite_scalar(
            mapping.get("panel_yaw_from_east_deg"),
            "panel_yaw_from_east_deg",
        )
        enu_from_imu_reference_rpy = np.deg2rad(_finite_vector(
            mapping.get("enu_from_imu_reference_rpy_deg"),
            3,
            "enu_from_imu_reference_rpy_deg",
        ))
        imu_from_base_rpy = np.deg2rad(_finite_vector(
            mapping.get("imu_from_base_rpy_deg"), 3, "imu_from_base_rpy_deg"
        ))
        panel_from_xiaolan_rpy = np.deg2rad(_finite_vector(
            mapping.get("panel_from_xiaolan_rpy_deg"),
            3,
            "panel_from_xiaolan_rpy_deg",
        ))
        boundary = np.asarray(
            mapping.get("pv_boundary_xy_m"), dtype=np.float64
        ).reshape(-1, 2)
        if len(boundary) < 3 or not np.isfinite(boundary).all():
            raise ValueError("pv_boundary_xy_m must contain at least three finite points")
        next_point = np.roll(boundary, -1, axis=0)
        area = 0.5 * abs(np.sum(
            boundary[:, 0] * next_point[:, 1]
            - boundary[:, 1] * next_point[:, 0]
        ))
        if area <= 1e-6:
            raise ValueError("pv_boundary_xy_m must enclose a non-zero area")
        return cls(
            panel_origin_geodetic_deg_m=panel_origin,
            panel_yaw_from_east_rad=np.deg2rad(panel_yaw_deg),
            enu_from_imu_reference_rotation=rotation_from_rpy(
                *enu_from_imu_reference_rpy
            ),
            imu_from_base_rotation=rotation_from_rpy(*imu_from_base_rpy),
            robot_rtk_antenna_in_base_m=_finite_vector(
                mapping.get("robot_rtk_antenna_in_base_m"),
                3,
                "robot_rtk_antenna_in_base_m",
            ),
            panel_from_xiaolan_rotation=rotation_from_rpy(
                *panel_from_xiaolan_rpy
            ),
            xiaolan_rtk_antenna_in_xiaolan_m=_finite_vector(
                mapping.get("xiaolan_rtk_antenna_in_xiaolan_m"),
                3,
                "xiaolan_rtk_antenna_in_xiaolan_m",
            ),
            pv_boundary_xy_m=boundary,
        )


class RelativePoseEstimator:
    """用双 RTK 位置和机器人 IMU 姿态生成同一 ``pv_map`` 位姿。"""

    def __init__(self, calibration):
        self.calibration = calibration
        self.origin_ecef = geodetic_to_ecef(
            calibration.panel_origin_geodetic_deg_m
        )
        self.enu_from_ecef = ecef_to_enu_rotation(
            calibration.panel_origin_geodetic_deg_m
        )
        # panel x 从 ENU East 逆时针旋转 panel_yaw 得到。
        self.panel_from_enu = rotation_from_rpy(
            0.0, 0.0, -calibration.panel_yaw_from_east_rad
        )

    def _panel_position(self, geodetic_deg_m):
        delta_ecef = geodetic_to_ecef(geodetic_deg_m) - self.origin_ecef
        return self.panel_from_enu @ self.enu_from_ecef @ delta_ecef

    @staticmethod
    def _transform(rotation, translation):
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = rotation
        transform[:3, 3] = translation
        return transform

    def estimate(self, robot_geodetic, xiaolan_geodetic, imu_quaternion_xyzw):
        """返回 ``(pv_from_base, pv_from_xiaolan)``。"""

        calibration = self.calibration
        imu_reference_from_imu = rotation_from_quaternion(
            imu_quaternion_xyzw
        )
        panel_from_base_rotation = (
            self.panel_from_enu
            @ calibration.enu_from_imu_reference_rotation
            @ imu_reference_from_imu
            @ calibration.imu_from_base_rotation
        )
        robot_antenna_panel = self._panel_position(robot_geodetic)
        base_position_panel = (
            robot_antenna_panel
            - panel_from_base_rotation
            @ calibration.robot_rtk_antenna_in_base_m
        )

        panel_from_xiaolan_rotation = calibration.panel_from_xiaolan_rotation
        xiaolan_antenna_panel = self._panel_position(xiaolan_geodetic)
        xiaolan_position_panel = (
            xiaolan_antenna_panel
            - panel_from_xiaolan_rotation
            @ calibration.xiaolan_rtk_antenna_in_xiaolan_m
        )
        return (
            self._transform(panel_from_base_rotation, base_position_panel),
            self._transform(panel_from_xiaolan_rotation, xiaolan_position_panel),
        )
