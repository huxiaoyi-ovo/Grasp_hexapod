"""控制代码共用的小工具。

输入/输出：NumPy位姿、点集、关节数组和NavigationState。
内容：坐标变换、光伏板边界判断、控制器与外部关节顺序转换。
边界：只做无状态计算和数据整理，不承担步态、ROS或硬件控制。
"""

from dataclasses import dataclass, field

import numpy as np

from kinematics import JOINT_NAMES, LEG_NAMES


@dataclass
class NavigationState:
    """自动接近所需的最小环境状态，全部几何量位于pv_map。"""

    stamp: float = 0.0
    valid: bool = False
    landing_confirmed: bool = False
    pv_from_base: np.ndarray = field(
        default_factory=lambda: np.eye(4, dtype=np.float64)
    )
    pv_from_xiaolan: np.ndarray = field(
        default_factory=lambda: np.eye(4, dtype=np.float64)
    )
    pv_boundary: np.ndarray = field(
        default_factory=lambda: np.empty((0, 2), dtype=np.float64)
    )

    def normalized(self):
        """复制并统一控制周期所需的数组形状和数据类型。"""

        return NavigationState(
            stamp=float(self.stamp),
            valid=bool(self.valid),
            landing_confirmed=bool(self.landing_confirmed),
            pv_from_base=np.asarray(
                self.pv_from_base,
                dtype=np.float64,
            ).reshape(4, 4).copy(),
            pv_from_xiaolan=np.asarray(
                self.pv_from_xiaolan,
                dtype=np.float64,
            ).reshape(4, 4).copy(),
            pv_boundary=np.asarray(
                self.pv_boundary,
                dtype=np.float64,
            ).reshape(-1, 2).copy(),
        )


CONTROL_DOF_NAMES = tuple(
    f"{leg_name}_{joint_name}_joint"
    for leg_name in LEG_NAMES
    for joint_name in JOINT_NAMES
)


def build_dof_indices(external_dof_names):
    """建立控制器顺序到外部18关节顺序的索引映射。"""

    external_index_by_name = {
        name: index
        for index, name in enumerate(external_dof_names)
    }
    missing_names = [
        name
        for name in CONTROL_DOF_NAMES
        if name not in external_index_by_name
    ]
    if missing_names:
        raise ValueError(f"Missing DOF names: {missing_names}")

    return np.array(
        [external_index_by_name[name] for name in CONTROL_DOF_NAMES],
        dtype=np.int64,
    )


def external_to_control(external_values, dof_indices):
    """外部一维18关节数组转换为控制器(6,3)数组。"""

    return np.asarray(external_values)[dof_indices].reshape(6, 3)


def control_to_external(control_values, dof_indices):
    """控制器(6,3)数组转换为外部一维18关节数组。"""

    external_values = np.empty(18, dtype=np.float32)
    external_values[dof_indices] = np.asarray(control_values).reshape(18)
    return external_values


def wrap_angle(angle):
    """把角度压到[-pi, pi)，单位rad。"""

    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def transform_from_xy_yaw(x, y, yaw):
    """生成只包含平面位置和偏航角的4×4齐次变换。"""

    cosine = np.cos(yaw)
    sine = np.sin(yaw)
    transform = np.eye(4, dtype=np.float64)
    transform[:2, :2] = [[cosine, -sine], [sine, cosine]]
    transform[:2, 3] = [x, y]
    return transform


def yaw_from_transform(transform):
    """读取4×4齐次变换中的偏航角，单位rad。"""

    transform = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    return np.arctan2(transform[1, 0], transform[0, 0])


def invert_transform(transform):
    """计算刚体齐次变换的逆。"""

    transform = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = transform[:3, :3].T
    inverse[:3, 3] = -inverse[:3, :3] @ transform[:3, 3]
    return inverse


def transform_points(transform, points):
    """用4×4齐次变换批量转换N个二维或三维点。"""

    transform = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    points = np.asarray(points, dtype=np.float64)
    dimensions = points.shape[-1]
    flat_points = points.reshape(-1, dimensions)

    if dimensions == 2:
        flat_points = np.column_stack(
            (flat_points, np.zeros(len(flat_points)))
        )
    elif dimensions != 3:
        raise ValueError("points must end in dimension 2 or 3")

    homogeneous = np.column_stack(
        (flat_points, np.ones(len(flat_points)))
    )
    transformed = (transform @ homogeneous.T).T[:, :3]
    output_dimensions = 2 if dimensions == 2 else 3
    return transformed[:, :output_dimensions].reshape(
        *points.shape[:-1],
        output_dimensions,
    )


def polygon_area(polygon):
    """计算有序二维多边形的有符号面积。"""

    polygon = np.asarray(polygon, dtype=np.float64).reshape(-1, 2)
    if len(polygon) < 3:
        return 0.0
    next_vertex = np.roll(polygon, -1, axis=0)
    return 0.5 * np.sum(
        polygon[:, 0] * next_vertex[:, 1]
        - polygon[:, 1] * next_vertex[:, 0]
    )


def distance_to_polygon_boundary(points, polygon):
    """计算每个二维点到多边形边界的最短距离。"""

    points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    polygon = np.asarray(polygon, dtype=np.float64).reshape(-1, 2)
    segment_start = polygon
    segment_end = np.roll(polygon, -1, axis=0)
    segment = segment_end - segment_start
    segment_length_sq = np.sum(segment * segment, axis=1)
    valid_segment = segment_length_sq > 1e-15
    segment_start = segment_start[valid_segment]
    segment = segment[valid_segment]
    segment_length_sq = segment_length_sq[valid_segment]
    if len(segment_start) == 0:
        return np.full(len(points), np.inf, dtype=np.float64)

    point_delta = points[:, np.newaxis, :] - segment_start
    projection = np.sum(
        point_delta * segment[np.newaxis, :, :],
        axis=2,
    ) / segment_length_sq
    projection = np.clip(projection, 0.0, 1.0)
    closest = (
        segment_start[np.newaxis, :, :]
        + projection[..., np.newaxis] * segment[np.newaxis, :, :]
    )
    return np.min(
        np.linalg.norm(
            points[:, np.newaxis, :] - closest,
            axis=2,
        ),
        axis=1,
    )


def points_in_polygon(points, polygon):
    """判断二维点是否位于多边形内部；落在边界上的点也视为内部。"""

    points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    polygon = np.asarray(polygon, dtype=np.float64).reshape(-1, 2)
    x = points[:, 0]
    y = points[:, 1]
    inside = np.zeros(len(points), dtype=bool)

    previous = polygon[-1]
    for current in polygon:
        x_previous, y_previous = previous
        x_current, y_current = current
        crosses = (
            (y_current > y) != (y_previous > y)
        ) & (
            x
            < (
                (x_previous - x_current)
                * (y - y_current)
                / (y_previous - y_current + 1e-15)
                + x_current
            )
        )
        inside ^= crosses
        previous = current

    on_boundary = distance_to_polygon_boundary(points, polygon) <= 1e-9
    return inside | on_boundary


def sample_segment(start, end, spacing):
    """按最大spacing对二维线段均匀采样，包含两个端点。"""

    start = np.asarray(start, dtype=np.float64).reshape(2)
    end = np.asarray(end, dtype=np.float64).reshape(2)
    distance = np.linalg.norm(end - start)
    sample_count = max(2, int(np.ceil(distance / spacing)) + 1)
    ratios = np.linspace(0.0, 1.0, sample_count)
    return start + ratios[:, np.newaxis] * (end - start)


def circular_path_feasible(
    start,
    end,
    polygon,
    safety_radius,
    sample_spacing=0.02,
):
    """检查圆形机器人安全包络沿直线路径是否完全位于多边形内。"""

    polygon = np.asarray(polygon, dtype=np.float64).reshape(-1, 2)
    if len(polygon) < 3 or abs(polygon_area(polygon)) < 1e-9:
        return False, 0.0

    samples = sample_segment(start, end, sample_spacing)
    inside = points_in_polygon(samples, polygon)
    clearance = distance_to_polygon_boundary(samples, polygon)
    feasible = bool(inside.all() and np.all(clearance >= safety_radius))
    return feasible, float(clearance.min())
