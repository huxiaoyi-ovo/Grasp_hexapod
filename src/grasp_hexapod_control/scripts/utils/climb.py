"""攀爬配置、支撑面和离线几何工具。

这些函数只做确定性模型计算；几何支撑诊断不证明真实接触、承载或稳定性。
"""

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

from . import (
    distance_to_polygon_boundary,
    points_in_polygon,
    polygon_area,
)


CLIMB_CONFIG_SCHEMA_VERSION = 1


def _null_config_paths(value, prefix=""):
    """返回字典中每个 JSON null 的点分路径。"""

    paths = []
    if isinstance(value, dict):
        for key, child in value.items():
            child_path = f"{prefix}.{key}" if prefix else key
            paths.extend(_null_config_paths(child, child_path))
    elif value is None:
        paths.append(prefix)
    return paths


def _config_path_value(config, dotted_path):
    """读取已有点分字典路径，不补充默认值。"""

    value = config
    for part in dotted_path.split("."):
        if not isinstance(value, dict) or part not in value:
            raise ValueError(
                f"unresolved climb config path does not exist: {dotted_path}"
            )
        value = value[part]
    return value


def validate_climb_config(config):
    """校验攀爬配置。

    参数:
        config: 从 JSON 读取的攀爬配置。

    返回:
        校验后的原配置。
    """

    if not isinstance(config, dict):
        raise ValueError("climb config must be a JSON object at the top level")

    schema_version = config.get("schema_version")
    if (
        not isinstance(schema_version, int)
        or schema_version != CLIMB_CONFIG_SCHEMA_VERSION
    ):
        raise ValueError(
            f"unsupported climb config schema_version: {schema_version!r}"
        )

    status = config.get("status")
    if status not in ("UNRESOLVED", "READY"):
        raise ValueError(f"invalid climb config status: {status!r}")

    units = config.get("units")
    if (
        not isinstance(units, dict)
        or units.get("length") != "m"
        or units.get("angle") != "rad"
    ):
        raise ValueError(
            "climb config units must be exactly length=m and angle=rad"
        )

    unresolved = config.get("unresolved")
    if not isinstance(unresolved, list) or not all(
        isinstance(path, str) and bool(path) for path in unresolved
    ):
        raise ValueError(
            "climb config unresolved must be a list of dotted path strings"
        )

    if len(set(unresolved)) != len(unresolved):
        raise ValueError("climb config unresolved paths must be unique")
    for path in unresolved:
        if _config_path_value(config, path) is not None:
            raise ValueError(
                f"unresolved climb config path must contain null: {path}"
            )

    null_paths = set(_null_config_paths(config))
    unresolved_paths = set(unresolved)
    if null_paths != unresolved_paths:
        missing = sorted(null_paths - unresolved_paths)
        extra = sorted(unresolved_paths - null_paths)
        raise ValueError(
            "climb config unresolved/null mismatch: "
            f"missing={missing}, extra={extra}"
        )

    if status == "READY" and unresolved:
        raise ValueError(
            "climb config status READY with a nonempty unresolved list"
        )
    if status == "UNRESOLVED" and not unresolved:
        raise ValueError(
            "climb config status UNRESOLVED requires unresolved values"
        )

    return config


def load_climb_config(path):
    """读取并校验攀爬配置文件。

    参数:
        path: 配置文件路径。

    返回:
        配置字典。

    不补充默认值。
    """

    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as config_file:
        config = json.load(config_file)
    return validate_climb_config(config)


def climb_config_ready(config):
    """检查攀爬配置是否可用。

    参数:
        config: 已读取的攀爬配置。

    返回:
        配置有效且状态为 READY 时返回 True。
    """

    try:
        validate_climb_config(config)
    except ValueError:
        return False
    return config.get("status") == "READY" and not config.get("unresolved")


@dataclass(frozen=True)
class SurfaceFrame:
    """表示 C 系中的攀爬支撑面。

    `basis_climb` 的列依次为 u、v、法线；`safe_polygon_uv` 的单位为 m。
    """

    name: str
    point_climb: np.ndarray
    basis_climb: np.ndarray
    safe_polygon_uv: np.ndarray


def build_surface_frame(
    name,
    point_climb,
    normal_climb,
    tangent_hint_climb,
    safe_polygon_uv,
):
    """创建支撑面坐标系。

    参数:
        name: 支撑面名称。
        point_climb: 面上的一点，单位 m。
        normal_climb: 支撑面外法线。
        tangent_hint_climb: 用于确定 u 轴方向的参考向量。
        safe_polygon_uv: uv 平面的安全多边形，单位 m。

    返回:
        校验后的 SurfaceFrame。
    """

    if not isinstance(name, str) or not name.strip():
        raise ValueError("surface name must be a nonempty string")

    point = np.asarray(point_climb, dtype=np.float64).reshape(3).copy()
    normal = np.asarray(normal_climb, dtype=np.float64).reshape(3).copy()
    tangent_hint = np.asarray(
        tangent_hint_climb,
        dtype=np.float64,
    ).reshape(3).copy()
    polygon = np.asarray(
        safe_polygon_uv,
        dtype=np.float64,
    ).reshape(-1, 2).copy()

    if not np.all(np.isfinite(point)):
        raise ValueError("surface point must be finite")
    if not np.all(np.isfinite(normal)) or np.linalg.norm(normal) <= 0.0:
        raise ValueError("surface normal must be finite and nonzero")
    if not np.all(np.isfinite(tangent_hint)):
        raise ValueError("surface tangent hint must be finite")
    if not np.all(np.isfinite(polygon)):
        raise ValueError("surface safe polygon must be finite")

    normal = normal / np.linalg.norm(normal)
    tangent_projected = tangent_hint - normal * np.dot(normal, tangent_hint)
    tangent_norm = np.linalg.norm(tangent_projected)
    if tangent_norm <= 0.0:
        raise ValueError(
            "surface tangent hint must not be parallel to the normal"
        )
    u = tangent_projected / tangent_norm
    v = np.cross(normal, u)
    v = v / np.linalg.norm(v)
    basis = np.column_stack((u, v, normal))

    if len(np.unique(polygon, axis=0)) < 3:
        raise ValueError("surface safe polygon needs at least 3 unique vertices")
    if abs(polygon_area(polygon)) <= 1e-12:
        raise ValueError("surface safe polygon must have nonzero area")

    point.setflags(write=False)
    basis.setflags(write=False)
    polygon.setflags(write=False)
    return SurfaceFrame(
        name=name,
        point_climb=point,
        basis_climb=basis,
        safe_polygon_uv=polygon,
    )


def project_points_to_surface(surface, points_climb):
    """把 C 系点投影到支撑面。

    参数:
        surface: 支撑面坐标系。
        points_climb: 末维为 3 的 C 系坐标，单位 m。

    返回:
        uv 坐标和沿外法线的有符号高度，单位 m。
    """

    points = np.asarray(points_climb, dtype=np.float64)
    if points.shape[-1] != 3:
        raise ValueError("points_climb must end in dimension 3")
    if not np.all(np.isfinite(points)):
        raise ValueError("points_climb must be finite")

    flat = points.reshape(-1, 3)
    local = (flat - surface.point_climb) @ surface.basis_climb
    uv = local[:, :2].reshape(*points.shape[:-1], 2)
    signed_height = local[:, 2].reshape(points.shape[:-1])
    return uv, signed_height


def surface_points_from_uv(surface, uv, signed_height=0.0):
    """把支撑面 uv 坐标换回 C 系坐标。

    参数:
        surface: 支撑面坐标系。
        uv: 末维为 2 的平面坐标，单位 m。
        signed_height: 沿外法线的高度，单位 m。

    返回:
        对应的 C 系三维坐标。
    """

    uv_array = np.asarray(uv, dtype=np.float64)
    if uv_array.shape[-1] != 2:
        raise ValueError("uv must end in dimension 2")
    if not np.all(np.isfinite(uv_array)):
        raise ValueError("uv must be finite")

    heights = np.asarray(signed_height, dtype=np.float64)
    if not np.all(np.isfinite(heights)):
        raise ValueError("signed_height must be finite")

    points = (
        surface.point_climb
        + uv_array @ surface.basis_climb[:, :2].T
        + heights[..., np.newaxis] * surface.basis_climb[:, 2]
    )
    return points


def terminal_axis_surface_angles(terminal_axes_climb, surface_normals_climb):
    """计算终端轴与支撑面法线的夹角。

    参数:
        terminal_axes_climb: 单个 `(3,)` 或多个 `(N, 3)` 终端轴。
        surface_normals_climb: 单个 `(3,)` 或多个 `(N, 3)` 支撑面法线。

    返回:
        终端轴反向对齐法线时为 0 的角度，单位 rad。
    """

    axes = np.asarray(terminal_axes_climb, dtype=np.float64)
    if axes.shape[-1] != 3:
        raise ValueError("terminal_axes_climb must end in dimension 3")
    axes_flat = axes.reshape(-1, 3)
    axis_norms = np.linalg.norm(axes_flat, axis=1)
    if not np.all(np.isfinite(axes_flat)) or np.any(axis_norms <= 0.0):
        raise ValueError(
            "terminal_axes_climb must be finite and nonzero"
        )
    unit_axes = axes_flat / axis_norms[:, np.newaxis]

    normals = np.asarray(surface_normals_climb, dtype=np.float64)
    if normals.shape[-1] != 3:
        raise ValueError("surface_normals_climb must end in dimension 3")
    if normals.ndim == 1:
        normal_norm = np.linalg.norm(normals)
        if not np.all(np.isfinite(normals)) or normal_norm <= 0.0:
            raise ValueError(
                "surface_normals_climb must be finite and nonzero"
            )
        unit_normals = np.broadcast_to(
            normals / normal_norm,
            (len(axes_flat), 3),
        )
    else:
        normals_flat = normals.reshape(-1, 3)
        if len(normals_flat) not in (1, len(axes_flat)):
            raise ValueError(
                "surface_normals_climb count must match terminal axes count"
            )
        normal_norms = np.linalg.norm(normals_flat, axis=1)
        if not np.all(np.isfinite(normals_flat)) or np.any(
            normal_norms <= 0.0
        ):
            raise ValueError(
                "surface_normals_climb must be finite and nonzero"
            )
        unit_normals = normals_flat / normal_norms[:, np.newaxis]

    dot = np.sum(-unit_axes * unit_normals, axis=1)
    return np.arccos(np.clip(dot, -1.0, 1.0)).reshape(axes.shape[:-1])


def _cross2d(a, b, c):
    """二维叉积 (b-a) x (c-a)。"""

    return (
        (b[0] - a[0]) * (c[1] - a[1])
        - (b[1] - a[1]) * (c[0] - a[0])
    )


def convex_hull_2d(points):
    """计算二维点集的逆时针凸包。

    参数:
        points: 形状为 `(N, 2)` 的二维点。

    返回:
        去重后的凸包顶点；退化输入直接返回去重结果。
    """

    raw_points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if not np.all(np.isfinite(raw_points)):
        raise ValueError("points must be finite")
    unique = np.unique(raw_points, axis=0)
    if len(unique) < 3:
        return unique.copy()

    lower = []
    for point in unique:
        while len(lower) >= 2 and _cross2d(
            lower[-2],
            lower[-1],
            point,
        ) <= 0.0:
            lower.pop()
        lower.append(point)

    upper = []
    for point in reversed(unique):
        while len(upper) >= 2 and _cross2d(
            upper[-2],
            upper[-1],
            point,
        ) <= 0.0:
            upper.pop()
        upper.append(point)

    hull = np.asarray(lower[:-1] + upper[:-1], dtype=np.float64)
    return hull.copy()


def signed_polygon_margin(point, polygon):
    """计算点到多边形边界的有符号距离。

    参数:
        point: 二维点，单位 m。
        polygon: 多边形顶点，单位 m。

    返回:
        点在多边形内时为非负，外部时为负的距离，单位 m。
    """

    point_array = np.asarray(point, dtype=np.float64).reshape(2)
    polygon_array = np.asarray(polygon, dtype=np.float64).reshape(-1, 2)
    if not np.all(np.isfinite(point_array)):
        raise ValueError("point must be finite")
    if not np.all(np.isfinite(polygon_array)):
        raise ValueError("polygon must be finite")
    if len(np.unique(polygon_array, axis=0)) < 3:
        raise ValueError(
            "polygon needs at least 3 unique vertices for a signed margin"
        )
    if abs(polygon_area(polygon_array)) <= 1e-12:
        raise ValueError("polygon must have nonzero area")

    distance = float(
        distance_to_polygon_boundary(
            point_array[np.newaxis, :],
            polygon_array,
        )[0]
    )
    inside = bool(
        points_in_polygon(
            point_array[np.newaxis, :],
            polygon_array,
        )[0]
    )
    return distance if inside else -distance


@dataclass(frozen=True)
class SupportProjection:
    """保存支撑投影的几何结果。

    它不表示真实接触、承载或摩擦可行性。
    """

    valid: bool
    com_uv: np.ndarray
    hull_uv: np.ndarray
    raw_margin_m: float
    uncertainty_radius_m: float
    robust_margin_m: float
    robust_inside: bool
    reason: str

    @classmethod
    def build(
        cls,
        valid,
        com_uv,
        hull_uv,
        raw_margin_m,
        uncertainty_radius_m,
        robust_margin_m,
        robust_inside,
        reason,
    ):
        """创建不可变的支撑投影结果。

        参数:
            valid: 投影多边形是否有效。
            com_uv: 质心的 uv 坐标。
            hull_uv: 支撑凸包顶点。
            raw_margin_m: 原始几何余量，单位 m。
            uncertainty_radius_m: 扣除的不确定度，单位 m。
            robust_margin_m: 扣除不确定度后的余量，单位 m。
            robust_inside: 扣除不确定度后是否仍在凸包内。
            reason: 结果说明。

        返回:
            SupportProjection 实例。
        """

        com_uv_array = np.asarray(com_uv, dtype=np.float64).copy()
        hull_uv_array = np.asarray(hull_uv, dtype=np.float64).copy()
        com_uv_array.setflags(write=False)
        hull_uv_array.setflags(write=False)
        return cls(
            valid=bool(valid),
            com_uv=com_uv_array,
            hull_uv=hull_uv_array,
            raw_margin_m=float(raw_margin_m),
            uncertainty_radius_m=float(uncertainty_radius_m),
            robust_margin_m=float(robust_margin_m),
            robust_inside=bool(robust_inside),
            reason=str(reason),
        )


def gravity_projected_support(
    com_climb,
    contacts_climb,
    gravity_climb,
    uncertainty_radius_m=0.0,
    tangent_hint_climb=(1, 0, 0),
):
    """计算重力投影在支撑多边形内的几何余量。

    参数:
        com_climb: C 系质心坐标，单位 m。
        contacts_climb: C 系支撑点，形状为 `(N, 3)`，单位 m。
        gravity_climb: C 系重力方向。
        uncertainty_radius_m: 要扣除的不确定度，单位 m。
        tangent_hint_climb: 用于确定投影平面的参考方向。

    返回:
        支撑多边形和几何余量。

    这只是几何诊断，不证明真实接触、摩擦或承载。
    """

    com = np.asarray(com_climb, dtype=np.float64).reshape(3)
    contacts = np.asarray(contacts_climb, dtype=np.float64).reshape(-1, 3)
    gravity = np.asarray(gravity_climb, dtype=np.float64).reshape(3)
    tangent_hint = np.asarray(
        tangent_hint_climb,
        dtype=np.float64,
    ).reshape(3)
    uncertainty = float(uncertainty_radius_m)

    if not np.all(np.isfinite(com)):
        raise ValueError("com_climb must be finite")
    if not np.all(np.isfinite(contacts)):
        raise ValueError("contacts_climb must be finite")
    if not np.all(np.isfinite(gravity)) or np.linalg.norm(gravity) <= 0.0:
        raise ValueError("gravity_climb must be finite and nonzero")
    if not np.all(np.isfinite(tangent_hint)):
        raise ValueError("tangent_hint_climb must be finite")
    if not np.isfinite(uncertainty) or uncertainty < 0.0:
        raise ValueError("uncertainty_radius_m must be finite and nonnegative")

    hint_norm = np.linalg.norm(tangent_hint)
    if hint_norm <= 0.0:
        raise ValueError("tangent_hint_climb must be nonzero")

    normal = -gravity / np.linalg.norm(gravity)
    if abs(np.dot(normal, tangent_hint) / hint_norm) > 1.0 - 1e-12:
        tangent_hint = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        if abs(np.dot(normal, tangent_hint)) > 1.0 - 1e-12:
            tangent_hint = np.array([0.0, 0.0, 1.0], dtype=np.float64)

    tangent_projected = tangent_hint - normal * np.dot(
        normal,
        tangent_hint,
    )
    tangent_projected_norm = np.linalg.norm(tangent_projected)
    if tangent_projected_norm <= 0.0:
        raise ValueError("failed to find a nonparallel tangent basis")
    u = tangent_projected / tangent_projected_norm
    v = np.cross(normal, u)
    v = v / np.linalg.norm(v)
    basis = np.column_stack((u, v, normal))

    contacts_uv = (contacts - com) @ basis[:, :2]
    com_uv = np.zeros(2, dtype=np.float64)
    hull_uv = convex_hull_2d(contacts_uv)

    if len(hull_uv) < 3 or abs(polygon_area(hull_uv)) <= 1e-12:
        return SupportProjection.build(
            valid=False,
            com_uv=com_uv,
            hull_uv=hull_uv,
            raw_margin_m=-np.inf,
            uncertainty_radius_m=uncertainty,
            robust_margin_m=-np.inf,
            robust_inside=False,
            reason="fewer than 3 unique/noncollinear projected contacts",
        )

    raw_margin = signed_polygon_margin(com_uv, hull_uv)
    robust_margin = raw_margin - uncertainty
    return SupportProjection.build(
        valid=True,
        com_uv=com_uv,
        hull_uv=hull_uv,
        raw_margin_m=raw_margin,
        uncertainty_radius_m=uncertainty,
        robust_margin_m=robust_margin,
        robust_inside=robust_margin >= 0.0,
        reason=(
            "geometry-only support projection; necessary diagnostic, "
            "not sufficient for friction/wrench feasibility or contact/load"
        ),
    )


# M2/M3 离线分析元数据与就绪门。
#
# 这些辅助函数仅提供分析元数据，不编码轨迹、目标坐标、时间、阈值、接触
# 声明或 P0 重置顺序，也绝不批准运动。

CLIMB_LEG_NAMES = ("lb", "lf", "lm", "rb", "rf", "rm")

# 离线几何输入存在性门所需的物理输入。
# 当前 M3 与 M2 要求相同，因为尚未定义额外的实测 C4/C5 输入。
# 此处刻意不要求 IMU/RTK/LoRa/事件阈值字段。
M2_REQUIRED_PHYSICAL_PATHS = (
    "unknown.task_frame_origin_climb_m",
    "unknown.task_frame_y_positive_reference",
    "unknown.xiaolan_orientation_climb",
    "unknown.deck_height_tolerance_m",
    "unknown.deck_height_reference",
    "unknown.deck_edge_survey_climb",
    "unknown.deck_safe_polygon_uv_climb",
    "unknown.deck_normal_survey_climb",
    "unknown.body_collision_envelope_m",
    "unknown.bottom_camera_collision_envelope_m",
    "unknown.motor_collision_envelope_m",
    "unknown.measured_foot_geometry",
    "unknown.real_mass_kg",
    "unknown.real_com_uncertainty_m",
    "unknown.friction_range",
    "unknown.lx15d_loaded_tracking_error_rad",
    "unknown.lx15d_backlash_rad",
    "unknown.lx15d_loaded_speed_rad_s",
    "unknown.lx15d_board_skew_rad",
)
M3_REQUIRED_PHYSICAL_PATHS = M2_REQUIRED_PHYSICAL_PATHS


@dataclass(frozen=True)
class ClimbSequenceSpec:
    """保存一个离线攀爬顺序候选。"""

    name: str
    p0_strategy: str
    initial_platform_groups: tuple

    def __post_init__(self):
        """检查腿组配置并冻结为元组。"""

        if not isinstance(self.name, str) or not self.name:
            raise ValueError("climb sequence name must be a nonempty string")
        if not isinstance(self.p0_strategy, str) or not self.p0_strategy:
            raise ValueError(
                "climb sequence p0_strategy must be a nonempty string"
            )
        groups = tuple(tuple(group) for group in self.initial_platform_groups)
        if not groups:
            raise ValueError(
                "climb sequence initial_platform_groups must not be empty"
            )
        seen_legs = []
        for group in groups:
            if not group:
                raise ValueError(
                    "climb sequence platform group must not be empty"
                )
            for leg in group:
                if leg not in CLIMB_LEG_NAMES:
                    raise ValueError(
                        f"invalid climb sequence leg name: {leg!r}"
                    )
                if leg in seen_legs:
                    raise ValueError(
                        f"duplicate climb sequence leg name: {leg!r}"
                    )
                seen_legs.append(leg)
        object.__setattr__(self, "initial_platform_groups", groups)


def climb_sequence_specs():
    """返回固定顺序的 M2 分析候选。

    返回:
        ClimbSequenceSpec 元组。
    """

    specs = (
        ClimbSequenceSpec(
            "pair_first_rm_retract",
            "rm_retract",
            (("rb", "rf"), ("rm",)),
        ),
        ClimbSequenceSpec(
            "pair_first_rb_rf_extend",
            "rb_rf_extend",
            (("rb", "rf"), ("rm",)),
        ),
        ClimbSequenceSpec(
            "pair_first_combined",
            "combined",
            (("rb", "rf"), ("rm",)),
        ),
        ClimbSequenceSpec(
            "rm_first_then_pair",
            "uncommitted_rm_first_sequence",
            (("rm",), ("rb", "rf")),
        ),
    )
    names = [spec.name for spec in specs]
    if len(set(names)) != len(names):
        raise ValueError("climb sequence strategy names must be unique")
    return specs


@dataclass(frozen=True)
class OfflineInputPresence:
    """保存 M2/M3 离线输入是否齐全的结果。"""

    milestone: str
    status: str
    missing_paths: tuple
    note: str

    def __post_init__(self):
        """把缺失路径固定为元组。"""

        object.__setattr__(
            self,
            "missing_paths",
            tuple(self.missing_paths),
        )


def offline_input_presence(config, milestone):
    """检查 M2/M3 所需的离线输入是否存在。

    参数:
        config: 攀爬配置。
        milestone: 阶段名，支持 `M2` 或 `M3`。

    返回:
        按固定顺序给出每项必需输入是否存在。

    只检查是否存在，不检查内容是否正确。
    """

    validate_climb_config(config)
    if milestone not in ("M2", "M3"):
        raise ValueError(
            f"unknown offline analysis milestone: {milestone!r}"
        )
    required_paths = (
        M2_REQUIRED_PHYSICAL_PATHS
        if milestone == "M2"
        else M3_REQUIRED_PHYSICAL_PATHS
    )
    missing_paths = tuple(
        path
        for path in required_paths
        if _config_path_value(config, path) is None
    )
    note = (
        "required physical inputs unresolved; input presence is not "
        "schema/value validation, feasibility, certification, deployment "
        "approval, or contact/load proof"
        if missing_paths
        else (
            "required physical inputs present; input presence is not "
            "schema/value validation, feasibility, certification, "
            "deployment approval, or contact/load proof"
        )
    )
    return OfflineInputPresence(
        milestone=milestone,
        status="UNRESOLVED" if missing_paths else "INPUTS_PRESENT",
        missing_paths=missing_paths,
        note=note,
    )


def resolve_compact_stage_range(config, start=None, end=None):
    """解析 compact 预览的闭区间阶段选择器。

    ``C1`` 到 ``Cn`` 是用户可见的数组序号别名；运行时阶段名仍直接
    接受。这个函数不改变配置，因而可由 Isaac 入口和离线预览检查共用。
    """

    stages = config.get("stages") if isinstance(config, dict) else None
    if not isinstance(stages, list) or not stages:
        raise ValueError("compact stage list is invalid")
    names = tuple(stage.get("name") for stage in stages)
    if any(not isinstance(name, str) for name in names):
        raise ValueError("compact stage names are invalid")

    def resolve(selector, default, label):
        if selector is None:
            return default
        if not isinstance(selector, str) or not selector.strip():
            raise ValueError(
                f"--climb-{label} must be a Cn alias or runtime stage name"
            )
        value = selector.strip()
        if value.startswith("C") and value[1:].isdigit():
            index = int(value[1:]) - 1
            if 0 <= index < len(stages):
                return index
            raise ValueError(f"unknown compact stage selector: {selector!r}")
        if value in names:
            return names.index(value)
        raise ValueError(f"unknown compact stage selector: {selector!r}")

    start_index = resolve(start, 0, "from")
    end_index = resolve(end, len(stages) - 1, "to")
    if start_index > end_index:
        raise ValueError("--climb-from must not be after --climb-to")
    return start_index, end_index
