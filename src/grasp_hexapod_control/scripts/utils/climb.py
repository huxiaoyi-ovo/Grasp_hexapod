"""攀爬配置、支撑面和离线几何工具。

这些函数只做确定性模型计算；几何支撑诊断不证明真实接触、承载或稳定性。
"""

from dataclasses import dataclass
import numpy as np

from . import (
    distance_to_polygon_boundary,
    points_in_polygon,
    polygon_area,
)


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
