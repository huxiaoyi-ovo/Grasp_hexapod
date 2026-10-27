"""M2/M3 offline-analysis readiness gate and deterministic model-baseline report.

This CLI validates the climb config, derives a deterministic readiness/model
baseline from GraspKinematic and Q_STAND, and reports the M2/M3
OfflineInputPresence results. It does not search joint configurations, solve
IK, emit a plan, or authorize any real-robot motion.

Run from the repository root as:
    python3 src/grasp_hexapod_control/scripts/tools/analyze_climb.py
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
from collections import defaultdict
from dataclasses import dataclass

import numpy as np


SCRIPTS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS_DIR))

from utils import climb
import kinematics as k
from utils import points_in_polygon


REPORT_SCHEMA = "climb_offline_readiness_baseline"
REPORT_SCHEMA_VERSION = 1
HORIZONTAL_OUTWARD_NORMAL = np.array([0.0, 0.0, 1.0], dtype=np.float64)
GRAVITY_DOWN = np.array([0.0, 0.0, -9.81], dtype=np.float64)
XIAOLAN_CAD_CONFIG_KEY = "xiaolan_cad_low_contact"
_XIAOLAN_DEFAULT_MESH_PATH = (
    SCRIPTS_DIR.parents[1]
    / "grasp_hexapod_description"
    / "meshes"
    / "xiaolan"
    / "base_link_xiaolan.STL"
)
_VERTEX_KEY_PRECISION_M = 1e-7
_ANCHOR_MATCH_TOL_M = 1e-6
_PROJECTED_AREA_EPS = 1e-15

LIMITATIONS = (
    "no trajectory search was performed by this milestone",
    "geometry-only support is not a contact/load/friction/stability proof",
    "support margins use the estimated Q_STAND COM with zero applied "
    "uncertainty; real COM uncertainty remains unresolved",
    "this report grants no real-robot authorization",
)


def _json_compatible(value):
    """把 NumPy 数据转成可写入 JSON 的 Python 数据。

    参数:
        value: 待转换的数据。

    返回:
        可写入 JSON 的数据。
    """

    if isinstance(value, dict):
        return {
            str(key): _json_compatible(item)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [_json_compatible(item) for item in value]
    if isinstance(value, np.ndarray):
        return _json_compatible(value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    return value


def _assert_finite(value):
    """检查数据中是否有 NaN 或无穷值。

    参数:
        value: 已转为普通 Python 类型的数据。

    返回:
        无。
    """

    if isinstance(value, dict):
        for item in value.values():
            _assert_finite(item)
    elif isinstance(value, list):
        for item in value:
            _assert_finite(item)
    elif isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(
                "climb offline baseline contains a non-finite value"
            )


def _finalize_report(report):
    """整理报告数据，确保能安全写入 JSON。

    参数:
        report: 原始报告数据。

    返回:
        可写入 JSON 的报告。
    """

    compatible = _json_compatible(report)
    _assert_finite(compatible)
    json.dumps(compatible, allow_nan=False, sort_keys=True)
    return compatible


def _vertex_key(point):
    """把 STL 顶点坐标转成稳定的整数键。

    参数:
        point: 顶点坐标。

    返回:
        取整后的坐标键。
    """

    rounded = np.round(
        np.asarray(point, dtype=np.float64) / _VERTEX_KEY_PRECISION_M
    )
    return tuple(int(value) for value in rounded)


def _vertex_coord(key):
    """把顶点整数键还原为坐标。

    参数:
        key: 顶点整数键。

    返回:
        规范化后的顶点坐标。
    """

    return np.asarray(key, dtype=np.float64) * _VERTEX_KEY_PRECISION_M


def read_binary_stl(path):
    """读取二进制 STL 网格。

    参数:
        path: STL 文件路径。

    返回:
        法向量 `(N, 3)` 和顶点 `(N, 3, 3)`。
    """

    data = Path(path).read_bytes()
    if len(data) < 84:
        raise ValueError(
            "binary STL is too short to contain an 80-byte header and "
            f"triangle count: {len(data)} bytes"
        )
    header = data[:80]
    if len(header) != 80:
        raise ValueError("binary STL header must be exactly 80 bytes")

    triangle_count = struct.unpack_from("<I", data, 80)[0]
    expected_size = 84 + 50 * triangle_count
    if len(data) != expected_size:
        raise ValueError(
            "binary STL size mismatch: header declares "
            f"{triangle_count} triangles ({expected_size} bytes expected), "
            f"but file contains {len(data)} bytes"
        )
    if triangle_count == 0:
        raise ValueError("binary STL declares zero triangles")

    normals = np.empty((triangle_count, 3), dtype=np.float64)
    vertices = np.empty((triangle_count, 3, 3), dtype=np.float64)
    for index in range(triangle_count):
        record_offset = 84 + 50 * index
        normal = np.asarray(
            struct.unpack_from("<3f", data, record_offset),
            dtype=np.float64,
        )
        triangle = np.asarray(
            struct.unpack_from("<9f", data, record_offset + 12),
            dtype=np.float64,
        ).reshape(3, 3)
        if (
            not np.all(np.isfinite(normal))
            or not np.all(np.isfinite(triangle))
        ):
            raise ValueError(
                f"binary STL triangle {index} contains non-finite data"
            )
        if np.linalg.norm(normal) <= 0.0:
            raise ValueError(
                f"binary STL triangle {index} has a zero-length normal"
            )
        normals[index] = normal
        vertices[index] = triangle
    return normals, vertices


def _sha256_hex(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _xiaolan_cad_object(config):
    try:
        cad = config["known"][XIAOLAN_CAD_CONFIG_KEY]
    except (KeyError, TypeError) as exc:
        raise ValueError(
            "climb config is missing known.xiaolan_cad_low_contact"
        ) from exc
    if not isinstance(cad, dict):
        raise ValueError("known.xiaolan_cad_low_contact must be a JSON object")
    required = {
        "status",
        "stl_sha256",
        "internal_features_policy",
        "upper_surfaces_policy",
        "upper_surfaces_height_range_m",
        "negative_x",
        "positive_x",
        "selector_tolerance_semantics",
    }
    missing = sorted(required - set(cad))
    if missing:
        raise ValueError(
            "known.xiaolan_cad_low_contact is missing fields: "
            + ", ".join(missing)
        )
    return cad


def _validate_selector(selector):
    if not isinstance(selector, dict):
        raise ValueError("xiaolan CAD side selector must be a JSON object")
    required = {
        "normal",
        "plane_offset_m",
        "normal_tolerance",
        "plane_offset_tolerance_m",
        "transition_anchors_xy_m",
    }
    missing = sorted(required - set(selector))
    if missing:
        raise ValueError(
            "xiaolan CAD side selector is missing fields: "
            + ", ".join(missing)
        )


@dataclass
class _XiaolanSideExtraction:
    """保存单侧小蓝低斜面的提取结果。"""

    selected_triangle_count: int
    selected_triangle_area_m2: float
    selected_bounds_m: np.ndarray
    plane_normal: np.ndarray
    projection_normal: np.ndarray
    plane_offset_m: float
    outer_loop_vertices_3d_m: np.ndarray
    outer_loop_vertices_2d_m: np.ndarray
    outer_loop_projected_area_m2: float
    internal_loops_3d_m: list
    internal_loops_2d_m: list
    internal_loop_areas_m2: list
    step_high_segments_3d_m: list
    step_high_segments_2d_m: list
    outer_drop_segments_3d_m: list
    outer_drop_segments_2d_m: list
    step_high_segment_count: int
    outer_drop_segment_count: int


def extract_xiaolan_side(vertices, normals, selector):
    """从网格中提取一侧小蓝低斜面。

    参数:
        vertices: STL 顶点，shape 为 `(N, 3, 3)`。
        normals: STL 法向量，shape 为 `(N, 3)`。
        selector: 斜面筛选条件。

    返回:
        该侧斜面的边界和边缘信息。
    """

    _validate_selector(selector)
    vertices = np.asarray(vertices, dtype=np.float64)
    normals = np.asarray(normals, dtype=np.float64)
    if vertices.ndim != 3 or vertices.shape[1:] != (3, 3):
        raise ValueError("vertices must have shape (N, 3, 3)")
    if normals.shape != (len(vertices), 3):
        raise ValueError("normals must have shape (N, 3)")
    if not np.all(np.isfinite(vertices)) or not np.all(np.isfinite(normals)):
        raise ValueError("mesh vertices and normals must be finite")

    normal_lengths = np.linalg.norm(normals, axis=1)
    if np.any(normal_lengths <= 0.0):
        raise ValueError("mesh contains a zero-length normal")
    unit_normals = normals / normal_lengths[:, np.newaxis]

    selector_normal = np.asarray(selector["normal"], dtype=np.float64)
    if selector_normal.shape != (3,) or not np.all(np.isfinite(selector_normal)):
        raise ValueError("selector normal must be a finite length-3 vector")
    selector_normal_raw = selector_normal.copy()
    selector_normal_norm = np.linalg.norm(selector_normal)
    if selector_normal_norm <= 0.0:
        raise ValueError("selector normal must be nonzero")
    selector_normal = selector_normal / selector_normal_norm
    selector_offset = float(selector["plane_offset_m"])
    normal_tolerance = float(selector["normal_tolerance"])
    offset_tolerance = float(selector["plane_offset_tolerance_m"])
    if (
        normal_tolerance <= 0.0
        or offset_tolerance <= 0.0
        or not math.isfinite(normal_tolerance)
        or not math.isfinite(offset_tolerance)
    ):
        raise ValueError("selector tolerances must be finite and positive")

    centroids = vertices.mean(axis=1)
    normal_distances = np.linalg.norm(
        unit_normals - selector_normal,
        axis=1,
    )
    offset_distances = np.abs(centroids @ selector_normal - selector_offset)
    selected = np.flatnonzero(
        (normal_distances <= normal_tolerance)
        & (offset_distances <= offset_tolerance)
    )
    if len(selected) == 0:
        raise ValueError("selector matched no triangles on the mesh")

    selected_vertices = vertices[selected]
    selected_unit_normals = unit_normals[selected]
    cross_products = np.cross(
        selected_vertices[:, 1] - selected_vertices[:, 0],
        selected_vertices[:, 2] - selected_vertices[:, 0],
    )
    selected_triangle_area = float(
        0.5 * np.sum(np.linalg.norm(cross_products, axis=1))
    )
    if selected_triangle_area <= 0.0:
        raise ValueError("selected CAD patch has zero triangle area")
    selected_bounds = np.array(
        [
            selected_vertices.min(axis=(0, 1)),
            selected_vertices.max(axis=(0, 1)),
        ],
        dtype=np.float64,
    )

    plane_normal = selected_unit_normals.mean(axis=0)
    plane_normal = plane_normal / np.linalg.norm(plane_normal)
    plane_offset = float(
        np.mean(centroids[selected] @ plane_normal)
    )

    vertex_id = {}
    vertex_coords = []
    triangle_ids = []
    edge_triangles = defaultdict(list)
    for triangle_index, triangle in enumerate(selected_vertices):
        ids = []
        for point in triangle:
            key = _vertex_key(point)
            if key not in vertex_id:
                vertex_id[key] = len(vertex_id)
                vertex_coords.append(_vertex_coord(key))
            ids.append(vertex_id[key])
        if len(set(ids)) != 3:
            raise ValueError(
                "selected CAD triangle collapses to fewer than three "
                "canonical vertices"
            )
        triangle_ids.append(ids)
        for first, second in (
            (ids[0], ids[1]),
            (ids[1], ids[2]),
            (ids[2], ids[0]),
        ):
            if first == second:
                raise ValueError("selected CAD triangle has a zero-length edge")
            if first > second:
                first, second = second, first
            edge_triangles[(first, second)].append(triangle_index)

    for edge, triangle_list in edge_triangles.items():
        if len(triangle_list) not in (1, 2):
            raise ValueError(
                f"non-manifold relevant edge {edge} is shared by "
                f"{len(triangle_list)} triangles"
            )

    triangle_adjacency = [set() for _ in range(len(selected_vertices))]
    for triangle_list in edge_triangles.values():
        if len(triangle_list) == 2:
            first, second = triangle_list
            triangle_adjacency[first].add(second)
            triangle_adjacency[second].add(first)
    visited = set()
    stack = [0]
    while stack:
        current = stack.pop()
        if current in visited:
            continue
        visited.add(current)
        stack.extend(triangle_adjacency[current] - visited)
    if len(visited) != len(selected_vertices):
        raise ValueError(
            "selector matched a disconnected patch: "
            f"{len(visited)} connected of {len(selected_vertices)} triangles"
        )

    vertex_coords = np.asarray(vertex_coords, dtype=np.float64)
    boundary_edges = [
        edge for edge, triangle_list in edge_triangles.items()
        if len(triangle_list) == 1
    ]
    boundary_adjacency = defaultdict(set)
    for first, second in boundary_edges:
        boundary_adjacency[first].add(second)
        boundary_adjacency[second].add(first)
    if any(
        len(neighbors) != 2
        for neighbors in boundary_adjacency.values()
    ):
        raise ValueError("boundary graph contains a non-manifold vertex")

    directed_next = {}
    for first, second in boundary_edges:
        second_options = [
            candidate for candidate in boundary_adjacency[second]
            if candidate != first
        ]
        first_options = [
            candidate for candidate in boundary_adjacency[first]
            if candidate != second
        ]
        if len(second_options) != 1 or len(first_options) != 1:
            raise ValueError("boundary graph cannot be traced as closed loops")
        directed_next[(first, second)] = (second, second_options[0])
        directed_next[(second, first)] = (first, first_options[0])

    unused_directed = set(directed_next)
    loop_vertex_ids = []
    while unused_directed:
        start = next(iter(unused_directed))
        directed_cycle = []
        current = start
        while current in unused_directed:
            unused_directed.remove(current)
            directed_cycle.append(current)
            current = directed_next[current]
        if current != start or len(directed_cycle) < 3:
            raise ValueError("boundary tracing did not close into a loop")
        undirected_cycle = {
            frozenset((first, second))
            for first, second in directed_cycle
        }
        opposite_directed = [
            edge for edge in unused_directed
            if frozenset(edge) in undirected_cycle
        ]
        unused_directed.difference_update(opposite_directed)
        loop_vertex_ids.append(
            [first for first, _second in directed_cycle]
        )

    u_axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    projection_normal = selector_normal_raw.copy()
    v_axis = np.cross(projection_normal, u_axis)
    if abs(np.linalg.norm(v_axis) - 1.0) > 1e-6:
        raise ValueError("plane projection basis is not orthonormal")

    loop_3d = []
    loop_2d = []
    loop_areas = []
    for ids in loop_vertex_ids:
        if len(ids) < 3 or len(set(ids)) < 3:
            raise ValueError("degenerate boundary loop")
        points_3d = vertex_coords[ids]
        points_2d = np.column_stack(
            (points_3d @ u_axis, points_3d @ v_axis)
        )
        area = abs(
            0.5
            * np.sum(
                points_2d[:, 0] * np.roll(points_2d[:, 1], -1)
                - points_2d[:, 1] * np.roll(points_2d[:, 0], -1)
            )
        )
        if area <= _PROJECTED_AREA_EPS:
            raise ValueError("degenerate zero-area boundary loop")
        loop_3d.append(points_3d)
        loop_2d.append(points_2d)
        loop_areas.append(float(area))

    outer_index = int(
        max(range(len(loop_areas)), key=lambda index: loop_areas[index])
    )
    outer_area = loop_areas[outer_index]
    if sum(
        1
        for index, area in enumerate(loop_areas)
        if index != outer_index and abs(area - outer_area) <= 1e-12
    ):
        raise ValueError("outer boundary loop is ambiguous")
    outer_ids = loop_vertex_ids[outer_index]
    outer_3d = loop_3d[outer_index]
    outer_2d = loop_2d[outer_index]

    anchor_ids = []
    for anchor_xy in selector["transition_anchors_xy_m"]:
        anchor_xy = np.asarray(anchor_xy, dtype=np.float64)
        if anchor_xy.shape != (2,) or not np.all(np.isfinite(anchor_xy)):
            raise ValueError("transition anchors must be finite length-2 arrays")
        matches = [
            vertex
            for vertex in outer_ids
            if (
                abs(vertex_coords[vertex][0] - anchor_xy[0])
                <= _ANCHOR_MATCH_TOL_M
                and abs(vertex_coords[vertex][1] - anchor_xy[1])
                <= _ANCHOR_MATCH_TOL_M
            )
        ]
        if len(matches) != 1:
            raise ValueError(
                "transition anchor is missing or ambiguous on the outer loop: "
                f"{anchor_xy.tolist()} matched {len(matches)} vertices"
            )
        anchor_ids.append(matches[0])
    if anchor_ids[0] == anchor_ids[1]:
        raise ValueError("transition anchors must be distinct outer vertices")

    first_index = outer_ids.index(anchor_ids[0])
    second_index = outer_ids.index(anchor_ids[1])
    ordered_outer = (
        outer_ids[first_index:] + outer_ids[:first_index]
    )
    second_ordered_index = ordered_outer.index(anchor_ids[1])
    path_a = ordered_outer[: second_ordered_index + 1]
    path_b = ordered_outer[second_ordered_index:] + [ordered_outer[0]]

    mean_abs_x_a = float(
        np.mean([abs(vertex_coords[vertex][0]) for vertex in path_a])
    )
    mean_abs_x_b = float(
        np.mean([abs(vertex_coords[vertex][0]) for vertex in path_b])
    )
    if abs(mean_abs_x_a - mean_abs_x_b) <= 1e-12:
        raise ValueError(
            "outer-loop transition paths have ambiguous mean abs(x)"
        )
    if mean_abs_x_a < mean_abs_x_b:
        step_high_path = path_a
        outer_drop_path = path_b
    else:
        step_high_path = path_b
        outer_drop_path = path_a

    def path_segments(path):
        segments_3d = []
        segments_2d = []
        for first, second in zip(path[:-1], path[1:]):
            segments_3d.append(
                [vertex_coords[first].tolist(), vertex_coords[second].tolist()]
            )
            segments_2d.append(
                [
                    np.array(
                        [
                            vertex_coords[first] @ u_axis,
                            vertex_coords[first] @ v_axis,
                        ]
                    ).tolist(),
                    np.array(
                        [
                            vertex_coords[second] @ u_axis,
                            vertex_coords[second] @ v_axis,
                        ]
                    ).tolist(),
                ]
            )
        return segments_3d, segments_2d

    step_high_3d, step_high_2d = path_segments(step_high_path)
    outer_drop_3d, outer_drop_2d = path_segments(outer_drop_path)
    covered_edges = set()
    for path in (step_high_path, outer_drop_path):
        for first, second in zip(path[:-1], path[1:]):
            edge = tuple(sorted((first, second)))
            if edge in covered_edges:
                raise ValueError("outer-loop edge classified more than once")
            covered_edges.add(edge)
    outer_vertex_set = set(outer_ids)
    expected_outer_edges = {
        edge for edge in boundary_edges
        if edge[0] in outer_vertex_set and edge[1] in outer_vertex_set
    }
    if covered_edges != expected_outer_edges:
        raise ValueError(
            "outer-loop edge classification does not cover every outer "
            "edge exactly once"
        )

    internal_loops_3d = [
        loop_3d[index] for index in range(len(loop_3d))
        if index != outer_index
    ]
    internal_loops_2d = [
        loop_2d[index] for index in range(len(loop_2d))
        if index != outer_index
    ]
    internal_loop_areas = [
        loop_areas[index] for index in range(len(loop_areas))
        if index != outer_index
    ]

    return _XiaolanSideExtraction(
        selected_triangle_count=int(len(selected_vertices)),
        selected_triangle_area_m2=selected_triangle_area,
        selected_bounds_m=selected_bounds,
        plane_normal=plane_normal,
        projection_normal=projection_normal,
        plane_offset_m=plane_offset,
        outer_loop_vertices_3d_m=outer_3d,
        outer_loop_vertices_2d_m=outer_2d,
        outer_loop_projected_area_m2=outer_area,
        internal_loops_3d_m=internal_loops_3d,
        internal_loops_2d_m=internal_loops_2d,
        internal_loop_areas_m2=internal_loop_areas,
        step_high_segments_3d_m=step_high_3d,
        step_high_segments_2d_m=step_high_2d,
        outer_drop_segments_3d_m=outer_drop_3d,
        outer_drop_segments_2d_m=outer_drop_2d,
        step_high_segment_count=len(step_high_3d),
        outer_drop_segment_count=len(outer_drop_3d),
    )


def _xiaolan_side_report(extraction, selector):
    tilt_rad = math.acos(
        float(np.clip(extraction.plane_normal @ HORIZONTAL_OUTWARD_NORMAL, -1.0, 1.0))
    )
    loops = [
        {
            "kind": "PLANNER_OUTER",
            "vertex_count": len(extraction.outer_loop_vertices_3d_m),
            "projected_area_m2": extraction.outer_loop_projected_area_m2,
            "vertices_3d_m": extraction.outer_loop_vertices_3d_m.tolist(),
            "vertices_2d_m": extraction.outer_loop_vertices_2d_m.tolist(),
        }
    ]
    for points_3d, points_2d, area in zip(
        extraction.internal_loops_3d_m,
        extraction.internal_loops_2d_m,
        extraction.internal_loop_areas_m2,
    ):
        loops.append(
            {
                "kind": "IGNORED_INTERNAL",
                "vertex_count": len(points_3d),
                "projected_area_m2": area,
                "vertices_3d_m": points_3d.tolist(),
                "vertices_2d_m": points_2d.tolist(),
            }
        )
    return {
        "selector": {
            "normal": [
                float(value) for value in selector["normal"]
            ],
            "plane_offset_m": float(selector["plane_offset_m"]),
            "normal_tolerance": float(selector["normal_tolerance"]),
            "plane_offset_tolerance_m": float(
                selector["plane_offset_tolerance_m"]
            ),
            "transition_anchors_xy_m": [
                [float(value) for value in anchor]
                for anchor in selector["transition_anchors_xy_m"]
            ],
        },
        "selected_triangle_count": extraction.selected_triangle_count,
        "selected_triangle_area_m2": extraction.selected_triangle_area_m2,
        "plane_normal": extraction.plane_normal.tolist(),
        "plane_offset_m": extraction.plane_offset_m,
        "tilt_from_horizontal_rad": tilt_rad,
        "tilt_from_horizontal_deg": math.degrees(tilt_rad),
        "bounds_3d_m": {
            "min": extraction.selected_bounds_m[0].tolist(),
            "max": extraction.selected_bounds_m[1].tolist(),
        },
        "boundary_loop_count": len(loops),
        "boundary_loops": loops,
        "ignored_internal_loop_count": len(
            extraction.internal_loops_3d_m
        ),
        "planner_outer_boundary_area_m2": (
            extraction.outer_loop_projected_area_m2
        ),
        "planner_outer_boundary_vertex_count": len(
            extraction.outer_loop_vertices_3d_m
        ),
        "planner_outer_boundary_vertices_3d_m": (
            extraction.outer_loop_vertices_3d_m.tolist()
        ),
        "planner_outer_boundary_vertices_2d_m": (
            extraction.outer_loop_vertices_2d_m.tolist()
        ),
        "step_high_segment_count": extraction.step_high_segment_count,
        "outer_drop_segment_count": extraction.outer_drop_segment_count,
        "step_high_segments_3d_m": extraction.step_high_segments_3d_m,
        "step_high_segments_2d_m": extraction.step_high_segments_2d_m,
        "outer_drop_segments_3d_m": extraction.outer_drop_segments_3d_m,
        "outer_drop_segments_2d_m": extraction.outer_drop_segments_2d_m,
    }


def build_xiaolan_cad_report(config, mesh_path):
    """生成两侧小蓝低斜面的 CAD 报告。

    参数:
        config: 攀爬配置。
        mesh_path: 小蓝 STL 文件路径。

    返回:
        两侧低斜面的几何信息。
    """

    cad = _xiaolan_cad_object(config)
    mesh_path = Path(mesh_path)
    actual_hash = _sha256_hex(mesh_path)
    expected_hash = cad["stl_sha256"]
    if actual_hash.lower() != expected_hash.lower():
        raise ValueError(
            "xiaolan STL SHA-256 mismatch: "
            f"expected {expected_hash}, got {actual_hash}"
        )
    normals, vertices = read_binary_stl(mesh_path)
    side_reports = {}
    for side_name in ("negative_x", "positive_x"):
        selector = cad[side_name]
        _validate_selector(selector)
        extraction = extract_xiaolan_side(vertices, normals, selector)
        side_reports[side_name] = _xiaolan_side_report(extraction, selector)
    return {
        "status": "MODEL_GEOMETRY_READY",
        "mesh_path": str(mesh_path),
        "stl_sha256": actual_hash.lower(),
        "internal_features_policy": cad["internal_features_policy"],
        "upper_surfaces_policy": cad["upper_surfaces_policy"],
        "upper_surfaces_height_range_m": [
            float(value) for value in cad["upper_surfaces_height_range_m"]
        ],
        "selector_tolerance_semantics": cad[
            "selector_tolerance_semantics"
        ],
        "negative_x": side_reports["negative_x"],
        "positive_x": side_reports["positive_x"],
        "note": (
            "Raw CAD geometry extraction only; no safety margin, IK, "
            "trajectory, contact/load proof, or real-robot authorization "
            "is implied."
        ),
    }


def _point_to_segment_distances(point_2d, segments_2d):
    point = np.asarray(point_2d, dtype=np.float64).reshape(2)
    segments = np.asarray(segments_2d, dtype=np.float64).reshape(-1, 2, 2)
    starts = segments[:, 0]
    ends = segments[:, 1]
    deltas = ends - starts
    lengths_sq = np.sum(deltas * deltas, axis=1)
    valid = lengths_sq > 1e-15
    if not np.any(valid):
        return np.inf
    starts = starts[valid]
    deltas = deltas[valid]
    lengths_sq = lengths_sq[valid]
    ratios = np.sum(
        (point - starts) * deltas,
        axis=1,
    ) / lengths_sq
    ratios = np.clip(ratios, 0.0, 1.0)
    closest = starts + ratios[:, np.newaxis] * deltas
    return float(np.min(np.linalg.norm(closest - point, axis=1)))


def _project_to_xiaolan_plane(point_3d, plane_normal):
    u_axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    v_axis = np.cross(plane_normal, u_axis)
    return np.array(
        [point_3d @ u_axis, point_3d @ v_axis],
        dtype=np.float64,
    )


def xiaolan_plane_distance_report(extraction, point):
    """计算一点到小蓝外边缘的距离。

    参数:
        extraction: 小蓝斜面提取结果。
        point: 平面二维坐标或小蓝三维坐标。

    返回:
        点是否在规划区域内及到两类边缘的距离。
    """

    point = np.asarray(point, dtype=np.float64).reshape(-1)
    if point.shape == (3,):
        point_2d = _project_to_xiaolan_plane(
            point,
            extraction.projection_normal,
        )
    elif point.shape == (2,):
        point_2d = point
    else:
        raise ValueError("point must be length 2 or 3")
    if not np.all(np.isfinite(point_2d)):
        raise ValueError("point must be finite")
    inside = bool(
        points_in_polygon(
            point_2d.reshape(1, 2),
            extraction.outer_loop_vertices_2d_m,
        )[0]
    )
    if not inside:
        raise ValueError("point is outside the planner outer polygon")
    return {
        "inside_planner_polygon": True,
        "outer_drop_distance_m": _point_to_segment_distances(
            point_2d,
            extraction.outer_drop_segments_2d_m,
        ),
        "step_high_distance_m": _point_to_segment_distances(
            point_2d,
            extraction.step_high_segments_2d_m,
        ),
    }


def build_baseline_report(config, mesh_path=None):
    """生成 M2/M3 的离线基础报告。

    参数:
        config: 攀爬配置。
        mesh_path: 可选的小蓝 STL 文件路径。

    返回:
        可写入 JSON 的离线报告。

    结果只反映配置和模型，不代表真实接触或稳定性。
    """

    climb.validate_climb_config(config)

    kinematic = k.GraspKinematic()
    feet_base = kinematic.forward_base(k.Q_STAND)
    hip_origin_x_delta = float(
        k.HIP_XYZ[5, 0] - 0.5 * (k.HIP_XYZ[3, 0] + k.HIP_XYZ[4, 0])
    )
    foot_center_x_delta = float(
        feet_base[5, 0] - 0.5 * (feet_base[3, 0] + feet_base[4, 0])
    )

    estimated_com = kinematic.center_of_mass_base(k.Q_STAND)
    minimum_joint_limit_margin = float(
        np.min(kinematic.joint_limit_margins(k.Q_STAND))
    )
    minimum_jacobian_singular_value = float(
        np.min(kinematic.jacobian_min_singular_values(k.Q_STAND))
    )
    maximum_terminal_axis_error = float(
        np.max(
            climb.terminal_axis_surface_angles(
                kinematic.terminal_axes_base(k.Q_STAND),
                HORIZONTAL_OUTWARD_NORMAL,
            )
        )
    )

    near_four_support = climb.gravity_projected_support(
        estimated_com,
        feet_base[[5, 1, 0, 2]],
        GRAVITY_DOWN,
    )
    far_tripod_support = climb.gravity_projected_support(
        estimated_com,
        feet_base[[1, 0, 2]],
        GRAVITY_DOWN,
    )

    m2_presence = climb.offline_input_presence(config, "M2")
    m3_presence = climb.offline_input_presence(config, "M3")
    both_inputs_present = (
        m2_presence.status == "INPUTS_PRESENT"
        and m3_presence.status == "INPUTS_PRESENT"
    )

    report = {
        "schema": REPORT_SCHEMA,
        "schema_version": REPORT_SCHEMA_VERSION,
        "config_status": config["status"],
        "known_nominal_values": {
            "deck_height_nominal_m": float(
                config["known"]["deck_height_nominal_m"]
            ),
            "rm_motor_envelope_rough_range_m": [
                float(value)
                for value in config["known"]["rm_motor_envelope_rough_range_m"]
            ],
            "hip_origin_delta_m": float(
                config["known"]["hip_origin_delta_m"]
            ),
            "foot_center_delta_nominal_m": float(
                config["known"]["foot_center_delta_nominal_m"]
            ),
            "model_foot_radius_m": float(
                config["known"]["model_foot_radius_m"]
            ),
        },
        "model_baseline": {
            "hip_origin_x_delta_rm_minus_mean_rb_rf_m": hip_origin_x_delta,
            "foot_center_x_delta_rm_minus_mean_rb_rf_m": foot_center_x_delta,
            "estimated_model_com_base_m": estimated_com.tolist(),
            "minimum_joint_limit_margin_rad": minimum_joint_limit_margin,
            "minimum_jacobian_singular_value_m_per_rad": (
                minimum_jacobian_singular_value
            ),
            "maximum_terminal_axis_error_rad": maximum_terminal_axis_error,
        },
        "geometry_only_support": {
            "near_four": {
                "legs": ["rm", "lf", "lb", "lm"],
                "valid": bool(near_four_support.valid),
                "raw_margin_m": float(near_four_support.raw_margin_m),
                "inside_raw_geometry": bool(
                    near_four_support.raw_margin_m >= 0.0
                ),
                "applied_uncertainty_radius_m": 0.0,
                "uncertainty_status": "REAL_COM_UNCERTAINTY_UNRESOLVED",
                "reason": str(near_four_support.reason),
            },
            "far_tripod": {
                "legs": ["lb", "lf", "lm"],
                "valid": bool(far_tripod_support.valid),
                "raw_margin_m": float(far_tripod_support.raw_margin_m),
                "inside_raw_geometry": bool(
                    far_tripod_support.raw_margin_m >= 0.0
                ),
                "applied_uncertainty_radius_m": 0.0,
                "uncertainty_status": "REAL_COM_UNCERTAINTY_UNRESOLVED",
                "reason": str(far_tripod_support.reason),
            },
        },
        "candidate_sequences": [
            {
                "name": spec.name,
                "p0_strategy": spec.p0_strategy,
                "initial_platform_groups": [
                    list(group) for group in spec.initial_platform_groups
                ],
            }
            for spec in climb.climb_sequence_specs()
        ],
        "readiness_gates": {
            "M2": {
                "milestone": m2_presence.milestone,
                "status": m2_presence.status,
                "missing_paths": list(m2_presence.missing_paths),
                "note": m2_presence.note,
            },
            "M3": {
                "milestone": m3_presence.milestone,
                "status": m3_presence.status,
                "missing_paths": list(m3_presence.missing_paths),
                "note": m3_presence.note,
            },
        },
        "overall_status": (
            "READY_FOR_OFFLINE_SEARCH"
            if both_inputs_present
            else "UNRESOLVED"
        ),
        "limitations": list(LIMITATIONS),
    }
    if mesh_path is not None:
        report["xiaolan_cad_geometry"] = build_xiaolan_cad_report(
            config,
            mesh_path,
        )
    return _finalize_report(report)


def _default_config_path():
    return SCRIPTS_DIR.parent / "config" / "climb.json"


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "检查 M2/M3 离线输入，并生成模型基础报告"
        )
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=_default_config_path(),
        metavar="PATH",
        help="攀爬配置 JSON，默认使用包内 config/climb.json",
    )
    parser.add_argument(
        "--xiaolan-mesh",
        type=Path,
        default=_XIAOLAN_DEFAULT_MESH_PATH,
        metavar="PATH",
        help=(
            "小蓝 base_link STL，默认使用仓库中的 "
            "grasp_hexapod_description/meshes/xiaolan/"
            "base_link_xiaolan.STL)"
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        metavar="PATH",
        help="将格式化 UTF-8 JSON 写入 PATH，不输出到终端",
    )
    parser.add_argument(
        "--require-m2-inputs",
        action="store_true",
        help="M2 输入未解决时以退出码 2 结束",
    )
    args = parser.parse_args(argv)

    with args.config.open("r", encoding="utf-8") as config_file:
        config = json.load(config_file)
    report = build_baseline_report(config, args.xiaolan_mesh)
    text = (
        json.dumps(
            report,
            indent=2,
            sort_keys=True,
            ensure_ascii=False,
            allow_nan=False,
        )
        + "\n"
    )

    if args.output is None:
        sys.stdout.write(text)
    else:
        args.output.write_text(text, encoding="utf-8")

    if (
        args.require_m2_inputs
        and report["readiness_gates"]["M2"]["status"] == "UNRESOLVED"
    ):
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
