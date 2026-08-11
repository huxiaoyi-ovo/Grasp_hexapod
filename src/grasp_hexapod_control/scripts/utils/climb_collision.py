"""Pure-NumPy visual-CAD collision queries for the offline climb planner.

This module deliberately consumes the visual STL meshes, not URDF collision
primitives.  It is a deterministic model-geometry diagnostic only; a clear
query is not evidence of real-world clearance, contact, or load support.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import hashlib
import struct
import numpy as np

import kinematics as k


EPSILON_M = 1.0e-9
ROUND_DECIMALS = 9
LINK_NAMES = ("thigh", "knee", "ankle")

# 当前踝关节可视 STL 的第 10 个连通块是实际脚端/脚垫。
# 只允许按这个网格哈希排除该块，不能把整条链路都忽略。
ANKLE_VISUAL_FOOTPAD_COMPONENT_INDEX = 10
ANKLE_VISUAL_FOOTPAD_MESH_SHA256 = (
    "1f48a886fce0e2effc51b481d9bee94e637b5d7c7f29e59074a264575a0db5c9"
)
ANKLE_VISUAL_FOOTPAD_SEMANTICS = "physical_terminal_foot_or_footpad_visual"


def _require_finite(value, name):
    if not np.all(np.isfinite(value)):
        raise ValueError(name + " must be finite")


def read_binary_stl(path: Path) -> np.ndarray:
    """读取二进制 STL 三角面片。

    参数:
        path: STL 文件路径。

    返回:
        局部坐标三角面片，shape 为 `(N, 3, 3)`。
    """
    path = Path(path)
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError("binary STL is shorter than its header: " + str(path))
    count = struct.unpack_from("<I", data, 80)[0]
    expected_size = 84 + 50 * count
    if len(data) != expected_size:
        raise ValueError("binary STL size mismatch: " + str(path))
    triangles = np.empty((count, 3, 3), dtype=np.float64)
    offset = 84
    for index in range(count):
        # 面片法向量不用，后续直接由顶点计算。
        triangles[index] = np.frombuffer(
            data, dtype="<f4", count=9, offset=offset + 12
        ).reshape(3, 3)
        offset += 50
    _require_finite(triangles, "STL vertices")
    return triangles


def _triangle_bounds(triangles: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return np.min(triangles, axis=1), np.max(triangles, axis=1)


def connected_triangle_components(triangles: np.ndarray) -> tuple[np.ndarray, ...]:
    """按共享顶点拆分网格连通块。

    参数:
        triangles: 三角面片，shape 为 `(N, 3, 3)`。

    返回:
        每个连通块的三角面片索引。
    """
    triangles = np.asarray(triangles, dtype=np.float64)
    _require_finite(triangles, "component triangles")
    if triangles.ndim != 3 or triangles.shape[1:] != (3, 3):
        raise ValueError("triangles must have shape (N, 3, 3)")
    parent = np.arange(len(triangles), dtype=np.int64)

    def find(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def join(left, right):
        left, right = find(left), find(right)
        if left != right:
            parent[right] = left

    owner = {}
    for tri_index, tri in enumerate(np.round(triangles, ROUND_DECIMALS)):
        for vertex in tri:
            key = tuple(vertex.tolist())
            previous = owner.setdefault(key, tri_index)
            join(tri_index, previous)
    groups = {}
    for tri_index in range(len(triangles)):
        groups.setdefault(find(tri_index), []).append(tri_index)
    return tuple(
        np.asarray(groups[root], dtype=np.int64) for root in sorted(groups)
    )


def aabb_overlaps(min_a, max_a, min_b, max_b, tolerance=EPSILON_M) -> bool:
    """检查两个 AABB 是否相交。

    参数:
        min_a: 第一个 AABB 的最小坐标。
        max_a: 第一个 AABB 的最大坐标。
        min_b: 第二个 AABB 的最小坐标。
        max_b: 第二个 AABB 的最大坐标。
        tolerance: 相交容差。

    返回:
        是否相交或接近接触。
    """
    min_a, max_a = np.asarray(min_a), np.asarray(max_a)
    min_b, max_b = np.asarray(min_b), np.asarray(max_b)
    _require_finite(np.stack((min_a, max_a, min_b, max_b)), "AABB")
    return bool(np.all(max_a + tolerance >= min_b) and np.all(max_b + tolerance >= min_a))


def _point_in_triangle_2d(point, tri, tolerance):
    cross = []
    for index in range(3):
        edge = tri[(index + 1) % 3] - tri[index]
        relative = point - tri[index]
        cross.append(edge[0] * relative[1] - edge[1] * relative[0])
    return min(cross) >= -tolerance or max(cross) <= tolerance


def _segment_intersects_2d(a0, a1, b0, b1, tolerance):
    def orient(p0, p1, p2):
        edge, relative = p1 - p0, p2 - p0
        return edge[0] * relative[1] - edge[1] * relative[0]

    o1, o2 = orient(a0, a1, b0), orient(a0, a1, b1)
    o3, o4 = orient(b0, b1, a0), orient(b0, b1, a1)
    if ((o1 > tolerance and o2 < -tolerance) or (o1 < -tolerance and o2 > tolerance)) and ((o3 > tolerance and o4 < -tolerance) or (o3 < -tolerance and o4 > tolerance)):
        return True
    for point, start, end, value in ((b0, a0, a1, o1), (b1, a0, a1, o2), (a0, b0, b1, o3), (a1, b0, b1, o4)):
        if abs(value) <= tolerance and np.all(point >= np.minimum(start, end) - tolerance) and np.all(point <= np.maximum(start, end) + tolerance):
            return True
    return False


def _coplanar_triangles_intersect(tri_a, tri_b, normal, tolerance):
    # 去掉法向量分量最大的轴，转到稳定的二维平面判断。
    keep = np.delete(np.arange(3), int(np.argmax(np.abs(normal))))
    a2, b2 = tri_a[:, keep], tri_b[:, keep]
    for first in (a2, b2):
        second = b2 if first is a2 else a2
        for index in range(3):
            for other in range(3):
                if _segment_intersects_2d(first[index], first[(index + 1) % 3], second[other], second[(other + 1) % 3], tolerance):
                    return True
    return _point_in_triangle_2d(a2[0], b2, tolerance) or _point_in_triangle_2d(b2[0], a2, tolerance)


def _segment_triangle_intersects(start, end, triangle, tolerance):
    """检查线段是否穿过三角形。

    参数:
        start: 线段起点。
        end: 线段终点。
        triangle: 三角形顶点。
        tolerance: 判断容差。

    返回:
        是否相交。
    """
    direction = end - start
    edge0, edge1 = triangle[1] - triangle[0], triangle[2] - triangle[0]
    cross = np.cross(direction, edge1)
    determinant = float(np.dot(edge0, cross))
    determinant_scale = np.linalg.norm(edge0) * np.linalg.norm(cross)
    if determinant_scale == 0.0 or abs(determinant) <= tolerance * determinant_scale:
        return False
    inverse = 1.0 / determinant
    relative = start - triangle[0]
    u = inverse * float(np.dot(relative, cross))
    if u < -tolerance or u > 1.0 + tolerance:
        return False
    qvec = np.cross(relative, edge0)
    v = inverse * float(np.dot(direction, qvec))
    if v < -tolerance or u + v > 1.0 + tolerance:
        return False
    distance = inverse * float(np.dot(edge1, qvec))
    return -tolerance <= distance <= 1.0 + tolerance


def triangles_intersect(tri_a, tri_b, tolerance=EPSILON_M) -> bool:
    """检查两个三角形是否相交。

    参数:
        tri_a: 第一个三角形的顶点。
        tri_b: 第二个三角形的顶点。
        tolerance: 判断容差。

    返回:
        是否相交或接触。

    退化三角形只要包围盒重叠，也按碰撞处理。
    """
    tri_a = np.asarray(tri_a, dtype=np.float64).reshape(3, 3)
    tri_b = np.asarray(tri_b, dtype=np.float64).reshape(3, 3)
    _require_finite(np.stack((tri_a, tri_b)), "triangle")
    if not aabb_overlaps(*_triangle_bounds(tri_a[None, ...]), *_triangle_bounds(tri_b[None, ...]), tolerance):
        return False
    normal_a = np.cross(tri_a[1] - tri_a[0], tri_a[2] - tri_a[0])
    normal_b = np.cross(tri_b[1] - tri_b[0], tri_b[2] - tri_b[0])
    norm_a, norm_b = np.linalg.norm(normal_a), np.linalg.norm(normal_b)
    if norm_a <= tolerance * tolerance or norm_b <= tolerance * tolerance:
        return True
    cross_normal_sine = np.linalg.norm(np.cross(normal_a, normal_b)) / (norm_a * norm_b)
    plane_a = np.max(np.abs((tri_b - tri_a[0]) @ normal_a)) / norm_a
    plane_b = np.max(np.abs((tri_a - tri_b[0]) @ normal_b)) / norm_b
    if cross_normal_sine <= tolerance and plane_a <= tolerance and plane_b <= tolerance:
        return _coplanar_triangles_intersect(tri_a, tri_b, normal_a, tolerance)
    for triangle, other in ((tri_a, tri_b), (tri_b, tri_a)):
        for index in range(3):
            if _segment_triangle_intersects(triangle[index], triangle[(index + 1) % 3], other, tolerance):
                return True
    return False


def triangle_intersection_regressions():
    """运行三角形相交的固定回归用例。

    返回:
        每个用例是否通过。
    """
    flat = np.array(((0., 0., 0.), (1., 0., 0.), (0., 1., 0.)))
    cases = (
        (flat, np.array(((.2, .2, -1.), (.2, .2, 1.), (.8, .2, 0.))), True),
        (flat, np.array(((.2, .2, 0.), (.8, .2, 0.), (.2, .8, 0.))), True),
        (flat, np.array(((2., 0., 0.), (3., 0., 0.), (2., 1., 0.))), False),
        (flat, np.array(((1., 0., 0.), (1.2, 0., .2), (1., .2, .2))), True),
        (flat, np.array(((.2, .2, 2.e-9), (.8, .2, 2.e-9), (.2, .8, 2.e-9))), False),
    )
    return tuple(triangles_intersect(left, right) == expected for left, right, expected in cases)


def point_triangle_distance_squared(point, triangle):
    """计算点到三角形的距离平方。

    参数:
        point: 点坐标。
        triangle: 三角形顶点。

    返回:
        最短距离的平方。
    """
    point = np.asarray(point, dtype=np.float64).reshape(3)
    a, b, c = np.asarray(triangle, dtype=np.float64).reshape(3, 3)
    ab, ac, ap = b - a, c - a, point - a
    d1, d2 = np.dot(ab, ap), np.dot(ac, ap)
    if d1 <= 0.0 and d2 <= 0.0:
        return float(np.dot(ap, ap))
    bp = point - b
    d3, d4 = np.dot(ab, bp), np.dot(ac, bp)
    if d3 >= 0.0 and d4 <= d3:
        return float(np.dot(bp, bp))
    vc = d1 * d4 - d3 * d2
    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        v = d1 / (d1 - d3)
        delta = point - (a + v * ab)
        return float(np.dot(delta, delta))
    cp = point - c
    d5, d6 = np.dot(ab, cp), np.dot(ac, cp)
    if d6 >= 0.0 and d5 <= d6:
        return float(np.dot(cp, cp))
    vb = d5 * d2 - d1 * d6
    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        w = d2 / (d2 - d6)
        delta = point - (a + w * ac)
        return float(np.dot(delta, delta))
    va = d3 * d6 - d5 * d4
    if va <= 0.0 and d4 - d3 >= 0.0 and d5 - d6 >= 0.0:
        edge = c - b
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        delta = point - (b + w * edge)
        return float(np.dot(delta, delta))
    denominator = 1.0 / (va + vb + vc)
    projection = a + ab * (vb * denominator) + ac * (vc * denominator)
    delta = point - projection
    return float(np.dot(delta, delta))


def _ray_triangle_distance(origin, direction, triangle):
    edge0, edge1 = triangle[1] - triangle[0], triangle[2] - triangle[0]
    cross = np.cross(direction, edge1)
    determinant = float(np.dot(edge0, cross))
    scale = np.linalg.norm(edge0) * np.linalg.norm(cross)
    if scale == 0.0 or abs(determinant) <= EPSILON_M * scale:
        return None
    relative = origin - triangle[0]
    inverse = 1.0 / determinant
    u = inverse * float(np.dot(relative, cross))
    if u < -EPSILON_M or u > 1.0 + EPSILON_M:
        return None
    qvec = np.cross(relative, edge0)
    v = inverse * float(np.dot(direction, qvec))
    if v < -EPSILON_M or u + v > 1.0 + EPSILON_M:
        return None
    distance = inverse * float(np.dot(edge1, qvec))
    return distance if distance > EPSILON_M else None


def point_in_closed_mesh(point, mesh):
    """检查点是否在封闭网格内或边界上。

    参数:
        point: 点坐标。
        mesh: 封闭可视网格。

    返回:
        点是否在网格内或边界上。
    """
    point = np.asarray(point, dtype=np.float64).reshape(3)
    _require_finite(point, "containment point")
    minimum, maximum = np.min(mesh.triangles, axis=(0, 1)), np.max(mesh.triangles, axis=(0, 1))
    if np.any(point < minimum - EPSILON_M) or np.any(point > maximum + EPSILON_M):
        return False
    # 边界也算碰撞，只检查包围盒覆盖该点的面片。
    minimum_tri, maximum_tri = _triangle_bounds(mesh.triangles)
    boundary = np.flatnonzero(
        np.all(maximum_tri + EPSILON_M >= point, axis=1)
        & np.all(minimum_tri <= point + EPSILON_M, axis=1)
    )
    for index in boundary:
        if point_triangle_distance_squared(point, mesh.triangles[index]) <= EPSILON_M * EPSILON_M:
            return True
    direction = np.array((0.3713906763541037, 0.5570860145311556, 0.7427813527082074))
    triangles = mesh.triangles
    edge0, edge1 = triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
    cross = np.cross(np.broadcast_to(direction, edge1.shape), edge1)
    determinant = np.einsum("ij,ij->i", edge0, cross)
    scale = np.linalg.norm(edge0, axis=1) * np.linalg.norm(cross, axis=1)
    valid = np.abs(determinant) > EPSILON_M * scale
    relative = point - triangles[:, 0]
    inverse = np.zeros_like(determinant)
    inverse[valid] = 1.0 / determinant[valid]
    u = inverse * np.einsum("ij,ij->i", relative, cross)
    qvec = np.cross(relative, edge0)
    v = inverse * np.einsum("ij,j->i", qvec, direction)
    distance = inverse * np.einsum("ij,ij->i", edge1, qvec)
    valid &= (u >= -EPSILON_M) & (u <= 1.0 + EPSILON_M)
    valid &= (v >= -EPSILON_M) & (u + v <= 1.0 + EPSILON_M)
    valid &= distance > EPSILON_M
    distances = distance[valid].tolist()
    if not distances:
        return False
    distances.sort()
    unique = [distances[0]]
    for distance in distances[1:]:
        if distance > unique[-1] + EPSILON_M * max(1.0, abs(distance)):
            unique.append(distance)
    return bool(len(unique) % 2)


@dataclass(frozen=True)
class VisualComponent:
    """保存一个可视网格连通块的三角面片和包围盒。"""

    link_name: str
    component_index: int
    triangle_indices: np.ndarray
    bounds_min: np.ndarray
    bounds_max: np.ndarray


@dataclass(frozen=True)
class VisualMesh:
    """保存一个连杆的可视网格。"""

    link_name: str
    path: Path
    triangles: np.ndarray
    components: tuple[VisualComponent, ...]
    sha256: str

    @classmethod
    def load(cls, link_name, path, split_components=False):
        triangles = read_binary_stl(path)
        groups = connected_triangle_components(triangles) if split_components else (np.arange(len(triangles)),)
        components = []
        for component_index, indices in enumerate(groups):
            subset = triangles[indices]
            components.append(VisualComponent(link_name, component_index, indices, np.min(subset, axis=(0, 1)), np.max(subset, axis=(0, 1))))
        return cls(link_name, Path(path), triangles, tuple(components), hashlib.sha256(Path(path).read_bytes()).hexdigest())


def require_ankle_visual_footpad_binding(mesh):
    """获取与哈希匹配的踝关节脚垫连通块。

    参数:
        mesh: 踝关节可视网格。

    返回:
        脚垫连通块。
    """
    if mesh.link_name != "ankle":
        raise ValueError("footpad binding requires the ankle visual mesh")
    if mesh.sha256 != ANKLE_VISUAL_FOOTPAD_MESH_SHA256:
        raise ValueError("ankle visual mesh hash changed; re-audit footpad component")
    matches = tuple(
        component
        for component in mesh.components
        if component.component_index == ANKLE_VISUAL_FOOTPAD_COMPONENT_INDEX
    )
    if len(matches) != 1:
        raise ValueError("hash-bound ankle footpad component is missing")
    return matches[0]


def visual_mesh_from_triangles(link_name, triangles, path=Path("<memory>")):
    """从三角面片创建内存中的可视网格。

    参数:
        link_name: 连杆名称。
        triangles: 三角面片。
        path: 用于标识网格的路径。

    返回:
        可视网格对象。
    """
    triangles = np.asarray(triangles, dtype=np.float64).reshape(-1, 3, 3)
    _require_finite(triangles, "visual mesh triangles")
    indices = np.arange(len(triangles), dtype=np.int64)
    component = VisualComponent(
        link_name, 0, indices, np.min(triangles, axis=(0, 1)), np.max(triangles, axis=(0, 1))
    )
    return VisualMesh(
        link_name, Path(path), triangles, (component,), hashlib.sha256(triangles.tobytes()).hexdigest()
    )


def transformed_visual_mesh(mesh, transform):
    transform = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    _require_finite(transform, "world_from_xiaolan")
    triangles = mesh.triangles @ transform[:3, :3].T + transform[:3, 3]
    components = tuple(
        VisualComponent(
            mesh.link_name,
            component.component_index,
            component.triangle_indices,
            np.min(triangles[component.triangle_indices], axis=(0, 1)),
            np.max(triangles[component.triangle_indices], axis=(0, 1)),
        )
        for component in mesh.components
    )
    return VisualMesh(mesh.link_name, mesh.path, triangles, components, mesh.sha256)


@dataclass(frozen=True)
class TransformedComponent:
    """保存变换到世界坐标的网格连通块。"""

    mesh: VisualMesh
    component: VisualComponent
    leg_index: int | None
    triangles: np.ndarray
    bounds_min: np.ndarray
    bounds_max: np.ndarray


@dataclass(frozen=True)
class FootSphere:
    """保存一只脚的球形碰撞模型。"""

    leg_index: int
    center: np.ndarray
    radius_m: float


@dataclass(frozen=True)
class CollisionHit:
    """记录一对发生碰撞的三角面片。"""

    left_link: str
    left_leg: int | None
    left_component: int
    left_triangle: int
    right_link: str
    right_leg: int | None
    right_component: int
    right_triangle: int


@dataclass(frozen=True)
class CollisionResult:
    """保存一次碰撞查询的结果和检查数量。"""

    collision: bool
    hit: CollisionHit | None
    broad_phase_candidates: int
    narrow_phase_tests: int
    visual_narrow_phase_used: bool = True


class XiaolanTriangleIndex:
    """保存小蓝三角面片的 AABB 查询索引。"""

    def __init__(self, mesh: VisualMesh):
        self.mesh = mesh
        self.minimum, self.maximum = _triangle_bounds(mesh.triangles)
        self.order = np.argsort(self.minimum[:, 0], kind="mergesort")
        self.sorted_min_x = self.minimum[self.order, 0]

    def candidates(self, bounds_min, bounds_max):
        stop = np.searchsorted(self.sorted_min_x, bounds_max[0] + EPSILON_M, side="right")
        indices = self.order[:stop]
        mask = np.all(self.maximum[indices] + EPSILON_M >= bounds_min, axis=1)
        mask &= np.all(self.minimum[indices] <= bounds_max + EPSILON_M, axis=1)
        return indices[mask]


class VisualCollisionScene:
    """保存机器人和小蓝的可视碰撞网格。"""

    def __init__(self, body_mesh, link_meshes, xiaolan_mesh, world_from_xiaolan=None):
        self.body_mesh = body_mesh
        self.link_meshes = link_meshes
        self.xiaolan_mesh_local = xiaolan_mesh
        self.world_from_xiaolan = np.eye(4, dtype=np.float64) if world_from_xiaolan is None else np.asarray(world_from_xiaolan, dtype=np.float64).reshape(4, 4)
        _require_finite(self.world_from_xiaolan, "world_from_xiaolan")
        self.xiaolan_mesh = transformed_visual_mesh(xiaolan_mesh, self.world_from_xiaolan)
        self.xiaolan_index = XiaolanTriangleIndex(self.xiaolan_mesh)

    @staticmethod
    def _transform_component(mesh, component, transform, leg_index):
        transform = np.asarray(transform, dtype=np.float64).reshape(4, 4)
        _require_finite(transform, "link transform")
        local = mesh.triangles[component.triangle_indices]
        world = local @ transform[:3, :3].T + transform[:3, 3]
        return TransformedComponent(mesh, component, leg_index, world, np.min(world, axis=(0, 1)), np.max(world, axis=(0, 1)))

    def robot_components(self, joint_angles, world_from_base, leg_indices=None, include_body=True):
        """计算指定机器人部件的世界坐标网格。

        参数:
            joint_angles: 六条腿的关节角度，shape 为 `(6, 3)`。
            world_from_base: `base_link` 到世界坐标的变换矩阵。
            leg_indices: 要包含的腿索引，默认全部。
            include_body: 是否包含机身。

        返回:
            变换后的网格部件。
        """
        joint_angles = np.asarray(joint_angles, dtype=np.float64).reshape(6, 3)
        world_from_base = np.asarray(world_from_base, dtype=np.float64).reshape(4, 4)
        _require_finite(joint_angles, "joint angles")
        _require_finite(world_from_base, "world_from_base")
        if leg_indices is None:
            leg_indices = range(6)
        else:
            leg_indices = tuple(sorted({int(index) for index in leg_indices}))
            if any(index < 0 or index >= 6 for index in leg_indices):
                raise ValueError("leg_indices must be in [0, 5]")
        model = k.GraspKinematic()
        base_from_link = model.link_transforms_base(joint_angles)
        output = []
        if include_body:
            output.extend(
                self._transform_component(self.body_mesh, component, world_from_base, None)
                for component in self.body_mesh.components
            )
        for leg_index in leg_indices:
            for link_index, link_name in enumerate(LINK_NAMES):
                mesh = self.link_meshes[link_name]
                transform = world_from_base @ base_from_link[leg_index, link_index]
                output.extend(self._transform_component(mesh, component, transform, leg_index) for component in mesh.components)
        return tuple(output)

    def transformed_robot_component(
        self,
        joint_angles,
        world_from_base,
        link_name,
        leg_index,
        component_index,
    ):
        """获取一个指定腿部件的世界坐标网格。

        参数:
        joint_angles: 当前关节角。
        world_from_base: 机身位姿。
        link_name: 连杆名称。
        leg_index: 腿索引。
        component_index: 连通块索引。

        返回:
            变换后的网格部件。
        """
        if link_name not in LINK_NAMES:
            raise ValueError("link_name must name a leg visual link")
        leg_index = int(leg_index)
        component_index = int(component_index)
        if leg_index < 0 or leg_index >= 6:
            raise ValueError("leg index must be in [0, 5]")
        mesh = self.link_meshes[link_name]
        matches = tuple(
            component
            for component in mesh.components
            if component.component_index == component_index
        )
        if len(matches) != 1:
            raise ValueError("visual component index is not unique")
        model = k.GraspKinematic()
        transforms = model.link_transforms_base(joint_angles)
        link_index = LINK_NAMES.index(link_name)
        transform = (
            np.asarray(world_from_base, dtype=np.float64).reshape(4, 4)
            @ transforms[leg_index, link_index]
        )
        return self._transform_component(
            mesh, matches[0], transform, leg_index
        )

    def foot_spheres(self, joint_angles, world_from_base):
        joint_angles = np.asarray(joint_angles, dtype=np.float64).reshape(6, 3)
        world_from_base = np.asarray(world_from_base, dtype=np.float64).reshape(4, 4)
        _require_finite(joint_angles, "joint angles")
        _require_finite(world_from_base, "world_from_base")
        centers_base = k.GraspKinematic().forward_base(joint_angles)
        centers_world = centers_base @ world_from_base[:3, :3].T + world_from_base[:3, 3]
        return tuple(FootSphere(index, center, float(k.FOOT_RADIUS)) for index, center in enumerate(centers_world))

    @staticmethod
    def _ignored(link_name, leg_index, ignore_links):
        return (link_name, leg_index) in ignore_links

    def _containment_hit(self, component):
        xiaolan_min = np.min(self.xiaolan_mesh.triangles, axis=(0, 1))
        xiaolan_max = np.max(self.xiaolan_mesh.triangles, axis=(0, 1))
        # 部件完全落在小蓝外包围盒内时，才检查没有表面相交的穿入。
        if np.any(component.bounds_min < xiaolan_min - EPSILON_M) or np.any(component.bounds_max > xiaolan_max + EPSILON_M):
            return None
        # 用首尾顶点和中心点检查没有表面相交的穿入。
        vertices = component.triangles.reshape(-1, 3)
        representatives = (vertices[0], vertices[-1], np.mean(vertices, axis=0))
        for representative in representatives:
            if point_in_closed_mesh(representative, self.xiaolan_mesh):
                return CollisionHit(component.mesh.link_name, component.leg_index, component.component.component_index, int(component.component.triangle_indices[0]), "xiaolan", None, 0, -1)
        return None

    def components_vs_xiaolan(
        self,
        components,
        ignore_links=(),
        exclude_components=(),
    ):
        """检查指定网格部件与小蓝是否碰撞。

        参数:
            components: 已变换到世界坐标的网格部件。
            ignore_links: 要跳过的 `(link_name, leg_index)`。
            exclude_components: 要跳过的 `(link_name, leg_index, component_index)`。

        返回:
            碰撞结果和命中的面片信息。
        """
        broad, narrow = 0, 0
        ignored = frozenset(ignore_links)
        excluded = frozenset(
            (str(link), int(leg), int(component))
            for link, leg, component in exclude_components
        )
        for component in components:
            if self._ignored(component.mesh.link_name, component.leg_index, ignored):
                continue
            component_key = (
                component.mesh.link_name,
                component.leg_index,
                component.component.component_index,
            )
            if component_key in excluded:
                continue
            # 先用部件包围盒排除，再按单个三角形精查。
            if len(self.xiaolan_index.candidates(component.bounds_min, component.bounds_max)) == 0:
                continue
            for local_index, robot_tri in enumerate(component.triangles):
                robot_min, robot_max = _triangle_bounds(robot_tri[None, ...])
                candidate_indices = self.xiaolan_index.candidates(robot_min[0], robot_max[0])
                broad += len(candidate_indices)
                for xiaolan_index in candidate_indices:
                    narrow += 1
                    if triangles_intersect(robot_tri, self.xiaolan_mesh.triangles[xiaolan_index]):
                        return CollisionResult(True, CollisionHit(component.mesh.link_name, component.leg_index, component.component.component_index, int(component.component.triangle_indices[local_index]), "xiaolan", None, 0, int(xiaolan_index)), broad, narrow)
            containment = self._containment_hit(component)
            if containment is not None:
                return CollisionResult(True, containment, broad, narrow)
        return CollisionResult(False, None, broad, narrow)

    def component_xiaolan_triangle_hits(self, component):
        """找出一个部件与小蓝相交的所有三角形。

        参数:
            component: 已变换到世界坐标的网格部件。

        返回:
            排序后的三角形索引；`-1` 表示完全穿入。
        """
        if not isinstance(component, TransformedComponent):
            raise TypeError("component must be a TransformedComponent")
        hits = set()
        if len(
            self.xiaolan_index.candidates(
                component.bounds_min, component.bounds_max
            )
        ):
            for robot_triangle in component.triangles:
                minimum, maximum = _triangle_bounds(
                    robot_triangle[None, ...]
                )
                candidates = self.xiaolan_index.candidates(
                    minimum[0], maximum[0]
                )
                for triangle_index in candidates:
                    if triangles_intersect(
                        robot_triangle,
                        self.xiaolan_mesh.triangles[triangle_index],
                    ):
                        hits.add(int(triangle_index))
        if not hits and self._containment_hit(component) is not None:
            hits.add(-1)
        return tuple(sorted(hits))

    def feet_vs_components(self, feet, components):
        broad = narrow = 0
        for foot in feet:
            for component in components:
                if component.leg_index == foot.leg_index and component.mesh.link_name == "ankle":
                    continue
                if not aabb_overlaps(foot.center - foot.radius_m, foot.center + foot.radius_m, component.bounds_min, component.bounds_max):
                    continue
                broad += 1
                for index, triangle in enumerate(component.triangles):
                    minimum, maximum = _triangle_bounds(triangle[None, ...])
                    if not aabb_overlaps(foot.center - foot.radius_m, foot.center + foot.radius_m, minimum[0], maximum[0]):
                        continue
                    narrow += 1
                    if point_triangle_distance_squared(foot.center, triangle) <= (foot.radius_m + EPSILON_M) ** 2:
                        return CollisionResult(True, CollisionHit("foot", foot.leg_index, 0, -1, component.mesh.link_name, component.leg_index, component.component.component_index, int(component.component.triangle_indices[index])), broad, narrow)
        return CollisionResult(False, None, broad, narrow)

    def feet_vs_xiaolan(self, joint_angles, world_from_base, ignore_links=()):
        """检查解析脚球与小蓝是否碰撞。

        参数:
            joint_angles: 六条腿的关节角度。
            world_from_base: `base_link` 到世界坐标的变换矩阵。
            ignore_links: 要跳过的脚。

        返回:
            碰撞结果和命中的面片信息。
        """
        broad, narrow = 0, 0
        ignored = frozenset(ignore_links)
        for foot in self.foot_spheres(joint_angles, world_from_base):
            if self._ignored("foot", foot.leg_index, ignored):
                continue
            minimum, maximum = foot.center - foot.radius_m, foot.center + foot.radius_m
            candidate_indices = self.xiaolan_index.candidates(minimum, maximum)
            broad += len(candidate_indices)
            for xiaolan_index in candidate_indices:
                narrow += 1
                if point_triangle_distance_squared(foot.center, self.xiaolan_mesh.triangles[xiaolan_index]) <= (foot.radius_m + EPSILON_M) ** 2:
                    return CollisionResult(True, CollisionHit("foot", foot.leg_index, 0, -1, "xiaolan", None, 0, int(xiaolan_index)), broad, narrow)
            # 整个脚球穿入封闭物体时，可能没有表面相交。
            if point_in_closed_mesh(foot.center, self.xiaolan_mesh):
                return CollisionResult(True, CollisionHit("foot", foot.leg_index, 0, -1, "xiaolan", None, 0, -1), broad, narrow)
        return CollisionResult(False, None, broad, narrow)

    def foot_xiaolan_triangle_hits(self, leg_index, joint_angles, world_from_base):
        """找出一只脚与小蓝相交的所有三角形。

        参数:
            leg_index: 腿索引。
            joint_angles: 六条腿的关节角度。
            world_from_base: `base_link` 到世界坐标的变换矩阵。

        返回:
            命中的小蓝三角形索引。
        """
        leg_index = int(leg_index)
        if leg_index < 0 or leg_index >= 6:
            raise ValueError("foot leg index must be in [0, 5]")
        foot = self.foot_spheres(joint_angles, world_from_base)[leg_index]
        minimum, maximum = foot.center - foot.radius_m, foot.center + foot.radius_m
        hits = []
        for triangle_index in self.xiaolan_index.candidates(minimum, maximum):
            if point_triangle_distance_squared(foot.center, self.xiaolan_mesh.triangles[triangle_index]) <= (foot.radius_m + EPSILON_M) ** 2:
                hits.append(int(triangle_index))
        if point_in_closed_mesh(foot.center, self.xiaolan_mesh) and not hits:
            hits.append(-1)
        return tuple(hits)

    def robot_vs_xiaolan(self, joint_angles, world_from_base, ignore_links=()):
        components_result = self.components_vs_xiaolan(
            self.robot_components(joint_angles, world_from_base), ignore_links
        )
        feet_result = self.feet_vs_xiaolan(joint_angles, world_from_base, ignore_links)
        if components_result.collision:
            return CollisionResult(True, components_result.hit, components_result.broad_phase_candidates + feet_result.broad_phase_candidates, components_result.narrow_phase_tests + feet_result.narrow_phase_tests)
        return CollisionResult(feet_result.collision, feet_result.hit, components_result.broad_phase_candidates + feet_result.broad_phase_candidates, components_result.narrow_phase_tests + feet_result.narrow_phase_tests)

    @staticmethod
    def _mechanically_adjacent(left, right):
        if left.leg_index is None and right.leg_index is not None:
            return right.mesh.link_name == "thigh"
        if right.leg_index is None and left.leg_index is not None:
            return left.mesh.link_name == "thigh"
        if left.leg_index != right.leg_index:
            return False
        names = {left.mesh.link_name, right.mesh.link_name}
        return len(names) == 1 or names in ({"thigh", "knee"}, {"knee", "ankle"})

    def _self_collision(self, joint_angles, world_from_base, vectorized_aabb, components=None, static_component_count=0, components_only=False):
        """检查机器人自身碰撞。

        参数:
        joint_angles: 当前关节角。
        world_from_base: 机身位姿。
            vectorized_aabb: 是否使用加速的 AABB 筛选。
            components: 已变换的网格部件。
        static_component_count: 静态部件数。
        components_only: 是否只检查网格部件。

        返回:
            自碰撞结果。
        """
        if components is None:
            components = self.robot_components(joint_angles, world_from_base)
        # 世界坐标包围盒只适用于当前位姿。
        triangle_bounds = {}

        def bounds_for(component_index):
            """按需计算一个部件中各三角形的世界坐标包围盒。

            参数:
                component_index: 部件索引。

            返回:
                每个三角形的最小和最大坐标。
            """
            if component_index not in triangle_bounds:
                triangle_bounds[component_index] = _triangle_bounds(components[component_index].triangles)
            return triangle_bounds[component_index]

        sweep_indices = {}

        def sweep_index(component_index, axis):
            """建立一个部件在指定轴上的扫描索引。

            参数:
                component_index: 部件索引。
                axis: 坐标轴索引。

            返回:
                排序后的最小坐标和原始索引。
            """
            key = (component_index, axis)
            if key not in sweep_indices:
                minimum = bounds_for(component_index)[0]
                original = np.arange(len(minimum), dtype=np.int64)
                order = np.lexsort((original, minimum[:, axis]))
                sweep_indices[key] = (minimum[order, axis], order)
            return sweep_indices[key]

        def query_sweep_candidates(query_min, query_max, indexed_component, axis):
            minimum, maximum = bounds_for(indexed_component)
            sorted_minimum, order = sweep_index(indexed_component, axis)
            limit = np.searchsorted(sorted_minimum, query_max[axis] + EPSILON_M, side="right")
            candidate = order[:limit]
            if len(candidate) == 0:
                return candidate
            # 扫描轴先筛掉大部分候选，再做完整 AABB 判断。
            return candidate[
                np.all(query_max + EPSILON_M >= minimum[candidate], axis=1)
                & np.all(query_min <= maximum[candidate] + EPSILON_M, axis=1)
            ]

        def best_sweep_axis(query_minimum, query_maximum, indexed_component):
            """选择候选最少的扫描轴。

            参数:
        query_minimum: 查询三角形的最小包围坐标。
        query_maximum: 查询三角形的最大包围坐标。
                indexed_component: 被查询的部件索引。

            返回:
                坐标轴索引。
            """
            if len(query_minimum) == 0:
                return 0
            sample = np.linspace(0, len(query_minimum) - 1, min(64, len(query_minimum)), dtype=np.int64)
            scores = []
            for axis in range(3):
                sorted_minimum, _ = sweep_index(indexed_component, axis)
                scores.append(sum(np.searchsorted(sorted_minimum, query_maximum[index, axis] + EPSILON_M, side="right") for index in sample))
            return min(range(3), key=lambda axis: (scores[axis], axis))
        broad, narrow = 0, 0
        for left_index, left in enumerate(components):
            for right_index, right in enumerate(components[left_index + 1:], start=left_index + 1):
                if left_index < static_component_count and right_index < static_component_count:
                    continue
                if self._mechanically_adjacent(left, right) or not aabb_overlaps(left.bounds_min, left.bounds_max, right.bounds_min, right.bounds_max):
                    continue
                broad += 1
                if vectorized_aabb:
                    # 只扫描三角形较少的一侧，再排序保持返回顺序稳定。
                    pairs = []
                    left_min, left_max = bounds_for(left_index)
                    right_min, right_max = bounds_for(right_index)
                    if len(left.triangles) <= len(right.triangles):
                        axis = best_sweep_axis(left_min, left_max, right_index)
                        for l_index in range(len(left.triangles)):
                            for r_index in query_sweep_candidates(left_min[l_index], left_max[l_index], right_index, axis):
                                pairs.append((l_index, int(r_index)))
                    else:
                        axis = best_sweep_axis(right_min, right_max, left_index)
                        for r_index in range(len(right.triangles)):
                            for l_index in query_sweep_candidates(right_min[r_index], right_max[r_index], left_index, axis):
                                pairs.append((int(l_index), r_index))
                    pairs.sort()
                    candidate_pairs = pairs
                else:
                    candidate_pairs = None
                if vectorized_aabb:
                    pair_iterator = candidate_pairs
                else:
                    pair_iterator = ((l_index, r_index) for l_index in range(len(left.triangles)) for r_index in range(len(right.triangles)))
                for l_index, r_index in pair_iterator:
                    left_tri, right_tri = left.triangles[l_index], right.triangles[r_index]
                    if not vectorized_aabb:
                        l_min, l_max = _triangle_bounds(left_tri[None, ...])
                        r_min, r_max = _triangle_bounds(right_tri[None, ...])
                        if not aabb_overlaps(l_min[0], l_max[0], r_min[0], r_max[0]):
                            continue
                    narrow += 1
                    if triangles_intersect(left_tri, right_tri):
                        return CollisionResult(True, CollisionHit(left.mesh.link_name, left.leg_index, left.component.component_index, int(left.component.triangle_indices[l_index]), right.mesh.link_name, right.leg_index, right.component.component_index, int(right.component.triangle_indices[r_index])), broad, narrow)
        if components_only:
            return CollisionResult(False, None, broad, narrow)
        for foot in self.foot_spheres(joint_angles, world_from_base):
            for component in components:
                # 脚球固定在本腿踝关节上，跳过这对固定相邻部件。
                if component.leg_index == foot.leg_index and component.mesh.link_name == "ankle":
                    continue
                if not aabb_overlaps(foot.center - foot.radius_m, foot.center + foot.radius_m, component.bounds_min, component.bounds_max):
                    continue
                broad += 1
                for local_index, triangle in enumerate(component.triangles):
                    minimum, maximum = _triangle_bounds(triangle[None, ...])
                    if not aabb_overlaps(foot.center - foot.radius_m, foot.center + foot.radius_m, minimum[0], maximum[0]):
                        continue
                    narrow += 1
                    if point_triangle_distance_squared(foot.center, triangle) <= (foot.radius_m + EPSILON_M) ** 2:
                        return CollisionResult(True, CollisionHit("foot", foot.leg_index, 0, -1, component.mesh.link_name, component.leg_index, component.component.component_index, int(component.component.triangle_indices[local_index])), broad, narrow)
        feet = self.foot_spheres(joint_angles, world_from_base)
        for left_index, left in enumerate(feet):
            for right in feet[left_index + 1:]:
                narrow += 1
                if np.linalg.norm(left.center - right.center) <= left.radius_m + right.radius_m + EPSILON_M:
                    return CollisionResult(True, CollisionHit("foot", left.leg_index, 0, -1, "foot", right.leg_index, 0, -1), broad, narrow)
        return CollisionResult(False, None, broad, narrow)

    def self_collision_reference(self, joint_angles, world_from_base):
        """用逐项循环检查自碰撞，供回归对比。

        参数:
            joint_angles: 六条腿的关节角度。
            world_from_base: `base_link` 到世界坐标的变换矩阵。

        返回:
            自碰撞结果。
        """
        return self._self_collision(joint_angles, world_from_base, False)

    def self_collision(self, joint_angles, world_from_base):
        """用加速筛选检查自碰撞。

        参数:
            joint_angles: 六条腿的关节角度。
            world_from_base: `base_link` 到世界坐标的变换矩阵。

        返回:
            自碰撞结果。
        """
        return self._self_collision(joint_angles, world_from_base, True)

    def cached_active_component_collision(self, static_components, active_components):
        """检查活动部件与静态部件的碰撞。

        参数:
            static_components: 已检查过的静态部件。
            active_components: 当前需要检查的活动部件。

        返回:
            碰撞结果。
        """
        combined = tuple(static_components) + tuple(active_components)
        return self._self_collision(None, None, True, components=combined, static_component_count=len(static_components), components_only=True)


def default_visual_scene(repository_root: Path | None = None, world_from_xiaolan=None) -> VisualCollisionScene:
    root = Path(repository_root) if repository_root else Path(__file__).resolve().parents[4]
    meshes = root / "src" / "grasp_hexapod_description" / "meshes"
    body = VisualMesh.load("body", meshes / "body_link.STL")
    links = {name: VisualMesh.load(name, meshes / (name + "_link.STL"), split_components=(name == "ankle")) for name in LINK_NAMES}
    xiaolan = VisualMesh.load("xiaolan", meshes / "xiaolan" / "base_link_xiaolan.STL")
    return VisualCollisionScene(body, links, xiaolan, world_from_xiaolan)
