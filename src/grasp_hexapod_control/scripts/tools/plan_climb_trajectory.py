"""Deterministic, offline visual-CAD corridor search for CC011.

This is deliberately a bounded model search.  It neither proves contact/load
nor authorizes hardware motion.  A trace is successful only when every listed
sample passes visual STL narrow-phase checks; otherwise it records BLOCKED.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

SCRIPTS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS_DIR))

from tools import analyze_climb as analyze
from utils import climb_collision as visual
from utils import climb
import kinematics as k
from utils import distance_to_polygon_boundary, points_in_polygon


ROOT = SCRIPTS_DIR.parents[2]
CONFIG_PATH = ROOT / "src/grasp_hexapod_control/config/climb.json"
MESH_PATH = ROOT / "src/grasp_hexapod_description/meshes/xiaolan/base_link_xiaolan.STL"
WORLD_FROM_XIAOLAN = np.array(
    [[1.0, 0.0, 0.0, 0.45], [0.0, 1.0, 0.0, -0.03], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
    dtype=np.float64,
)
P0_Z = 0.06988468052793656
LOW_DEPTH = -0.24
P0_X = -0.08
PREP_POSE_Z_PITCH = (0.15, -0.15)
BODY_POSE_X_Z_PITCH = (-0.05, 0.15, -0.05)
LEG_NAMES = k.LEG_NAMES
RM, RB, RF = 5, 3, 4


def pose_matrix(x, y, z, pitch):
    cosine, sine = np.cos(pitch), np.sin(pitch)
    output = np.eye(4, dtype=np.float64)
    output[:3, :3] = ((cosine, 0.0, sine), (0.0, 1.0, 0.0), (-sine, 0.0, cosine))
    output[:3, 3] = (x, y, z)
    return output


def smoothstep(value):
    value = float(value)
    return value**3 * (10.0 - 15.0 * value + 6.0 * value**2)


def interpolate_pose(start, end, count):
    for index in range(count):
        weight = smoothstep(index / (count - 1))
        yield pose_matrix(
            start[0] * (1.0 - weight) + end[0] * weight,
            start[1] * (1.0 - weight) + end[1] * weight,
            start[2] * (1.0 - weight) + end[2] * weight,
            start[3] * (1.0 - weight) + end[3] * weight,
        )


def world_points(transform, base_points):
    return base_points @ transform[:3, :3].T + transform[:3, 3]


def base_points(transform, world_points_value):
    return (world_points_value - transform[:3, 3]) @ transform[:3, :3]


def solve_anchors(model, anchors_world, world_from_base, seed, convergence_tolerance=1e-7):
    """从初始关节角求解固定足端位置的逆运动学。

    参数:
        model: 运动学模型。
        seed: 初始关节角。
        anchors_world: 足端世界坐标。
        world_from_base: 机身位姿。
        convergence_tolerance: 收敛误差上限。

    返回:
        求得的关节角和最大足端误差。
    """
    target_base = base_points(world_from_base, anchors_world)
    target_hip = model.base_to_hip(target_base)
    q = np.asarray(seed, dtype=np.float64).copy()
    for _ in range(180):
        # 雅可比在髋关节坐标中计算，误差也要使用同一坐标系。
        error = target_hip - model.forward(q)
        residual = float(np.max(np.linalg.norm(error, axis=1)))
        if residual <= convergence_tolerance:
            return q, residual
        delta = np.einsum("nij,nj->ni", model.damped_inverse_jacobian(q, damping=0.003), error)
        q = np.clip(q + np.clip(delta, -0.10, 0.10), k.JOINT_LOWER, k.JOINT_UPPER)
    residual = float(np.max(np.linalg.norm(target_hip - model.forward(q), axis=1)))
    return q, residual


def selector_geometry(config):
    normals, triangles = analyze.read_binary_stl(MESH_PATH)
    selector = config["known"][analyze.XIAOLAN_CAD_CONFIG_KEY]["negative_x"]
    extraction = analyze.extract_xiaolan_side(triangles, normals, selector)
    normal = extraction.plane_normal
    ids = np.flatnonzero(
        (np.linalg.norm(normals / np.linalg.norm(normals, axis=1)[:, None] - normal, axis=1) <= selector["normal_tolerance"])
        & (np.abs(triangles.mean(axis=1) @ normal - extraction.plane_offset_m) <= selector["plane_offset_tolerance_m"])
    )
    return triangles, extraction, normal, ids


def y_interval_at_x(outer, local_x):
    values = []
    for first, second in zip(outer, np.vstack((outer[1:], outer[:1]))):
        if min(first[0], second[0]) - 1e-12 <= local_x <= max(first[0], second[0]) + 1e-12 and abs(second[0] - first[0]) > 1e-12:
            fraction = (local_x - first[0]) / (second[0] - first[0])
            if -1e-12 <= fraction <= 1.0 + 1e-12:
                values.append(float(first[1] + fraction * (second[1] - first[1])))
    values = sorted(set(round(value, 12) for value in values))
    if len(values) != 2:
        raise ValueError("low-surface x section has no unique outer y interval")
    return values[0], values[1]


def low_goals(extraction, normal):
    """生成三只脚落在低斜面上的目标点。

    参数:
        extraction: 小蓝斜面提取结果。
        normal: 低斜面法向量。

    返回:
        足端接触点、球心目标点和边界距离。

    每只脚按自己的 y 坐标计算斜面高度，避免法向量带 y 分量时偏离斜面。
    """
    row = float(k.GraspKinematic().forward_base(k.Q_STAND)[RF, 1])
    minimum, maximum = y_interval_at_x(extraction.outer_loop_vertices_3d_m, LOW_DEPTH)
    center = 0.5 * (minimum + maximum)
    if center - row <= minimum or center + row >= maximum:
        raise ValueError("fixed low-surface depth does not admit rb/rf row spacing")
    contact_y = np.array((center - row, center + row, center), dtype=np.float64)
    contact = np.column_stack((np.full(3, LOW_DEPTH), contact_y, (extraction.plane_offset_m - normal[0] * LOW_DEPTH - normal[1] * contact_y) / normal[2]))
    outer_xy = extraction.outer_loop_vertices_3d_m[:, :2]
    if not points_in_polygon(contact[:, :2], outer_xy).all() or not np.all((contact_y >= minimum) & (contact_y <= maximum)):
        raise ValueError("fixed low-surface contacts lie outside its usable polygon")
    boundary_distances = distance_to_polygon_boundary(contact[:, :2], outer_xy)
    if np.min(boundary_distances) <= k.FOOT_RADIUS:
        raise ValueError("fixed low-surface contacts lack the model-only boundary margin")
    local = contact + k.FOOT_RADIUS * normal
    world = world_points(WORLD_FROM_XIAOLAN, local)
    return {
        "depth_local_x_m": LOW_DEPTH,
        "robot_center_y_local_m": center,
        "outer_y_interval_m": [minimum, maximum],
        "row_boundary_margin_m": min(center - row - minimum, maximum - (center + row)),
        "contact_outer_boundary_distances_m": boundary_distances,
        "minimum_contact_outer_boundary_distance_m": float(np.min(boundary_distances)),
        "boundary_margin_is_model_only": True,
        "contact_points_local_m": contact,
        "support_plane_normal_local": normal,
        "goals_local_m": local,
        "goals_world_m": world,
        "outer_polygon_and_x_section_checked": True,
    }


def visual_p0(scene, q, world_from_base):
    self_result = scene.self_collision(q, world_from_base)
    xiaolan_result = scene.robot_vs_xiaolan(q, world_from_base)
    return (not self_result.collision and not xiaolan_result.collision), self_result, xiaolan_result


def json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {key: json_value(child) for key, child in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_value(child) for child in value]
    return value


def hit_value(result):
    if result.hit is None:
        return None
    return {field: getattr(result.hit, field) for field in result.hit.__dataclass_fields__}


def _new_worst(include_support=False):
    result = {
        "max_ik_residual_m": 0.0,
        "min_joint_margin_rad": float("inf"),
        "min_sigma": float("inf"),
        "max_adjacent_joint_delta_rad": 0.0,
        "max_adjacent_base_translation_step_m": 0.0,
        "max_adjacent_pitch_step_rad": 0.0,
    }
    if include_support:
        result["support_raw_margin_m"] = float("inf")
    return result


def _record_sample_worst(worst, model, q, residual, previous_q, previous_transform, transform):
    worst["max_ik_residual_m"] = max(worst["max_ik_residual_m"], float(residual))
    worst["min_joint_margin_rad"] = min(worst["min_joint_margin_rad"], float(np.min(model.joint_limit_margins(q))))
    worst["min_sigma"] = min(worst["min_sigma"], float(np.min(model.jacobian_min_singular_values(q))))
    if previous_q is not None:
        worst["max_adjacent_joint_delta_rad"] = max(worst["max_adjacent_joint_delta_rad"], float(np.max(np.abs(q - previous_q))))
    if previous_transform is not None:
        worst["max_adjacent_base_translation_step_m"] = max(worst["max_adjacent_base_translation_step_m"], float(np.linalg.norm(transform[:3, 3] - previous_transform[:3, 3])))
        previous_pitch = float(np.arctan2(previous_transform[0, 2], previous_transform[0, 0]))
        pitch = float(np.arctan2(transform[0, 2], transform[0, 0]))
        worst["max_adjacent_pitch_step_rad"] = max(worst["max_adjacent_pitch_step_rad"], abs(pitch - previous_pitch))


def adaptive_ik_curve(model, seed_q, coarse_times, pose_at, anchors_at, active_legs, cap=501):
    """自适应细分轨迹，并求解每个采样点的逆运动学。

    参数:
        model: 运动学模型。
        seed_q: 初始关节角。
        coarse_times: 初始采样时刻。
        pose_at: 按时刻返回机身位姿的函数。
        anchors_at: 按时刻返回足端目标的函数。
        active_legs: 运动中的腿。
        cap: 最大采样数。

    返回:
        按时间排序的采样点。
    """
    samples = []

    def refine(left_t, left_q, right_t):
        transform = pose_at(right_t)
        anchors = anchors_at(right_t)
        final = right_t == coarse_times[-1]
        right_q, residual = solve_anchors(model, anchors, transform, left_q, 1e-10 if final else 1e-7)
        left_transform, left_anchors = pose_at(left_t), anchors_at(left_t)
        joint_step = float(np.max(np.abs(right_q - left_q)))
        foot_step = float(np.max(np.linalg.norm(anchors[list(active_legs)] - left_anchors[list(active_legs)], axis=1))) if active_legs else 0.0
        base_step = float(np.linalg.norm(transform[:3, 3] - left_transform[:3, 3]))
        pitch_step = abs(float(np.arctan2(transform[0, 2], transform[0, 0]) - np.arctan2(left_transform[0, 2], left_transform[0, 0])))
        if joint_step > .05 or foot_step > .003 or base_step > .003 or pitch_step > .02:
            middle = .5 * (left_t + right_t)
            middle_q = refine(left_t, left_q, middle)
            return refine(middle, middle_q, right_t)
        if len(samples) >= cap:
            raise ValueError("adaptive sample cap exceeded")
        samples.append({"t": right_t, "q_rad": right_q, "transform": transform, "anchors": anchors, "residual_m": residual})
        return right_q

    initial_t = float(coarse_times[0])
    samples.append({"t": initial_t, "q_rad": np.asarray(seed_q).copy(), "transform": pose_at(initial_t), "anchors": anchors_at(initial_t), "residual_m": 0.0})
    q = np.asarray(seed_q).copy()
    for right_t in coarse_times[1:]:
        q = refine(initial_t, q, float(right_t))
        initial_t = float(right_t)
    return samples


def evaluate_prep(model, p0_pose, anchors, candidate):
    witness = {"candidate": list(candidate)}
    def pose_at(time_value):
        return next(interpolate_pose(p0_pose, candidate, 2)) if time_value == 0.0 else pose_matrix(*(np.asarray(p0_pose) * (1.0 - smoothstep(time_value)) + np.asarray(candidate) * smoothstep(time_value)))
    try:
        samples = adaptive_ik_curve(model, k.Q_STAND, np.linspace(0.0, 1.0, 31), pose_at, lambda _: anchors.copy(), (), MAX_ADAPTIVE_SAMPLES)
    except ValueError as exc:
        return None, {"reason": "adaptive_sample_cap", "detail": str(exc)}
    previous_transform = None
    worst = _new_worst()
    previous_q = None
    for sample_index, sample in enumerate(samples):
        transform, q, residual = sample["transform"], sample["q_rad"], sample["residual_m"]
        if residual > 1e-6:
            witness.update({"sample": sample_index, "residual_m": residual})
            return None, witness
        target_base = base_points(transform, anchors)
        # 该边界只适用于平地步态，不能拿来限制非共面攀爬姿态。
        margins = model.joint_limit_margins(q)
        if np.min(margins) < 0.0:
            witness.update({"sample": sample_index, "joint_margin_rad": float(np.min(margins))})
            return None, witness
        _record_sample_worst(worst, model, q, residual, previous_q, previous_transform, transform)
        sample["index"] = sample_index
        previous_q = q
        previous_transform = transform
    worst["ground_gait_workspace_evaluated"] = False
    return {"pose": candidate, "q": samples[-1]["q_rad"], "samples": samples, "worst": worst}, witness


def certify_visual_prep(scene, evaluation, p0_pose):
    """检查 PREP 阶段所有采样点的可视网格碰撞。

    参数:
        scene: 可视碰撞场景。
        evaluation: PREP 阶段的逆运动学采样结果。
        p0_pose: 起始机身位姿。

    返回:
        是否通过，以及首个碰撞信息。
    """
    for sample in evaluation["samples"]:
        transform, q = sample["transform"], np.asarray(sample["q_rad"], dtype=np.float64)
        self_result = scene.self_collision(q, transform)
        object_result = scene.robot_vs_xiaolan(q, transform)
        if self_result.collision or object_result.collision:
            return False, {"sample": sample["index"], "self_hit": hit_value(self_result), "xiaolan_hit": hit_value(object_result)}
    return True, {"visual_samples": len(evaluation["samples"])}


def _contact_state(model, q, transform, leg, extraction, normal_local):
    centers_world = world_points(transform, model.forward_base(q))
    centers_local = base_points(WORLD_FROM_XIAOLAN, centers_world)
    contact = centers_local[leg] - k.FOOT_RADIUS * normal_local
    signed_error = float(np.dot(contact, normal_local) - extraction.plane_offset_m)
    return {
        "signed_plane_error_m": signed_error,
        "plane_error_m": abs(signed_error),
        "polygon_inside": bool(points_in_polygon(contact[None, :2], extraction.outer_loop_vertices_3d_m[:, :2])[0]),
        "contact_local_m": contact.tolist(),
    }


def _goal_contact_witness(scene, model, q, transform, legs, extraction, normal_local):
    output = {}
    for leg in legs:
        state = _contact_state(model, q, transform, leg, extraction, normal_local)
        state["raw_hit_ids_diagnostic"] = list(scene.foot_xiaolan_triangle_hits(leg, q, transform))
        if state["plane_error_m"] > 1e-8 or not state["polygon_inside"]:
            return False, {"leg": LEG_NAMES[leg], **state}
        output[LEG_NAMES[leg]] = state
    return True, output


def _foot_contact_policy(scene, model, q, transform, descent_legs, static_contact_legs, extraction, normal_local):
    """检查脚与低斜面的模型接触是否符合当前阶段。

    参数:
        scene: 当前碰撞场景。
        model: 运动学模型。
        q: 当前关节角。
        transform: 机身位姿。
        descent_legs: 正在落脚的腿。
        static_contact_legs: 保持接触的腿。
        extraction: 小蓝低斜面信息。
        normal_local: 低斜面法线。

    返回:
        是否符合要求，以及不符合时的原因。

    这里只判断模型中的斜面位置，不代表真实接触。
    """
    allowed = set(descent_legs) | set(static_contact_legs)
    for leg in range(6):
        hits = tuple(scene.foot_xiaolan_triangle_hits(leg, q, transform))
        if not hits:
            continue
        state = _contact_state(model, q, transform, leg, extraction, normal_local)
        state["raw_hit_ids_diagnostic"] = list(hits)
        if leg not in allowed or -1 in hits or state["plane_error_m"] > 1e-6 or not state["polygon_inside"]:
            return False, {"leg": LEG_NAMES[leg], **state}
    return True, {}


def evaluate_rm_template(scene, model, prep_q, prep_pose, anchors, goal_world, normal_world, extraction, normal_local, lift, retreat_x, x_offset, approach):
    """检查 RM 抬脚、前移和落脚模板。

    参数:
        scene: 当前碰撞场景。
        model: 运动学模型。
        prep_q: PREP 关节角。
        prep_pose: PREP 机身位姿。
        anchors: 固定足端位置。
        goal_world: RM 目标点。
        normal_world: 低斜面世界法线。
        extraction: 小蓝低斜面信息。
        normal_local: 低斜面局部法线。
        lift: 抬脚高度。
        retreat_x: 后退距离。
        x_offset: 横向偏移。
        approach: 落脚前的接近距离。

    返回:
        是否通过，以及采样检查结果。
    """
    start = anchors[RM].copy()
    waypoints = [start, start + np.array((retreat_x, 0.0, lift)), goal_world + np.array((x_offset, 0.0, lift)), goal_world + normal_world * approach, goal_world]
    def anchors_at(time_value):
        segment = min(int(time_value), 3)
        value = time_value - segment if time_value < 4.0 else 1.0
        target = anchors.copy()
        target[RM] = waypoints[segment] * (1.0 - smoothstep(value)) + waypoints[segment + 1] * smoothstep(value)
        return target
    try:
        samples = adaptive_ik_curve(model, prep_q, np.linspace(0.0, 4.0, 41), lambda _: prep_pose, anchors_at, (RM,), MAX_ADAPTIVE_SAMPLES)
    except ValueError as exc:
        return False, {"reason": "adaptive_sample_cap", "detail": str(exc), "sample_cap": MAX_ADAPTIVE_SAMPLES}
    previous_q, previous_transform = None, None
    worst = _new_worst()
    cache = build_phase_cache(scene, prep_q, prep_pose, (RM,))
    for sample_index, sample in enumerate(samples):
            segment = min(int(sample["t"] - 1e-12), 3)
            q, residual = sample["q_rad"], sample["residual_m"]
            if residual > (1e-10 if sample_index == len(samples) - 1 else 1e-6) or np.min(model.joint_limit_margins(q)) < 0.0:
                return False, {"sample": sample_index, "reason": "ik_limit_or_analytic_self", "residual_m": residual}
            visual_ok, visual_witness = cached_phase_visual(scene, cache, q, prep_pose)
            if not visual_ok:
                return False, {"sample": sample_index, "reason": "cached_visual", **visual_witness}
            policy_ok, policy_witness = _foot_contact_policy(
                scene, model, q, prep_pose, (RM,) if segment == 3 else (), (), extraction, normal_local
            )
            if not policy_ok:
                return False, {"sample": sample_index, "reason": "foot_plane_polygon_policy", "segment": segment, **policy_witness}
            _record_sample_worst(worst, model, q, residual, previous_q, previous_transform, prep_pose)
            previous_q, previous_transform = q.copy(), prep_pose
    contact_ok, contact = _goal_contact_witness(scene, model, q, prep_pose, (RM,), extraction, normal_local)
    if not contact_ok:
        return False, {"sample": sample_index - 1, "reason": "rm_goal_low_plane_contact", **contact}
    return True, {"visual_sample_count": len(samples), "q_rad": q.tolist(), "waypoints_world_m": [point.tolist() for point in waypoints], "goal_contact": contact, "final_fk_goal_residual_m": residual, "worst": worst}


def visual_fixed_feet(scene, model, q, transform, extraction, normal_local, allowed_legs):
    self_result = scene.self_collision(q, transform)
    component_result = scene.components_vs_xiaolan(scene.robot_components(q, transform))
    if self_result.collision or component_result.collision:
        return False, {"self_hit": hit_value(self_result), "xiaolan_hit": hit_value(component_result)}
    return _foot_contact_policy(scene, model, q, transform, (), allowed_legs, extraction, normal_local)


def _first_foot_pair_collision(feet):
    for index, left in enumerate(feet):
        for right in feet[index + 1:]:
            if np.linalg.norm(left.center - right.center) <= left.radius_m + right.radius_m + visual.EPSILON_M:
                return [LEG_NAMES[left.leg_index], LEG_NAMES[right.leg_index]]
    return None


def build_phase_cache(
    scene,
    q,
    transform,
    active_legs,
    xiaolan_component_exclusions=(),
):
    xiaolan_component_exclusions = tuple(xiaolan_component_exclusions)
    static_legs = tuple(leg for leg in range(6) if leg not in active_legs)
    static = scene.robot_components(q, transform, static_legs, include_body=True)
    static_xiaolan = scene.components_vs_xiaolan(
        static,
        exclude_components=xiaolan_component_exclusions,
    )
    static_self = scene.cached_active_component_collision((), static)
    static_feet = tuple(foot for foot in scene.foot_spheres(q, transform) if foot.leg_index not in active_legs)
    static_foot_components = scene.feet_vs_components(static_feet, static)
    static_foot_pair = _first_foot_pair_collision(static_feet)
    return {"static_components": static, "static_feet": static_feet, "static_xiaolan": static_xiaolan, "static_self": static_self, "static_foot_components": static_foot_components, "static_foot_pair": static_foot_pair, "active_legs": tuple(active_legs), "xiaolan_component_exclusions": xiaolan_component_exclusions}


def cached_phase_visual(
    scene,
    cache,
    q,
    transform,
    xiaolan_component_exclusions=None,
):
    recorded_exclusions = tuple(
        cache.get("xiaolan_component_exclusions", ())
    )
    if xiaolan_component_exclusions is None:
        xiaolan_component_exclusions = recorded_exclusions
    else:
        xiaolan_component_exclusions = tuple(
            xiaolan_component_exclusions
        )
        if xiaolan_component_exclusions != recorded_exclusions:
            raise ValueError(
                "cached Xiaolan component exclusions differ from cache build"
            )
    if cache["static_xiaolan"].collision:
        return False, {"static_xiaolan_hit": hit_value(cache["static_xiaolan"])}
    if cache["static_self"].collision:
        return False, {"static_self_hit": hit_value(cache["static_self"])}
    if cache["static_foot_components"].collision:
        return False, {"static_foot_component_hit": hit_value(cache["static_foot_components"])}
    if cache["static_foot_pair"] is not None:
        return False, {"static_foot_pair_collision": cache["static_foot_pair"]}
    active = scene.robot_components(q, transform, cache["active_legs"], include_body=False)
    feet = tuple(scene.foot_spheres(q, transform)[leg] for leg in cache["active_legs"])
    checks = (
        scene.components_vs_xiaolan(
            active,
            exclude_components=xiaolan_component_exclusions,
        ),
        scene.cached_active_component_collision(cache["static_components"], active),
        scene.feet_vs_components(feet, cache["static_components"] + active),
        scene.feet_vs_components(cache["static_feet"], active),
    )
    for result in checks:
        if result.collision:
            return False, {"visual_hit": hit_value(result)}
    active_pair = _first_foot_pair_collision(feet)
    if active_pair is not None:
        return False, {"foot_pair_collision": active_pair}
    for left in feet:
        for right in cache["static_feet"]:
            if np.linalg.norm(left.center - right.center) <= left.radius_m + right.radius_m + visual.EPSILON_M:
                return False, {"foot_static_collision": [LEG_NAMES[left.leg_index], LEG_NAMES[right.leg_index]]}
    return True, {}


def evaluate_body(model, scene, start_q, start_pose, anchors, candidate, extraction, normal_local):
    def pose_at(time_value):
        return pose_matrix(*(np.asarray(start_pose) * (1.0 - smoothstep(time_value)) + np.asarray(candidate) * smoothstep(time_value)))
    try:
        samples = adaptive_ik_curve(model, start_q, np.linspace(0.0, 1.0, 31), pose_at, lambda _: anchors.copy(), (), MAX_ADAPTIVE_SAMPLES)
    except ValueError as exc:
        return None, {"reason": "adaptive_sample_cap", "detail": str(exc), "sample_cap": MAX_ADAPTIVE_SAMPLES}
    q = np.asarray(start_q).copy()
    previous_q = None
    previous_transform = None
    worst = _new_worst(include_support=True)
    for index, sample in enumerate(samples):
        transform, q, residual = sample["transform"], sample["q_rad"], sample["residual_m"]
        if residual > 1e-6 or np.min(model.joint_limit_margins(q)) < 0.0:
            return None, {"sample": index, "reason": "ik_limit_or_analytic_self", "residual_m": residual}
        visual_ok, visual_witness = visual_fixed_feet(scene, model, q, transform, extraction, normal_local, {RM})
        if not visual_ok:
            return None, {"sample": index, "reason": "visual", **visual_witness}
        world_com = world_points(transform, model.center_of_mass_base(q)[None, :])[0]
        support = climb.gravity_projected_support(world_com, anchors[[0, 1, 2, 5]], np.array((0.0, 0.0, -9.81)))
        _record_sample_worst(worst, model, q, residual, previous_q, previous_transform, transform)
        worst["support_raw_margin_m"] = min(worst["support_raw_margin_m"], float(support.raw_margin_m))
        sample["index"] = index
        previous_q = q.copy()
        previous_transform = transform
    return {"q": q, "candidate": candidate, "samples": samples, "worst": worst}, {"visual_sample_count": len(samples)}


MAX_ADAPTIVE_SAMPLES = 501


def adaptive_pair_samples(model, body_q, body_pose, anchors, goals, normal_world, lift, approach, x_offset, retreat_x=0.0):
    """生成 RB 和 RF 同步运动的逆运动学采样点。

    参数:
        model: 运动学模型。
        body_q: 当前关节角。
        body_pose: 当前机身位姿。
        anchors: 当前足端位置。
        goals: RB、RF 目标点。
        normal_world: 低斜面的世界坐标法向量。
        lift: 抬脚高度。
        approach: 落脚前的接近距离。
        x_offset: 横向偏移。
        retreat_x: 后退距离。

    返回:
        采样点和路径关键点。
    """
    starts = anchors[[RB, RF]].copy()
    targets = np.asarray(goals, dtype=np.float64)
    paths = [starts, starts + np.array((retreat_x, 0.0, lift)), targets + normal_world * approach + np.array((x_offset, 0.0, 0.0)), targets]
    def target_at(segment, value):
        weight = smoothstep(value)
        target = anchors.copy()
        target[[RB, RF]] = paths[segment] * (1.0 - weight) + paths[segment + 1] * weight
        return target

    def anchors_at(time_value):
        segment = min(int(time_value), 2)
        return target_at(segment, time_value - segment if time_value < 3.0 else 1.0)

    samples = adaptive_ik_curve(model, body_q, np.linspace(0.0, 3.0, 43), lambda _: body_pose, anchors_at, (RB, RF), MAX_ADAPTIVE_SAMPLES)
    for sample in samples:
        sample["segment"] = min(int(sample["t"] - 1e-12), 2)
    return samples, paths


def evaluate_pair_template(scene, model, body_q, body_pose, anchors, goals, normal_world, extraction, normal_local, lift, retreat_x, x_offset, approach):
    """检查 RB 和 RF 同步摆动模板。

    参数:
        scene: 当前碰撞场景。
        model: 运动学模型。
        body_q: 当前关节角。
        body_pose: 当前机身位姿。
        anchors: 当前足端位置。
        goals: RB、RF 目标点。
        normal_world: 低斜面世界法线。
        extraction: 小蓝低斜面信息。
        normal_local: 低斜面局部法线。
        lift: 抬脚高度。
        retreat_x: 后退距离。
        x_offset: 横向偏移。
        approach: 落脚前的接近距离。

    返回:
        是否通过，以及采样检查结果。
    """
    try:
        samples, paths = adaptive_pair_samples(model, body_q, body_pose, anchors, goals, normal_world, lift, approach, x_offset, retreat_x)
    except ValueError as exc:
        return False, {"reason": "adaptive_sample_cap", "detail": str(exc), "sample_cap": MAX_ADAPTIVE_SAMPLES}
    q = np.asarray(body_q, dtype=np.float64).copy()
    worst = _new_worst()
    cache = build_phase_cache(scene, body_q, body_pose, (RB, RF))
    for sample_index, sample in enumerate(samples):
            segment, previous = sample["segment"], q.copy()
            q, residual = sample["q_rad"], sample["residual_m"]
            if residual > (1e-10 if sample_index == len(samples) - 1 else 1e-6) or np.min(model.joint_limit_margins(q)) < 0.0:
                return False, {"sample": sample_index, "reason": "ik_limit_or_analytic_self", "residual_m": residual}
            visual_ok, visual_witness = cached_phase_visual(scene, cache, q, body_pose)
            if not visual_ok:
                return False, {"sample": sample_index, "reason": "visual", **visual_witness}
            policy_ok, policy_witness = _foot_contact_policy(
                scene, model, q, body_pose, (RB, RF) if segment == 2 else (), (RM,), extraction, normal_local
            )
            if not policy_ok:
                return False, {"sample": sample_index, "reason": "foot_plane_polygon_policy", "segment": segment, **policy_witness}
            _record_sample_worst(worst, model, q, residual, previous, body_pose, body_pose)
    contact_ok, contact = _goal_contact_witness(scene, model, q, body_pose, (RB, RF), extraction, normal_local)
    if not contact_ok:
        return False, {"sample": sample_index - 1, "reason": "pair_goal_low_plane_contact", **contact}
    return True, {"visual_sample_count": len(samples), "q_rad": q.tolist(), "worst": worst, "goal_contact": contact, "final_fk_goal_residual_m": residual, "synchronized_common_time": True, "waypoints_world_m": [item.tolist() for item in paths]}


RM_TEMPLATES = ((0.05, 0.0, -0.02, 0.03), (0.05, -0.01, -0.02, 0.03), (0.06, 0.0, -0.02, 0.03), (0.06, -0.01, -0.02, 0.03))
PAIR_TEMPLATES = ((0.05, 0.0, 0.0, 0.05), (0.05, -0.01, 0.0, 0.05), (0.06, 0.0, 0.0, 0.05), (0.06, -0.01, 0.0, 0.05))


def _node(pose, q, anchors):
    return {
        "base_world": list(pose[:3]),
        "pitch_rad": float(pose[3]),
        "q_rad": np.asarray(q, dtype=np.float64).tolist(),
        "anchors_world_m": np.asarray(anchors, dtype=np.float64).tolist(),
    }


def terminal_axis_angles_deg(model, q, transform, normals_world, legs):
    axes_world = model.terminal_axes_base(q) @ transform[:3, :3].T
    normals_world = np.asarray(normals_world, dtype=np.float64)
    values = {}
    for leg, normal_world in zip(legs, normals_world):
        cosine = np.clip(np.dot(-axes_world[leg], normal_world) / np.linalg.norm(normal_world), -1.0, 1.0)
        values[LEG_NAMES[leg]] = float(np.degrees(np.arccos(cosine)))
    return values


def _stage(name, required, accepted, candidates, configured_candidate_count=1, **extra):
    return {
        "name": name,
        "samples_required": required,
        "accepted_count": int(accepted),
        "candidate_count": len(candidates),
        "configured_candidate_count": int(configured_candidate_count),
        "candidates": candidates,
        **extra,
    }


def build_trace(stop_after=None):
    started = time.monotonic()
    config = json.loads(CONFIG_PATH.read_text())
    _, extraction, normal, _ = selector_geometry(config)
    goal = low_goals(extraction, normal)
    support_normal_world = normal @ WORLD_FROM_XIAOLAN[:3, :3].T
    scene = visual.default_visual_scene(ROOT, WORLD_FROM_XIAOLAN)
    model = k.GraspKinematic()
    y = goal["robot_center_y_local_m"] - 0.03
    p0_pose = (P0_X, y, P0_Z, 0.0)
    p0_transform = pose_matrix(*p0_pose)
    p0_clear, p0_self, p0_xiaolan = visual_p0(scene, k.Q_STAND, p0_transform)
    p0_preflight = {"base_pose_world": list(p0_pose), "clear": p0_clear, "self_hit": hit_value(p0_self), "xiaolan_hit": hit_value(p0_xiaolan), "visual_narrow_tests": p0_self.narrow_phase_tests + p0_xiaolan.narrow_phase_tests}
    prep_pose = (-0.06, y, *PREP_POSE_Z_PITCH)
    body_pose = (BODY_POSE_X_Z_PITCH[0], y, BODY_POSE_X_Z_PITCH[1], BODY_POSE_X_Z_PITCH[2])
    base = {
        "schema": "SIMULATION_ONLY_VISUAL_MODEL_TRACE",
        "status": "BLOCKED",
        "leg_order": list(LEG_NAMES),
        "world_from_xiaolan": WORLD_FROM_XIAOLAN.tolist(),
        "URDF_COLLISION_GEOMETRY_NOT_USED": True,
        "GROUND_GAIT_WORKSPACE_NOT_USED_AS_CLIMB_GATE": True,
        "REAL_CLEARANCE_UNRESOLVED": True,
        "limitations": [
            "visual CAD is a model, not measured clearance",
            "visual zero intersection does not certify positive clearance",
            "geometry-only support is not contact/load/friction proof",
            "bottom camera, real motor envelopes, and real foot-geometry tolerances are not modelled",
        ],
        "stl_sha256": {"xiaolan": hashlib.sha256(MESH_PATH.read_bytes()).hexdigest(), "body": scene.body_mesh.sha256, **{name: mesh.sha256 for name, mesh in scene.link_meshes.items()}},
        "low_surface": {"source": "CAD fitted low-platform plane and usable 2D polygon", "normal_local": normal.tolist(), "plane_offset_m": float(extraction.plane_offset_m), "selected": goal},
        "p0_preflight": p0_preflight,
        "fixed_poses_world": {"P0": list(p0_pose), "PREP": list(prep_pose), "BODY": list(body_pose)},
        "prep_anchor_policy": "all six P0 football centers fixed in world for every PREP sample",
        "verification_leaves_are_not_optimized_poses": True,
        "nodes": {"P0": None, "PREP": None, "RM": None, "BODY": None, "PAIR": None},
        "goals_world_m": {"rb": goal["goals_world_m"][0].tolist(), "rf": goal["goals_world_m"][1].tolist(), "rm": goal["goals_world_m"][2].tolist()},
        "stages": [],
        "stage_requirements": [
            {"name": "P0_TO_PREP", "minimum_samples": 31, "configured_candidate_count": 1, "active_legs": [], "visual_components_and_self_required": True},
            {"name": "RM_SWING", "minimum_samples": 41, "configured_candidate_count": 4, "active_legs": ["rm"], "waypoint_templates": 4, "visual_components_and_self_required": True},
            {"name": "BODY_TRANSFER", "minimum_samples": 31, "configured_candidate_count": 1, "active_legs": [], "visual_components_and_self_required": True},
            {"name": "PAIR_SWING", "minimum_samples": 41, "configured_candidate_count": 4, "active_legs": ["rb", "rf"], "synchronized_common_time": True, "waypoint_templates": 4, "visual_components_and_self_required": True},
        ],
        "planner_elapsed_s": 0.0,
        "stop_after": stop_after,
    }
    if not p0_clear:
        base["blocked"] = {"stage": "P0", "reason": "fixed P0 visual preflight failed", "earliest_witness": p0_preflight}
        base["planner_elapsed_s"] = time.monotonic() - started
        return json_value(base)
    anchors = world_points(p0_transform, model.forward_base(k.Q_STAND))
    prep, prep_witness = evaluate_prep(model, p0_pose, anchors, prep_pose)
    prep_record = {"fixed_pose_world": list(prep_pose), "analytic_clear": prep is not None, "witness": prep_witness}
    if prep is not None:
        prep.update({"p0_pose": p0_pose, "anchors": anchors})
        prep_clear, prep_visual = certify_visual_prep(scene, prep, p0_pose)
        prep_record.update({"visual_clear": prep_clear, "visual_witness": prep_visual, "worst": prep["worst"]})
    base["stages"].append(_stage("P0_TO_PREP", 31, int(prep is not None and prep_record.get("visual_clear", False)), [prep_record], visual_components_and_self_required=True, visual_sample_count=0 if prep is None else len(prep["samples"])))
    if prep is None or not prep_record.get("visual_clear", False):
        base["blocked"] = {"stage": "PREP_VISUAL_SWEEP", "reason": "fixed PREP trajectory failed", "earliest_witness": prep_record}
        base["planner_elapsed_s"] = time.monotonic() - started
        return json_value(base)

    # 单阶段完成只表示该阶段已检查，不表示整条路径成功。
    if stop_after == "PREP":
        base["nodes"]["P0"] = _node(p0_pose, k.Q_STAND, anchors)
        base["nodes"]["PREP"] = _node(prep_pose, prep["q"], anchors)
        base["status"] = "PARTIAL_STAGE_COMPLETE"
        base["result"] = "PREP complete; later phases intentionally not evaluated"
        base["planner_elapsed_s"] = time.monotonic() - started
        return json_value(base)

    prep_transform = pose_matrix(*prep_pose)
    rm_records = []
    rm_witness = None
    for lift, retreat_x, x_offset, approach in RM_TEMPLATES:
        clear, witness = evaluate_rm_template(scene, model, prep["q"], prep_transform, anchors, goal["goals_world_m"][2], support_normal_world, extraction, normal, lift, retreat_x, x_offset, approach)
        record = {"lift_m": lift, "lift_stage_retreat_x_m": retreat_x, "above_goal_x_offset_m": x_offset, "normal_approach_m": approach, "clear": clear, "witness": witness}
        rm_records.append(record)
        if clear:
            rm_witness = witness
            break
    base["stages"].append(_stage("RM_SWING", 41, int(rm_witness is not None), rm_records, configured_candidate_count=4, visual_components_and_self_required=True, visual_sample_count=0 if rm_witness is None else rm_witness["visual_sample_count"]))
    if rm_witness is None:
        base["blocked"] = {"stage": "RM_SWING", "reason": "all four fixed RM trajectory templates failed", "earliest_witness": rm_records[0]}
        base["planner_elapsed_s"] = time.monotonic() - started
        return json_value(base)
    rm_anchors = anchors.copy(); rm_anchors[RM] = goal["goals_world_m"][2]
    rm_q = np.asarray(rm_witness["q_rad"], dtype=np.float64)
    base["nodes"]["P0"] = _node(p0_pose, k.Q_STAND, anchors)
    base["nodes"]["PREP"] = _node(prep_pose, prep["q"], anchors)
    base["nodes"]["RM"] = {**_node(prep_pose, rm_q, rm_anchors), "goal_contact": rm_witness["goal_contact"], "terminal_axis_angles_deg": terminal_axis_angles_deg(model, rm_q, prep_transform, (support_normal_world,), (RM,))}
    if stop_after == "RM":
        base["status"] = "PARTIAL_STAGE_COMPLETE"; base["result"] = "RM complete; later phases intentionally not evaluated"; base["planner_elapsed_s"] = time.monotonic() - started
        return json_value(base)
    body_value, body_witness = evaluate_body(model, scene, rm_q, prep_pose, rm_anchors, body_pose, extraction, normal)
    body_records = [{"fixed_pose_world": list(body_pose), "clear": body_value is not None, "witness": body_witness, "worst": None if body_value is None else body_value["worst"]}]
    base["stages"].append(_stage("BODY_TRANSFER", 31, int(body_value is not None), body_records, visual_components_and_self_required=True, visual_sample_count=0 if body_value is None else len(body_value["samples"])))
    if body_value is None:
        base["blocked"] = {"stage": "BODY_TRANSFER", "reason": "fixed BODY trajectory failed", "earliest_witness": body_records[0]}; base["planner_elapsed_s"] = time.monotonic() - started
        return json_value(base)
    body_transform = pose_matrix(*body_pose)
    base["nodes"]["BODY"] = {**_node(body_pose, body_value["q"], rm_anchors), "terminal_axis_angles_deg": terminal_axis_angles_deg(model, body_value["q"], body_transform, (support_normal_world,), (RM,))}
    if stop_after == "BODY":
        base["status"] = "PARTIAL_STAGE_COMPLETE"; base["result"] = "BODY complete; pair phase intentionally not evaluated"; base["planner_elapsed_s"] = time.monotonic() - started
        return json_value(base)
    pair_records = []
    pair_witness = None
    for lift, retreat_x, x_offset, approach in PAIR_TEMPLATES:
        clear, witness = evaluate_pair_template(scene, model, body_value["q"], body_transform, rm_anchors, goal["goals_world_m"][:2], support_normal_world, extraction, normal, lift, retreat_x, x_offset, approach)
        record = {"lift_m": lift, "lift_stage_retreat_x_m": retreat_x, "above_goal_x_offset_m": x_offset, "normal_approach_m": approach, "clear": clear, "witness": witness}
        pair_records.append(record)
        if clear:
            pair_witness = witness
            break
    base["stages"].append(_stage("PAIR_SWING", 41, int(pair_witness is not None), pair_records, configured_candidate_count=4, visual_components_and_self_required=True, synchronized_common_time=True, visual_sample_count=0 if pair_witness is None else pair_witness["visual_sample_count"]))
    if pair_witness is None:
        base["blocked"] = {"stage": "PAIR_SWING", "reason": "all four fixed PAIR trajectory templates failed", "earliest_witness": pair_records[0]}; base["planner_elapsed_s"] = time.monotonic() - started
        return json_value(base)
    pair_anchors = rm_anchors.copy(); pair_anchors[[RB, RF]] = goal["goals_world_m"][:2]
    pair_q = np.asarray(pair_witness["q_rad"], dtype=np.float64)
    base["nodes"]["PAIR"] = {**_node(body_pose, pair_q, pair_anchors), "goals_world_m": {"rb": goal["goals_world_m"][0].tolist(), "rf": goal["goals_world_m"][1].tolist()}, "goal_contact": pair_witness["goal_contact"], "terminal_axis_angles_deg": terminal_axis_angles_deg(model, pair_q, body_transform, (support_normal_world, support_normal_world), (RB, RF)), "synchronized_common_time": True}
    base["status"] = "MODEL_PATH_FOUND"
    base["result"] = "all verification leaves passed visual-CAD model checks only; real clearance/contact/load remain unresolved"
    base["planner_elapsed_s"] = time.monotonic() - started
    return json_value(base)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "src/grasp_hexapod_control/config/climb_trace.json")
    parser.add_argument("--stop-after", choices=("PREP", "RM", "BODY", "PAIR"), default="PAIR", help="写入截至该阶段的完整采样记录；PAIR 为整条链路默认值")
    args = parser.parse_args(argv)
    trace = build_trace(args.stop_after)
    args.output.write_text(json.dumps(trace, indent=2, sort_keys=True) + "\n")
    print("{}: {} ({:.3f}s)".format(trace["status"], trace.get("blocked", {}).get("stage", ""), trace["planner_elapsed_s"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
