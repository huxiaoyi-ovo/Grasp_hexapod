"""Deterministic fixed back-half visual-CAD planner for CC011.

This offline-only planner starts from the certified PAIR node in
``climb_trace.json``.  It verifies one fixed sequence; there is no foothold,
pose, triangle, or beam search.  Verification leaves are adaptively inserted
IK/visual samples, not trajectory optimization variables.  Visual STL is the
collision authority.  Controller capsules are recorded only as diagnostics.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np


SCRIPTS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS_DIR))

from utils import climb_collision as visual
from utils import climb
import control
import kinematics as k
from tools import plan_climb_trajectory as front
from utils import distance_to_polygon_boundary, points_in_polygon


ROOT = SCRIPTS_DIR.parents[2]
CONFIG_PATH = ROOT / "src/grasp_hexapod_control/config/climb.json"
SOURCE_TRACE_PATH = ROOT / "src/grasp_hexapod_control/config/climb_trace.json"
OUTPUT_PATH = ROOT / "src/grasp_hexapod_control/config/climb_back_trace.json"
XIAOLAN_MESH_PATH = (
    ROOT
    / "src/grasp_hexapod_description/meshes/xiaolan/base_link_xiaolan.STL"
)
WORLD_FROM_XIAOLAN = front.WORLD_FROM_XIAOLAN
LEG_NAMES = k.LEG_NAMES
LB, LF, LM, RB, RF, RM = range(6)

MAX_SAMPLES = 501
MAX_RESIDUAL_M = 1e-6
MAX_JOINT_STEP_RAD = 0.05
MAX_FOOT_STEP_M = 0.003
MAX_BASE_STEP_M = 0.003
MAX_ANGLE_STEP_RAD = 0.02
MIN_JOINT_MARGIN_RAD = 0.03
MIN_SUPPORT_MARGIN_M = 0.005
RM_HIGH_C_LIFT = 0.02
RM_HIGH_C_TARGET_FACET_ID = 18028
RM_HIGH_C_TARGET_NORMAL = np.array(
    (-0.08710263548618817, -0.034899478772949986, 0.9955878450808532),
    dtype=np.float64,
)
VISUAL_FOOTPAD_COMPONENT_EXCLUSIONS = tuple(
    (
        "ankle",
        leg,
        visual.ANKLE_VISUAL_FOOTPAD_COMPONENT_INDEX,
    )
    for leg in range(6)
)
STATIC_VISUAL_FOOTPAD_HITS_CACHE_KEY = (
    "back_static_visual_footpad_hits"
)

STAGE_NAMES = (
    "LB_LF_GROUND_SHIFT",
    "LM_GROUND_SHIFT",
    "BODY2",
    "RM_HIGH_C",
    "RB_RF_HIGH_C",
    "BODY3",
    "RM_SHIFT1",
    "RB_RF_SHIFT1",
    "LB_LF_GROUND_SHIFT1",
    "LM_GROUND_SHIFT1",
    "BODY4",
    "RB_RF_TOP_INWARD",
    "LB_LF_GROUND_EDGE",
    "LM_GROUND_EDGE",
    "RM_A",
    "BODY_A",
    "RM_B",
    "LM_LOW_LAND_FIRST",
    "LF_LOW_LAND_SECOND",
    "LB_HOVER_PREP",
    "LB_LOW_LAND_LAST",
    "FINAL_HOLD",
)


def pose_matrix(pose):
    """生成不含偏航角的机身世界坐标变换矩阵。

    参数:
        pose: 机身的 x、y、z、横滚角和俯仰角。

    返回:
        `T * Ry(pitch) * Rx(roll)` 变换矩阵。
    """
    x, y, z, roll, pitch = np.asarray(pose, dtype=np.float64)
    cosine_r, sine_r = np.cos(roll), np.sin(roll)
    cosine_p, sine_p = np.cos(pitch), np.sin(pitch)
    rotation_x = np.array(
        ((1.0, 0.0, 0.0), (0.0, cosine_r, -sine_r), (0.0, sine_r, cosine_r)),
        dtype=np.float64,
    )
    rotation_y = np.array(
        ((cosine_p, 0.0, sine_p), (0.0, 1.0, 0.0), (-sine_p, 0.0, cosine_p)),
        dtype=np.float64,
    )
    output = np.eye(4, dtype=np.float64)
    output[:3, :3] = rotation_y @ rotation_x
    output[:3, 3] = (x, y, z)
    return output


def world_points(transform, points):
    return np.asarray(points) @ transform[:3, :3].T + transform[:3, 3]


def base_points(transform, points):
    return (np.asarray(points) - transform[:3, 3]) @ transform[:3, :3]


def low_center(local_x, local_y, extraction, normal):
    local_z = (
        extraction.plane_offset_m - normal[0] * local_x - normal[1] * local_y
    ) / normal[2]
    contact = np.array((local_x, local_y, local_z), dtype=np.float64)
    return world_points(WORLD_FROM_XIAOLAN, (contact + k.FOOT_RADIUS * normal)[None])[0]


def _segment_value(knots, value):
    segment_count = len(knots) - 1
    segment = min(int(value), segment_count - 1)
    local = value - segment if value < segment_count else 1.0
    weight = front.smoothstep(local)
    return knots[segment] * (1.0 - weight) + knots[segment + 1] * weight


def _stage_pose(stage, value):
    normalized = value / stage["segments"]
    weight = front.smoothstep(normalized)
    return stage["pose_start"] * (1.0 - weight) + stage["pose_end"] * weight


def _stage_anchors(stage, value):
    return _segment_value(stage["anchor_knots"], value)


def _make_stage(
    name,
    pose_start,
    pose_end,
    anchor_knots,
    active_legs,
    support_legs,
    contacts_start,
    contacts_end,
    minimum_samples=31,
):
    knots = np.asarray(anchor_knots, dtype=np.float64)
    return {
        "name": name,
        "pose_start": np.asarray(pose_start, dtype=np.float64),
        "pose_end": np.asarray(pose_end, dtype=np.float64),
        "anchor_knots": knots,
        "segments": len(knots) - 1,
        "active_legs": tuple(active_legs),
        "support_legs": tuple(support_legs),
        "contacts_start": dict(contacts_start),
        "contacts_end": dict(contacts_end),
        "minimum_samples": int(minimum_samples),
    }


def _body_stage(name, pose_start, pose_end, anchors, support_legs, contacts):
    return _make_stage(
        name,
        pose_start,
        pose_end,
        (anchors.copy(), anchors.copy()),
        (),
        support_legs,
        contacts,
        contacts,
        31,
    )


def _swing_stage(
    name,
    pose_start,
    anchors,
    targets,
    active_legs,
    support_legs,
    contacts,
    final_surfaces,
    lift,
    pose_end=None,
):
    target = anchors.copy()
    for leg, point in zip(active_legs, targets):
        target[leg] = point
    lifted_start = anchors.copy()
    lifted_target = target.copy()
    lifted_start[list(active_legs), 2] += lift
    lifted_target[list(active_legs), 2] += lift
    contacts_end = dict(contacts)
    for leg, surface in zip(active_legs, final_surfaces):
        if surface is None:
            contacts_end.pop(leg, None)
        else:
            contacts_end[leg] = surface
    return _make_stage(
        name,
        pose_start,
        pose_start if pose_end is None else pose_end,
        (anchors.copy(), lifted_start, lifted_target, target),
        active_legs,
        support_legs,
        contacts,
        contacts_end,
        31,
    )


def fixed_stages(source, extraction, normal):
    """生成后半段固定阶段及其终点。

    参数:
        source: 前半段轨迹记录。
        extraction: 小蓝斜面提取结果。
        normal: 低斜面法向量。

    返回:
        固定阶段序列。
    """
    pair = source["nodes"]["PAIR"]
    pose = np.array(
        (*pair["base_world"], 0.0, pair["pitch_rad"]), dtype=np.float64
    )
    anchors = np.asarray(pair["anchors_world_m"], dtype=np.float64).copy()
    y_nominal = float(pair["base_world"][1])
    original_y = anchors[:, 1].copy()
    contacts = {RB: "low", RF: "low", RM: "low"}
    all_legs = tuple(range(6))
    stages = []

    def add(stage):
        nonlocal pose, anchors, contacts
        stages.append(stage)
        pose = stage["pose_end"].copy()
        anchors = stage["anchor_knots"][-1].copy()
        contacts = dict(stage["contacts_end"])

    targets = anchors[[LB, LF]].copy(); targets[:, 0] += 0.09
    add(_swing_stage("LB_LF_GROUND_SHIFT", pose, anchors, targets, (LB, LF),
                     (LM, RB, RF, RM), contacts, (None, None), 0.03))
    target = anchors[LM].copy(); target[0] += 0.12
    add(_swing_stage("LM_GROUND_SHIFT", pose, anchors, (target,), (LM,),
                     (LB, LF, RB, RF, RM), contacts, (None,), 0.03))

    body2 = np.array((0.06, y_nominal, 0.19, 0.0, -0.30))
    add(_body_stage("BODY2", pose, body2, anchors, all_legs, contacts))

    high_c = np.array((0.299434, -0.130227, 0.190408))
    rm_high_c = _swing_stage(
        "RM_HIGH_C",
        pose,
        anchors,
        (high_c,),
        (RM,),
        (LB, LF, LM, RB, RF),
        contacts,
        ("cad",),
        RM_HIGH_C_LIFT,
    )
    rm_high_c["anchor_knots"][1, RM] = (
        anchors[RM] + RM_HIGH_C_LIFT * normal
    )
    rm_high_c["anchor_knots"][2, RM] = (
        high_c + RM_HIGH_C_LIFT * RM_HIGH_C_TARGET_NORMAL
    )
    rm_high_c["trajectory_geometry"] = {
        "lift_m": RM_HIGH_C_LIFT,
        "source_low_surface_normal_world": normal.tolist(),
        "target_upward_facet_id": RM_HIGH_C_TARGET_FACET_ID,
        "target_upward_normal_world": RM_HIGH_C_TARGET_NORMAL.tolist(),
        "anchor_knots_world_m": rm_high_c["anchor_knots"].tolist(),
    }
    add(rm_high_c)
    pair_c = np.array(((0.269434, -0.220227, 0.184628),
                       (0.325, 0.07, 0.1965)))
    add(_swing_stage("RB_RF_HIGH_C", pose, anchors, pair_c, (RB, RF),
                     (LB, LF, LM, RM), contacts, ("cad", "top"), 0.05))

    body3 = np.array((0.10, y_nominal, 0.18, 0.0, -0.30))
    add(_body_stage("BODY3", pose, body3, anchors, all_legs, contacts))
    shift_fraction = 2.0 / 3.0
    rm_shift1_full = np.array((0.3594338511, -0.1302268496, 0.1956570176))
    rm_shift1 = anchors[RM] + shift_fraction * (
        rm_shift1_full - anchors[RM]
    )
    add(_swing_stage("RM_SHIFT1", pose, anchors, (rm_shift1,), (RM,),
                     (LB, LF, LM, RB, RF), contacts, ("cad",), 0.03))
    pair_shift1_full = np.array(((0.3394338511, -0.2202268496, 0.1907523882),
                                 (0.36, 0.07, 0.1965)))
    pair_shift1 = anchors[[RB, RF]] + shift_fraction * (
        pair_shift1_full - anchors[[RB, RF]]
    )
    add(_swing_stage("RB_RF_SHIFT1", pose, anchors, pair_shift1, (RB, RF),
                     (LB, LF, LM, RM), contacts, ("cad", "top"), 0.03))

    targets = anchors[[LB, LF]].copy(); targets[:, 0] += 0.09
    add(_swing_stage("LB_LF_GROUND_SHIFT1", pose, anchors, targets, (LB, LF),
                     (LM, RB, RF, RM), contacts, (None, None), 0.03))
    target = anchors[LM].copy(); target[0] += 0.05
    add(_swing_stage("LM_GROUND_SHIFT1", pose, anchors, (target,), (LM,),
                     (LB, LF, RB, RF, RM), contacts, (None,), 0.03))
    body4 = np.array((0.16, y_nominal, 0.18, 0.0, -0.20))
    add(_body_stage("BODY4", pose, body4, anchors, all_legs, contacts))

    corrected_pair = np.array(((0.43, -0.18, 0.1965),
                               (0.425, 0.02, 0.1965)))
    add(_swing_stage("RB_RF_TOP_INWARD", pose, anchors, corrected_pair,
                     (RB, RF), (LB, LF, LM, RM), contacts,
                     ("top", "top"), 0.03))
    targets = anchors[[LB, LF]].copy(); targets[:, 0] = 0.09; targets[:, 2] = 0.0065
    add(_swing_stage("LB_LF_GROUND_EDGE", pose, anchors, targets, (LB, LF),
                     (LM, RB, RF, RM), contacts, (None, None), 0.03))
    target = anchors[LM].copy(); target[0] = 0.09; target[2] = 0.0065
    add(_swing_stage("LM_GROUND_EDGE", pose, anchors, (target,), (LM,),
                     (LB, LF, RB, RF, RM), contacts, (None,), 0.03))

    rm_a = np.array((0.42, -0.13, 0.1965))
    add(_swing_stage("RM_A", pose, anchors, (rm_a,), (RM,),
                     (LB, LF, LM, RB, RF), contacts, ("top",), 0.03))
    body_a = np.array((0.23, y_nominal, 0.19, 0.0, -0.20))
    add(_body_stage("BODY_A", pose, body_a, anchors, all_legs, contacts))
    rm_b = np.array((0.51, -0.13, 0.1965))
    rm_b_mid = np.array((0.465, -0.13, 0.2065))
    rm_b_mid_anchors = anchors.copy()
    rm_b_end_anchors = anchors.copy()
    rm_b_mid_anchors[RM] = rm_b_mid
    rm_b_end_anchors[RM] = rm_b
    rm_b_contacts = dict(contacts)
    rm_b_contacts[RM] = "top"
    rm_b_stage = _make_stage(
        "RM_B",
        pose,
        pose,
        (anchors.copy(), rm_b_mid_anchors, rm_b_end_anchors),
        (RM,),
        (LB, LF, LM, RB, RF),
        contacts,
        rm_b_contacts,
        31,
    )
    rm_b_stage["trajectory_geometry"] = {
        "curve": "two_segment_piecewise_quintic_smoothstep",
        "segment_count": 2,
        "active_leg": "rm",
        "base_pose_fixed": True,
        "other_anchors_fixed": True,
        "anchor_knots_world_m": rm_b_stage["anchor_knots"].tolist(),
    }
    add(rm_b_stage)

    lm_local_y = original_y[RM] + 0.03
    lm_goal = low_center(-0.25, lm_local_y, extraction, normal)
    add(_swing_stage(
        "LM_LOW_LAND_FIRST",
        pose,
        anchors,
        (lm_goal,),
        (LM,),
        (LB, LF, RB, RF, RM),
        contacts,
        ("low",),
        0.04,
    ))

    lf_local_y = original_y[RF] + 0.03 + 0.02
    lf_goal = low_center(-0.25, lf_local_y, extraction, normal)
    lf_pose = np.array((0.23, y_nominal - 0.03, 0.19, 0.10, -0.20))
    add(_swing_stage(
        "LF_LOW_LAND_SECOND",
        pose,
        anchors,
        (lf_goal,),
        (LF,),
        (LB, LM, RB, RF, RM),
        contacts,
        ("low",),
        0.04,
        pose_end=lf_pose,
    ))

    lb_hover = np.array((0.09, original_y[LB], 0.08))
    lb_hover_pose = np.array(
        (0.245, y_nominal - 0.02, 0.210, 0.10, -0.15)
    )
    add(_swing_stage(
        "LB_HOVER_PREP",
        pose,
        anchors,
        (lb_hover,),
        (LB,),
        (LF, LM, RB, RF, RM),
        contacts,
        (None,),
        0.02,
        pose_end=lb_hover_pose,
    ))

    lb_local_y = original_y[RB] + 0.03 + 0.005
    lb_goal = low_center(-0.25, lb_local_y, extraction, normal)
    lb_land_pose = np.array(
        (0.245, y_nominal - 0.02, 0.220, 0.0, -0.15)
    )
    target = anchors.copy(); target[LB] = lb_goal
    contacts_end = dict(contacts); contacts_end[LB] = "low"
    add(_make_stage("LB_LOW_LAND_LAST", pose, lb_land_pose,
                    (anchors.copy(), target), (LB,),
                    (LF, LM, RB, RF, RM), contacts, contacts_end, 61))

    add(_body_stage(
        "FINAL_HOLD", pose, pose.copy(), anchors, all_legs, contacts
    ))
    return stages


def _adaptive_samples(model, seed, stage):
    samples = []

    def pose_at(value):
        return _stage_pose(stage, value)

    def anchors_at(value):
        return _stage_anchors(stage, value)

    def append(value, q_value, residual):
        if len(samples) >= MAX_SAMPLES:
            raise ValueError("adaptive sample cap exceeded")
        samples.append({
            "t": float(value),
            "pose": pose_at(value),
            "transform": pose_matrix(pose_at(value)),
            "anchors": anchors_at(value),
            "q_rad": np.asarray(q_value).copy(),
            "residual_m": float(residual),
        })

    def refine(left_t, left_q, right_t):
        right_pose = pose_at(right_t)
        right_transform = pose_matrix(right_pose)
        right_anchors = anchors_at(right_t)
        tolerance = 1e-10 if right_t == stage["segments"] else 1e-7
        right_q, residual = front.solve_anchors(
            model, right_anchors, right_transform, left_q, tolerance
        )
        left_pose = pose_at(left_t)
        left_anchors = anchors_at(left_t)
        joint_step = float(np.max(np.abs(right_q - left_q)))
        if stage["active_legs"]:
            active = list(stage["active_legs"])
            foot_step = float(np.max(np.linalg.norm(
                right_anchors[active] - left_anchors[active], axis=1
            )))
        else:
            foot_step = 0.0
        base_step = float(np.linalg.norm(right_pose[:3] - left_pose[:3]))
        roll_step = abs(float(right_pose[3] - left_pose[3]))
        pitch_step = abs(float(right_pose[4] - left_pose[4]))
        if (
            joint_step > MAX_JOINT_STEP_RAD
            or foot_step > MAX_FOOT_STEP_M
            or base_step > MAX_BASE_STEP_M
            or roll_step > MAX_ANGLE_STEP_RAD
            or pitch_step > MAX_ANGLE_STEP_RAD
        ):
            if len(samples) >= MAX_SAMPLES:
                raise ValueError("adaptive sample cap exceeded")
            middle = 0.5 * (left_t + right_t)
            middle_q = refine(left_t, left_q, middle)
            return refine(middle, middle_q, right_t)
        append(right_t, right_q, residual)
        return right_q

    initial_transform = pose_matrix(stage["pose_start"])
    q0, residual0 = front.solve_anchors(
        model, stage["anchor_knots"][0], initial_transform, seed, 1e-10
    )
    append(0.0, q0, residual0)
    coarse_count = max(stage["minimum_samples"], stage["segments"] * 10 + 1)
    coarse = np.linspace(0.0, stage["segments"], coarse_count)
    left_t, q_value = 0.0, q0
    for right_t in coarse[1:]:
        q_value = refine(left_t, q_value, float(right_t))
        left_t = float(right_t)
    return samples


def _top_triangles(scene):
    return scene.xiaolan_mesh_local.triangles[
        sorted(_top_triangle_ids(scene))
    ]


def _top_triangle_ids(scene):
    triangles = scene.xiaolan_mesh_local.triangles
    mask = np.max(np.abs(triangles[:, :, 2] - 0.19), axis=1) < 2e-5
    return frozenset(int(index) for index in np.flatnonzero(mask))


def _point_in_triangle_xy(point, triangle):
    first, second, third = triangle[:, :2]
    v0, v1, v2 = third - first, second - first, point - first
    denominator = v0[0] * v1[1] - v1[0] * v0[1]
    if abs(denominator) < 1e-15:
        return False
    u = (v2[0] * v1[1] - v1[0] * v2[1]) / denominator
    v = (v0[0] * v2[1] - v2[0] * v0[1]) / denominator
    return u >= -1e-9 and v >= -1e-9 and u + v <= 1.0 + 1e-9


def _contact_state(surface, center_world, extraction, normal, top, expected=None):
    center_local = base_points(WORLD_FROM_XIAOLAN, center_world[None])[0]
    if surface == "low":
        contact = center_local - k.FOOT_RADIUS * normal
        plane_error = abs(float(contact @ normal - extraction.plane_offset_m))
        inside = bool(points_in_polygon(
            contact[None, :2], extraction.outer_loop_vertices_3d_m[:, :2]
        )[0])
        boundary = float(distance_to_polygon_boundary(
            contact[None, :2], extraction.outer_loop_vertices_3d_m[:, :2]
        )[0])
        return {
            "surface": surface,
            "contact_local_m": contact.tolist(),
            "plane_error_m": plane_error,
            "polygon_inside": inside,
            "boundary_distance_m": boundary,
            "valid": plane_error <= 1e-6 and inside and boundary > k.FOOT_RADIUS,
        }
    if surface == "top":
        contact = center_local - np.array((0.0, 0.0, k.FOOT_RADIUS))
        plane_error = abs(float(contact[2] - 0.19))
        inside = any(_point_in_triangle_xy(contact[:2], item) for item in top)
        return {
            "surface": surface,
            "contact_local_m": contact.tolist(),
            "plane_error_m": plane_error,
            "polygon_inside": bool(inside),
            "valid": plane_error <= 1e-6 and bool(inside),
        }
    if surface == "cad":
        error = float(np.linalg.norm(center_world - expected))
        return {
            "surface": surface,
            "fixed_target_error_m": error,
            "fixed_CAD_target_is_planned_contact_not_contact_proof": True,
            "valid": error <= 1e-6,
        }
    return {"surface": surface, "valid": surface == "ground"}


def _hit_value(result):
    return front.hit_value(result)


def _allowed_contacts(stage, time_value):
    if time_value <= 1e-12:
        return dict(stage["contacts_start"])
    segment = min(int(time_value - 1e-12), stage["segments"] - 1)
    allowed = dict(stage["contacts_start"])
    for leg in stage["active_legs"]:
        allowed.pop(leg, None)
    if segment == stage["segments"] - 1:
        for leg in stage["active_legs"]:
            if leg in stage["contacts_end"]:
                allowed[leg] = stage["contacts_end"][leg]
    return allowed


def _stage_contact_state(
    surface,
    center_world,
    stage,
    leg,
    extraction,
    normal,
    top,
    expected_anchor=None,
):
    # 始终与阶段终点比较，避免中途碰撞被当前插值位置误判为有效接触。
    expected = (
        stage["anchor_knots"][-1][leg]
        if expected_anchor is None
        else expected_anchor
    )
    return _contact_state(
        surface,
        center_world,
        extraction,
        normal,
        top,
        expected,
    )


def _visual_contact_expected_anchor(stage, leg, time_value):
    index = 0 if time_value <= 1e-12 else -1
    return stage["anchor_knots"][index][leg]


def _classified_surface_hits(
    scene,
    surface,
    hits,
    contact_state,
    low_triangle_ids,
    top_triangle_ids,
):
    """判断计划接触点命中的小蓝面片是否允许。

    参数:
        scene: 可视碰撞场景。
        surface: 计划接触表面。
        hits: 命中的面片。
        contact_state: 当前接触状态。
        low_triangle_ids: 允许的低斜面面片索引。
        top_triangle_ids: 允许的顶面面片索引。

    返回:
        是否符合计划，以及分类结果。
    """
    hits = tuple(int(index) for index in hits)
    receipt = {
        "surface": surface,
        "raw_hit_ids_diagnostic": list(hits),
        "all_hit_facets_match_planned_support_surface": False,
    }
    if not hits:
        receipt["all_hit_facets_match_planned_support_surface"] = True
        return True, receipt
    if surface is None:
        receipt["reason"] = "contact_not_allowed_in_this_segment"
        return False, receipt
    if -1 in hits:
        receipt["reason"] = "surface_free_containment_is_unclassified"
        return False, receipt
    if not contact_state["valid"]:
        receipt["reason"] = "planned_contact_state_invalid"
        receipt["contact_state"] = contact_state
        return False, receipt
    if surface == "low":
        unexpected = sorted(set(hits) - set(low_triangle_ids))
    elif surface == "top":
        unexpected = sorted(set(hits) - set(top_triangle_ids))
    elif surface == "cad":
        unexpected = []
        for triangle_index in hits:
            triangle = scene.xiaolan_mesh_local.triangles[triangle_index]
            facet_normal = np.cross(
                triangle[1] - triangle[0], triangle[2] - triangle[0]
            )
            norm = float(np.linalg.norm(facet_normal))
            if norm == 0.0 or abs(float(facet_normal[2] / norm)) < 0.9:
                unexpected.append(triangle_index)
    else:
        unexpected = list(hits)
    if unexpected:
        receipt["reason"] = "side_or_unclassified_xiaolan_facet"
        receipt["unexpected_hit_ids"] = unexpected
        return False, receipt
    receipt["all_hit_facets_match_planned_support_surface"] = True
    return True, receipt


def _cache_static_visual_footpad_hits(
    scene, cache, q_value, transform
):
    """缓存固定腿脚垫命中的原始面片索引。

    参数:
        scene: 可视碰撞场景。
        cache: 当前阶段缓存。
        q_value: 关节角。
        transform: 机身世界坐标变换矩阵。

    返回:
        加入脚垫命中信息的缓存。
    """
    output = dict(cache)
    active_legs = frozenset(cache["active_legs"])
    output[STATIC_VISUAL_FOOTPAD_HITS_CACHE_KEY] = {
        leg: scene.component_xiaolan_triangle_hits(
            scene.transformed_robot_component(
                q_value,
                transform,
                "ankle",
                leg,
                visual.ANKLE_VISUAL_FOOTPAD_COMPONENT_INDEX,
            )
        )
        for leg in range(6)
        if leg not in active_legs
    }
    return output


def _visual_footpad_hits(scene, cache, q_value, transform, leg):
    if cache is not None:
        static_hits = cache.get(
            STATIC_VISUAL_FOOTPAD_HITS_CACHE_KEY, {}
        )
        if leg in static_hits:
            return tuple(static_hits[leg])
    footpad = scene.transformed_robot_component(
        q_value,
        transform,
        "ankle",
        leg,
        visual.ANKLE_VISUAL_FOOTPAD_COMPONENT_INDEX,
    )
    return scene.component_xiaolan_triangle_hits(footpad)


def _visual_leaf(
    scene,
    model,
    sample,
    stage,
    cache,
    extraction,
    normal,
    top,
    low_triangle_ids,
    top_triangle_ids,
):
    q_value = sample["q_rad"]
    transform = sample["transform"]
    if cache is None:
        components = scene.robot_components(q_value, transform)
        self_result = scene.self_collision(q_value, transform)
        object_result = scene.components_vs_xiaolan(
            components,
            exclude_components=VISUAL_FOOTPAD_COMPONENT_EXCLUSIONS,
        )
        if self_result.collision or object_result.collision:
            return False, {
                "self_hit": _hit_value(self_result),
                "xiaolan_hit": _hit_value(object_result),
            }
    else:
        clear, witness = front.cached_phase_visual(
            scene,
            cache,
            q_value,
            transform,
            VISUAL_FOOTPAD_COMPONENT_EXCLUSIONS,
        )
        if not clear:
            return False, witness

    allowed = _allowed_contacts(stage, sample["t"])
    feet = scene.foot_spheres(q_value, transform)
    for leg in range(6):
        hits = _visual_footpad_hits(
            scene, cache, q_value, transform, leg
        )
        if not hits:
            continue
        surface = allowed.get(leg)
        state = _stage_contact_state(
            surface,
            feet[leg].center,
            stage,
            leg,
            extraction,
            normal,
            top,
            _visual_contact_expected_anchor(stage, leg, sample["t"]),
        )
        classified, receipt = _classified_surface_hits(
            scene,
            surface,
            hits,
            state,
            low_triangle_ids,
            top_triangle_ids,
        )
        if not classified:
            return False, {
                "visual_footpad": LEG_NAMES[leg],
                "contact_state": state,
                **receipt,
            }
    for leg in range(6):
        hits = tuple(scene.foot_xiaolan_triangle_hits(leg, q_value, transform))
        if not hits:
            continue
        surface = allowed.get(leg)
        state = _stage_contact_state(
            surface,
            feet[leg].center,
            stage,
            leg,
            extraction,
            normal,
            top,
            _visual_contact_expected_anchor(stage, leg, sample["t"]),
        )
        classified, receipt = _classified_surface_hits(
            scene,
            surface,
            hits,
            state,
            low_triangle_ids,
            top_triangle_ids,
        )
        if not classified:
            return False, {
                "foot": LEG_NAMES[leg],
                "contact_state": state,
                **receipt,
            }
    return True, {}


def _new_metrics():
    return {
        "max_ik_residual_m": 0.0,
        "max_adjacent_joint_delta_rad": 0.0,
        "max_active_foot_step_m": 0.0,
        "max_base_translation_step_m": 0.0,
        "max_roll_step_rad": 0.0,
        "max_pitch_step_rad": 0.0,
        "min_joint_margin_rad": float("inf"),
        "min_sigma": float("inf"),
        "min_support_margin_m": float("inf"),
    }


def _update_metrics(metrics, model, sample, previous, support):
    q_value = sample["q_rad"]
    metrics["max_ik_residual_m"] = max(
        metrics["max_ik_residual_m"], sample["residual_m"]
    )
    metrics["min_joint_margin_rad"] = min(
        metrics["min_joint_margin_rad"],
        float(np.min(model.joint_limit_margins(q_value))),
    )
    metrics["min_sigma"] = min(
        metrics["min_sigma"],
        float(np.min(model.jacobian_min_singular_values(q_value))),
    )
    metrics["min_support_margin_m"] = min(
        metrics["min_support_margin_m"], support
    )
    if previous is None:
        return
    metrics["max_adjacent_joint_delta_rad"] = max(
        metrics["max_adjacent_joint_delta_rad"],
        float(np.max(np.abs(q_value - previous["q_rad"]))),
    )
    active = list(sample["stage_active_legs"])
    if active:
        metrics["max_active_foot_step_m"] = max(
            metrics["max_active_foot_step_m"],
            float(np.max(np.linalg.norm(
                sample["anchors"][active] - previous["anchors"][active], axis=1
            ))),
        )
    metrics["max_base_translation_step_m"] = max(
        metrics["max_base_translation_step_m"],
        float(np.linalg.norm(sample["pose"][:3] - previous["pose"][:3])),
    )
    metrics["max_roll_step_rad"] = max(
        metrics["max_roll_step_rad"],
        abs(float(sample["pose"][3] - previous["pose"][3])),
    )
    metrics["max_pitch_step_rad"] = max(
        metrics["max_pitch_step_rad"],
        abs(float(sample["pose"][4] - previous["pose"][4])),
    )


def _terminal_contacts(
    scene,
    model,
    sample,
    stage,
    extraction,
    normal,
    top,
    low_triangle_ids,
    top_triangle_ids,
):
    feet = scene.foot_spheres(sample["q_rad"], sample["transform"])
    result = {}
    for leg, surface in stage["contacts_end"].items():
        if surface not in ("low", "top", "cad"):
            continue
        state = _stage_contact_state(
            surface,
            feet[leg].center,
            stage,
            leg,
            extraction,
            normal,
            top,
        )
        sphere_hits = scene.foot_xiaolan_triangle_hits(
            leg, sample["q_rad"], sample["transform"]
        )
        footpad = scene.transformed_robot_component(
            sample["q_rad"],
            sample["transform"],
            "ankle",
            leg,
            visual.ANKLE_VISUAL_FOOTPAD_COMPONENT_INDEX,
        )
        footpad_hits = scene.component_xiaolan_triangle_hits(footpad)
        sphere_clear, sphere_receipt = _classified_surface_hits(
            scene,
            surface,
            sphere_hits,
            state,
            low_triangle_ids,
            top_triangle_ids,
        )
        footpad_clear, footpad_receipt = _classified_surface_hits(
            scene,
            surface,
            footpad_hits,
            state,
            low_triangle_ids,
            top_triangle_ids,
        )
        state["raw_hit_ids_diagnostic"] = list(sphere_hits)
        state["visual_footpad_hit_ids_diagnostic"] = list(footpad_hits)
        state["visual_footpad_component_contact_classified_not_ignored"] = True
        if not state["valid"] or not sphere_clear or not footpad_clear:
            return False, {"foot": LEG_NAMES[leg], "contact_state": state}
        state["analytic_sphere_hit_classification"] = sphere_receipt
        state["visual_footpad_hit_classification"] = footpad_receipt
        result[LEG_NAMES[leg]] = state
    return True, result


def evaluate_stage(
    scene,
    model,
    controller,
    state,
    stage,
    extraction,
    normal,
    top,
    low_triangle_ids,
    top_triangle_ids,
):
    started = time.monotonic()
    try:
        samples = _adaptive_samples(model, state["q"], stage)
    except (ValueError, RecursionError) as exc:
        return None, {
            "name": stage["name"],
            "clear": False,
            "reason": "adaptive_sample_cap",
            "witness": {"detail": str(exc), "sample_cap": MAX_SAMPLES},
            "elapsed_s": time.monotonic() - started,
        }
    cacheable = (
        bool(stage["active_legs"])
        and np.array_equal(stage["pose_start"], stage["pose_end"])
    )
    cache = None
    if cacheable:
        cache = front.build_phase_cache(
            scene,
            samples[0]["q_rad"],
            samples[0]["transform"],
            stage["active_legs"],
            VISUAL_FOOTPAD_COMPONENT_EXCLUSIONS,
        )
        cache = _cache_static_visual_footpad_hits(
            scene,
            cache,
            samples[0]["q_rad"],
            samples[0]["transform"],
        )
    metrics = _new_metrics()
    previous = None
    for index, sample in enumerate(samples):
        sample["stage_active_legs"] = stage["active_legs"]
        q_value = sample["q_rad"]
        margin = float(np.min(model.joint_limit_margins(q_value)))
        world_com = world_points(
            sample["transform"], model.center_of_mass_base(q_value)[None]
        )[0]
        support = climb.gravity_projected_support(
            world_com,
            sample["anchors"][list(stage["support_legs"])],
            np.array((0.0, 0.0, -9.81)),
        )
        support_margin = float(support.raw_margin_m)
        _update_metrics(metrics, model, sample, previous, support_margin)
        if (
            sample["residual_m"] > MAX_RESIDUAL_M
            or margin < MIN_JOINT_MARGIN_RAD
            or support_margin < MIN_SUPPORT_MARGIN_M
        ):
            return None, {
                "name": stage["name"],
                "clear": False,
                "reason": "numeric_or_support_gate",
                "witness": {
                    "sample": index,
                    "residual_m": sample["residual_m"],
                    "joint_margin_rad": margin,
                    "support_margin_m": support_margin,
                },
                "metrics": metrics,
                "visual_sample_count": index,
                "elapsed_s": time.monotonic() - started,
            }
        clear, witness = _visual_leaf(
            scene,
            model,
            sample,
            stage,
            cache,
            extraction,
            normal,
            top,
            low_triangle_ids,
            top_triangle_ids,
        )
        if not clear:
            return None, {
                "name": stage["name"],
                "clear": False,
                "reason": "visual_stl",
                "witness": {"sample": index, **witness},
                "metrics": metrics,
                "visual_sample_count": index + 1,
                "elapsed_s": time.monotonic() - started,
            }
        previous = sample
    contact_clear, contacts = _terminal_contacts(
        scene,
        model,
        samples[-1],
        stage,
        extraction,
        normal,
        top,
        low_triangle_ids,
        top_triangle_ids,
    )
    if not contact_clear:
        return None, {
            "name": stage["name"],
            "clear": False,
            "reason": "terminal_plane_polygon_contact",
            "witness": contacts,
            "metrics": metrics,
            "visual_sample_count": len(samples),
            "elapsed_s": time.monotonic() - started,
        }
    endpoint_q = samples[-1]["q_rad"]
    record = {
        "name": stage["name"],
        "clear": True,
        "active_legs": [LEG_NAMES[item] for item in stage["active_legs"]],
        "support_legs": [LEG_NAMES[item] for item in stage["support_legs"]],
        "minimum_samples": stage["minimum_samples"],
        "visual_sample_count": len(samples),
        "verification_leaves_are_not_optimized_poses": True,
        "phase_cache_used": cacheable,
        "visual_footpad_component_contact_classified_not_ignored": True,
        "metrics": metrics,
        "terminal_contacts": contacts,
        "analytic_capsule_diagnostic_only": {
            "link_collision_free": controller._link_collision_free(endpoint_q).tolist(),
            "foot_collision_free": controller._foot_collision_free(
                model.forward_base(endpoint_q)
            ).tolist(),
        },
        "elapsed_s": time.monotonic() - started,
    }
    if "trajectory_geometry" in stage:
        record["trajectory_geometry"] = _json_value(
            stage["trajectory_geometry"]
        )
    new_state = {
        "q": endpoint_q.copy(),
        "pose": samples[-1]["pose"].copy(),
        "anchors": samples[-1]["anchors"].copy(),
        "contacts": dict(stage["contacts_end"]),
    }
    return new_state, record


def _node(state):
    pose = np.asarray(state["pose"], dtype=np.float64)
    return {
        "pose_world": pose.tolist(),
        "base_world": pose[:3].tolist(),
        "roll_rad": float(pose[3]),
        "pitch_rad": float(pose[4]),
        "rotation_order": "T_Ry_pitch_Rx_roll",
        "q_rad": np.asarray(state["q"]).tolist(),
        "anchors_world_m": np.asarray(state["anchors"]).tolist(),
        "planned_contacts": {
            LEG_NAMES[leg]: surface for leg, surface in state["contacts"].items()
        },
    }


def _json_value(value):
    return front.json_value(value)


def _atomic_write(path, trace):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(_json_value(trace), indent=2, sort_keys=True) + "\n")
    temporary.replace(path)


def _fixed_targets(stages):
    by_name = {stage["name"]: stage for stage in stages}
    return {
        "HIGH_C": {
            "rm": by_name["RM_HIGH_C"]["anchor_knots"][-1][RM].tolist(),
            "rb": by_name["RB_RF_HIGH_C"]["anchor_knots"][-1][RB].tolist(),
            "rf": by_name["RB_RF_HIGH_C"]["anchor_knots"][-1][RF].tolist(),
            "surface_policy": {"rm": "cad", "rb": "cad", "rf": "top"},
            "fixed_CAD_target_is_planned_contact_not_contact_proof": True,
        },
        "right_high_final_world_m": {
            "rb": by_name["RB_RF_TOP_INWARD"]["anchor_knots"][-1][RB].tolist(),
            "rf": by_name["RB_RF_TOP_INWARD"]["anchor_knots"][-1][RF].tolist(),
            "rm": by_name["RM_B"]["anchor_knots"][-1][RM].tolist(),
        },
        "LF_C1": {
            "pose_world": by_name["LF_C1"]["pose_end"].tolist(),
            "goal_world_m": by_name["LF_C1"]["anchor_knots"][-1][LF].tolist(),
        },
        "LB_HOVER": {
            "goal_world_m": by_name["LB_HOVER"]["anchor_knots"][-1][LB].tolist(),
        },
        "B1": {"pose_world": by_name["B1"]["pose_end"].tolist()},
        "D4": {
            "pose_world": by_name["D4_LB_LOW"]["pose_end"].tolist(),
            "lb_goal_world_m": by_name["D4_LB_LOW"]["anchor_knots"][-1][LB].tolist(),
        },
        "M2_DONE": {
            "pose_world": by_name["M2_LM_LOW"]["pose_end"].tolist(),
            "lm_goal_world_m": by_name["M2_LM_LOW"]["anchor_knots"][-1][LM].tolist(),
        },
    }


def _fixed_stage_trajectory_geometry(stages):
    return {
        stage["name"]: _json_value(stage["trajectory_geometry"])
        for stage in stages
        if "trajectory_geometry" in stage
    }


def _trace_contract(source_bytes, scene, stages):
    return {
        "schema": "SIMULATION_ONLY_BACK_HALF_VISUAL_MODEL_TRACE_V1",
        "simulation_only": True,
        "model_only_not_contact_or_load_proof": True,
        "source_trace": {
            "path": "climb_trace.json",
            "sha256": hashlib.sha256(source_bytes).hexdigest(),
            "required_node": "PAIR",
        },
        "world_from_xiaolan": WORLD_FROM_XIAOLAN.tolist(),
        "rotation_order": "T_Ry_pitch_Rx_roll",
        "leg_order": list(LEG_NAMES),
        "URDF_COLLISION_GEOMETRY_NOT_USED": True,
        "CONTROLLER_CAPSULE_COLLISION_DIAGNOSTIC_ONLY": True,
        "fixed_CAD_target_is_planned_contact_not_contact_proof": True,
        "visual_footpad_component_contact_classified_not_ignored": True,
        "visual_footpad_component_binding": {
            "link": "ankle",
            "component_index": visual.ANKLE_VISUAL_FOOTPAD_COMPONENT_INDEX,
            "mesh_sha256": visual.ANKLE_VISUAL_FOOTPAD_MESH_SHA256,
            "semantics": visual.ANKLE_VISUAL_FOOTPAD_SEMANTICS,
        },
        "REAL_CLEARANCE_UNRESOLVED": True,
        "verification_leaves_are_not_optimized_poses": True,
        "limitations": [
            "visual CAD is not measured real clearance",
            "visual zero intersection does not certify positive clearance",
            "geometry-only support is not contact, friction, or load proof",
            "real motor, foot, backlash, compliance, and terrain tolerances remain unresolved",
        ],
        "mesh_sha256": {
            "xiaolan": hashlib.sha256(XIAOLAN_MESH_PATH.read_bytes()).hexdigest(),
            "body": scene.body_mesh.sha256,
            **{name: mesh.sha256 for name, mesh in scene.link_meshes.items()},
        },
        "thresholds": {
            "max_ik_residual_m": MAX_RESIDUAL_M,
            "max_adjacent_joint_delta_rad": MAX_JOINT_STEP_RAD,
            "max_active_foot_step_m": MAX_FOOT_STEP_M,
            "max_base_translation_step_m": MAX_BASE_STEP_M,
            "max_roll_or_pitch_step_rad": MAX_ANGLE_STEP_RAD,
            "min_joint_margin_rad": MIN_JOINT_MARGIN_RAD,
            "min_geometry_support_margin_m": MIN_SUPPORT_MARGIN_M,
            "adaptive_sample_cap": MAX_SAMPLES,
        },
        "fixed_targets": _fixed_targets(stages),
        "fixed_stage_trajectory_geometry": (
            _fixed_stage_trajectory_geometry(stages)
        ),
        "configured_stage_order": list(STAGE_NAMES),
    }


def _audit_completed_stage_record(record, stage):
    expected_active = [LEG_NAMES[leg] for leg in stage["active_legs"]]
    expected_support = [LEG_NAMES[leg] for leg in stage["support_legs"]]
    expected_cache = (
        bool(stage["active_legs"])
        and np.array_equal(stage["pose_start"], stage["pose_end"])
    )
    count = record.get("visual_sample_count")
    if (
        record.get("active_legs") != expected_active
        or record.get("support_legs") != expected_support
        or record.get("minimum_samples") != stage["minimum_samples"]
        or not isinstance(count, int)
        or isinstance(count, bool)
        or count < stage["minimum_samples"]
        or record.get("phase_cache_used") is not expected_cache
        or record.get(
            "visual_footpad_component_contact_classified_not_ignored"
        ) is not True
    ):
        raise ValueError(
            "resume stage receipt structure mismatch: " + stage["name"]
        )

    maximum_metrics = {
        "max_ik_residual_m": MAX_RESIDUAL_M,
        "max_adjacent_joint_delta_rad": MAX_JOINT_STEP_RAD,
        "max_active_foot_step_m": MAX_FOOT_STEP_M,
        "max_base_translation_step_m": MAX_BASE_STEP_M,
        "max_roll_step_rad": MAX_ANGLE_STEP_RAD,
        "max_pitch_step_rad": MAX_ANGLE_STEP_RAD,
    }
    minimum_metrics = {
        "min_joint_margin_rad": MIN_JOINT_MARGIN_RAD,
        "min_support_margin_m": MIN_SUPPORT_MARGIN_M,
    }
    expected_metric_keys = {
        *maximum_metrics,
        *minimum_metrics,
        "min_sigma",
    }
    metrics = record.get("metrics")
    if not isinstance(metrics, dict) or set(metrics) != expected_metric_keys:
        raise ValueError(
            "resume stage metric fields mismatch: " + stage["name"]
        )
    for name, value in metrics.items():
        if (
            not isinstance(value, (int, float))
            or isinstance(value, bool)
            or not np.isfinite(value)
        ):
            raise ValueError(
                "resume stage metric is not finite: "
                + stage["name"]
                + "/"
                + name
            )
    tolerance = 1e-12
    for name, limit in maximum_metrics.items():
        if metrics[name] < 0.0 or metrics[name] > limit + tolerance:
            raise ValueError(
                "resume stage maximum metric failed: "
                + stage["name"]
                + "/"
                + name
            )
    for name, limit in minimum_metrics.items():
        if metrics[name] < limit - tolerance:
            raise ValueError(
                "resume stage minimum metric failed: "
                + stage["name"]
                + "/"
                + name
            )
    if metrics["min_sigma"] < 0.0:
        raise ValueError("resume stage min_sigma failed: " + stage["name"])

    if "trajectory_geometry" in stage and _json_value(
        record.get("trajectory_geometry")
    ) != _json_value(stage["trajectory_geometry"]):
        raise ValueError(
            "resume stage trajectory geometry mismatch: " + stage["name"]
        )

    expected_contacts = {
        LEG_NAMES[leg]: surface
        for leg, surface in stage["contacts_end"].items()
        if surface in ("low", "top", "cad")
    }
    terminal = record.get("terminal_contacts")
    if not isinstance(terminal, dict) or set(terminal) != set(expected_contacts):
        raise ValueError(
            "resume stage terminal contact keys mismatch: " + stage["name"]
        )
    for leg_name, surface in expected_contacts.items():
        state = terminal[leg_name]
        analytic = (
            state.get("analytic_sphere_hit_classification")
            if isinstance(state, dict)
            else None
        )
        footpad = (
            state.get("visual_footpad_hit_classification")
            if isinstance(state, dict)
            else None
        )
        if (
            not isinstance(state, dict)
            or state.get("surface") != surface
            or state.get("valid") is not True
            or state.get(
                "visual_footpad_component_contact_classified_not_ignored"
            ) is not True
            or not isinstance(analytic, dict)
            or analytic.get(
                "all_hit_facets_match_planned_support_surface"
            ) is not True
            or not isinstance(footpad, dict)
            or footpad.get(
                "all_hit_facets_match_planned_support_surface"
            ) is not True
        ):
            raise ValueError(
                "resume stage terminal contact classification failed: "
                + stage["name"]
                + "/"
                + leg_name
            )


def _prepare_resume_artifact(trace, stages):
    """检查并复制可继续执行的阶段记录。

    参数:
        trace: 已生成的轨迹记录。
        stages: 当前固定阶段序列。

    返回:
        可用于继续检查的记录副本。
    """
    prepared = copy.deepcopy(trace)
    if prepared.get("status") == "PARTIAL_STAGE_COMPLETE":
        return prepared
    if prepared.get("status") != "BLOCKED":
        raise ValueError("resume accepts only PARTIAL or one safe BLOCKED retry")

    records = prepared.get("stages")
    blocked = prepared.get("blocked")
    if (
        not isinstance(records, list)
        or len(records) < 2
        or len(records) > len(stages)
        or not isinstance(blocked, dict)
        or set(blocked) != {"stage", "reason", "earliest_witness"}
    ):
        raise ValueError("resume BLOCKED receipt structure mismatch")
    clear_count = len(records) - 1
    for index, record in enumerate(records[:clear_count]):
        if (
            not isinstance(record, dict)
            or record.get("name") != stages[index]["name"]
            or record.get("clear") is not True
        ):
            raise ValueError("resume BLOCKED clear prefix mismatch")
    failed = records[-1]
    next_stage = stages[clear_count]
    if (
        not isinstance(failed, dict)
        or failed.get("clear") is not False
        or failed.get("name") != next_stage["name"]
        or blocked.get("stage") != next_stage["name"]
        or blocked.get("reason") != failed.get("reason")
        or blocked.get("earliest_witness") != failed.get("witness")
    ):
        raise ValueError("resume BLOCKED failed attempt mismatch")
    expected_nodes = {"PAIR_SOURCE", *STAGE_NAMES[:clear_count]}
    if not isinstance(prepared.get("nodes"), dict) or set(
        prepared["nodes"]
    ) != expected_nodes:
        raise ValueError("resume BLOCKED nodes exceed the clear prefix")
    history = prepared.get("failed_attempt_history", [])
    if not isinstance(history, list):
        raise ValueError("resume failed attempt history must be a list")
    expected_targets = _fixed_targets(stages)
    expected_geometry = _fixed_stage_trajectory_geometry(stages)
    if next_stage["name"] == "LF_C1" and (
        _json_value(prepared.get("fixed_targets"))
        != _json_value(expected_targets)
        or _json_value(prepared.get("fixed_stage_trajectory_geometry"))
        != _json_value(expected_geometry)
    ):
        legacy_targets = copy.deepcopy(expected_targets)
        legacy_pose = np.asarray(
            legacy_targets["LF_C1"]["pose_world"], dtype=np.float64
        )
        legacy_pose[2] = 0.18
        legacy_pose[3] = 0.15
        legacy_targets["LF_C1"]["pose_world"] = legacy_pose.tolist()
        legacy_geometry = copy.deepcopy(expected_geometry)
        legacy_geometry.pop("LF_C1")
        legacy_witness = {
            "sample": 58,
            "self_hit": None,
            "xiaolan_hit": {
                "left_link": "body",
                "left_leg": None,
                "left_component": 0,
                "left_triangle": 25554,
                "right_link": "xiaolan",
                "right_leg": None,
                "right_component": 0,
                "right_triangle": 17896,
            },
        }
        if (
            _json_value(prepared.get("fixed_targets"))
            != _json_value(legacy_targets)
            or _json_value(
                prepared.get("fixed_stage_trajectory_geometry")
            )
            != _json_value(legacy_geometry)
            or failed.get("reason") != "visual_stl"
            or failed.get("visual_sample_count") != 59
            or failed.get("witness") != legacy_witness
        ):
            raise ValueError("resume legacy LF_C1 artifact mismatch")
        prepared["fixed_targets"] = expected_targets
        prepared["fixed_stage_trajectory_geometry"] = expected_geometry
    if "fixed_stage_trajectory_geometry" not in prepared:
        if next_stage["name"] != "RM_B":
            raise ValueError("resume artifact lacks fixed trajectory geometry")
        prepared["fixed_stage_trajectory_geometry"] = (
            expected_geometry
        )
    prepared["failed_attempt_history"] = history + [
        {"stage_record": failed, "blocked": blocked}
    ]
    prepared["stages"] = records[:clear_count]
    prepared.pop("blocked")
    prepared["status"] = "PARTIAL_STAGE_COMPLETE"
    prepared["stop_after"] = stages[clear_count - 1]["name"]
    return prepared


def _resume_preflight(
    trace,
    requested_stop_after,
    source_bytes,
    scene,
    stages,
    model,
):
    if trace.get("status") != "PARTIAL_STAGE_COMPLETE":
        raise ValueError("resume requires PARTIAL_STAGE_COMPLETE status")
    if "blocked" in trace:
        raise ValueError("resume artifact must not contain blocked")
    contract = _trace_contract(source_bytes, scene, stages)
    for key, expected in contract.items():
        if _json_value(trace.get(key)) != _json_value(expected):
            raise ValueError("resume artifact contract mismatch: " + key)

    records = trace.get("stages")
    if not isinstance(records, list) or not records or len(records) > len(stages):
        raise ValueError("resume stages must be a non-empty fixed prefix")
    for index, record in enumerate(records):
        if (
            not isinstance(record, dict)
            or record.get("name") != stages[index]["name"]
            or record.get("clear") is not True
        ):
            raise ValueError("resume stages are not a strict clear prefix")
        _audit_completed_stage_record(record, stages[index])
    completed_count = len(records)
    last_stage = stages[completed_count - 1]
    last_name = last_stage["name"]
    if trace.get("stop_after") != last_name:
        raise ValueError("resume stop_after does not equal the last stage")
    requested_index = (
        len(stages)
        if requested_stop_after == "FULL"
        else STAGE_NAMES.index(requested_stop_after)
    )
    if requested_index <= completed_count - 1:
        raise ValueError("resume stop_after must be strictly later")

    nodes = trace.get("nodes")
    expected_nodes = {"PAIR_SOURCE", *STAGE_NAMES[:completed_count]}
    if not isinstance(nodes, dict) or set(nodes) != expected_nodes:
        raise ValueError("resume nodes do not match the completed prefix")
    node = nodes[last_name]
    pose = np.asarray(node.get("pose_world"), dtype=np.float64)
    q_value = np.asarray(node.get("q_rad"), dtype=np.float64)
    anchors = np.asarray(node.get("anchors_world_m"), dtype=np.float64)
    if pose.shape != (5,) or q_value.shape != (6, 3) or anchors.shape != (6, 3):
        raise ValueError("resume endpoint node has invalid shapes")
    if not (
        np.all(np.isfinite(pose))
        and np.all(np.isfinite(q_value))
        and np.all(np.isfinite(anchors))
    ):
        raise ValueError("resume endpoint node must be finite")
    if (
        not np.allclose(pose, last_stage["pose_end"], rtol=0.0, atol=1e-12)
        or not np.allclose(
            anchors,
            last_stage["anchor_knots"][-1],
            rtol=0.0,
            atol=1e-12,
        )
    ):
        raise ValueError("resume endpoint differs from the fixed stage")
    if (
        not np.allclose(node.get("base_world"), pose[:3], rtol=0.0, atol=1e-12)
        or abs(float(node.get("roll_rad")) - pose[3]) > 1e-12
        or abs(float(node.get("pitch_rad")) - pose[4]) > 1e-12
        or node.get("rotation_order") != "T_Ry_pitch_Rx_roll"
    ):
        raise ValueError("resume node pose fields disagree")
    expected_contacts = {
        LEG_NAMES[leg]: surface
        for leg, surface in last_stage["contacts_end"].items()
    }
    if node.get("planned_contacts") != expected_contacts:
        raise ValueError("resume planned contacts differ from the fixed stage")
    fk_world = world_points(
        pose_matrix(pose), model.forward_base(q_value)
    )
    if float(np.max(np.abs(fk_world - anchors))) > 1e-6:
        raise ValueError("resume endpoint FK does not match anchors")
    prior_elapsed = float(trace.get("planner_elapsed_s", -1.0))
    if not np.isfinite(prior_elapsed) or prior_elapsed < 0.0:
        raise ValueError("resume planner elapsed time is invalid")
    state = {
        "q": q_value.copy(),
        "pose": pose.copy(),
        "anchors": anchors.copy(),
        "contacts": dict(last_stage["contacts_end"]),
    }
    return state, completed_count, prior_elapsed


def build_trace(output_path=OUTPUT_PATH, stop_after="FULL", resume=False):
    started = time.monotonic()
    source_bytes = SOURCE_TRACE_PATH.read_bytes()
    source = json.loads(source_bytes)
    if source.get("status") != "MODEL_PATH_FOUND" or source["nodes"].get("PAIR") is None:
        raise ValueError("source climb_trace must contain a successful PAIR node")
    config = json.loads(CONFIG_PATH.read_text())
    _, extraction, normal, selector_ids = front.selector_geometry(config)
    low_triangle_ids = frozenset(int(index) for index in selector_ids)
    scene = visual.default_visual_scene(ROOT, WORLD_FROM_XIAOLAN)
    visual.require_ankle_visual_footpad_binding(
        scene.link_meshes["ankle"]
    )
    top = _top_triangles(scene)
    top_triangle_ids = _top_triangle_ids(scene)
    model = k.GraspKinematic()
    controller = control.GraspController(0.02, enable_link_collision_check=True)
    stages = fixed_stages(source, extraction, normal)
    resume_entry = None
    if resume:
        if not output_path.is_file():
            raise ValueError("resume output artifact does not exist")
        trace = _prepare_resume_artifact(
            json.loads(output_path.read_text()), stages
        )
        state, start_index, prior_elapsed = _resume_preflight(
            trace, stop_after, source_bytes, scene, stages, model
        )
        resume_entry = {
            "from_stage": stages[start_index - 1]["name"],
            "prior_elapsed_s": prior_elapsed,
            "added_elapsed_s": 0.0,
        }
        trace["resume_history"] = list(trace.get("resume_history", ())) + [
            resume_entry
        ]
        trace.pop("result", None)
    else:
        pair = source["nodes"]["PAIR"]
        initial_pose = np.array((*pair["base_world"], 0.0, pair["pitch_rad"]))
        state = {
            "q": np.asarray(pair["q_rad"], dtype=np.float64),
            "pose": initial_pose,
            "anchors": np.asarray(pair["anchors_world_m"], dtype=np.float64),
            "contacts": {RB: "low", RF: "low", RM: "low"},
        }
        trace = {
            **_trace_contract(source_bytes, scene, stages),
            "status": "BLOCKED",
            "stop_after": stop_after,
            "nodes": {"PAIR_SOURCE": _node(state)},
            "stages": [],
            "planner_elapsed_s": 0.0,
        }
        start_index = 0
        prior_elapsed = 0.0
        _atomic_write(output_path, trace)

    requested_stop_after = stop_after
    for stage in stages[start_index:]:
        new_state, record = evaluate_stage(
            scene,
            model,
            controller,
            state,
            stage,
            extraction,
            normal,
            top,
            low_triangle_ids,
            top_triangle_ids,
        )
        trace["stages"].append(record)
        added_elapsed = time.monotonic() - started
        trace["planner_elapsed_s"] = prior_elapsed + added_elapsed
        if resume_entry is not None:
            resume_entry["added_elapsed_s"] = added_elapsed
        if new_state is None:
            trace["status"] = "BLOCKED"
            trace["stop_after"] = requested_stop_after
            trace["blocked"] = {
                "stage": stage["name"],
                "reason": record["reason"],
                "earliest_witness": record["witness"],
            }
            _atomic_write(output_path, trace)
            print(
                "{} leaves={} elapsed={:.3f}s BLOCKED".format(
                    stage["name"], record.get("visual_sample_count", 0),
                    record["elapsed_s"],
                ),
                flush=True,
            )
            return _json_value(trace)
        state = new_state
        trace["nodes"][stage["name"]] = _node(state)
        print(
            "{} leaves={} elapsed={:.3f}s PASS".format(
                stage["name"], record["visual_sample_count"], record["elapsed_s"]
            ),
            flush=True,
        )
        trace["status"] = "PARTIAL_STAGE_COMPLETE"
        trace["stop_after"] = stage["name"]
        trace["result"] = (
            stage["name"]
            + " complete; later fixed stages not evaluated"
        )
        added_elapsed = time.monotonic() - started
        trace["planner_elapsed_s"] = prior_elapsed + added_elapsed
        if resume_entry is not None:
            resume_entry["added_elapsed_s"] = added_elapsed
        _atomic_write(output_path, trace)
        if requested_stop_after == stage["name"]:
            return _json_value(trace)
    trace["status"] = "MODEL_PATH_FOUND_BACK_HALF"
    trace["stop_after"] = "FULL"
    trace["result"] = (
        "all fixed back-half verification leaves passed the visual-CAD model; "
        "real contact, load, friction, and clearance remain unresolved"
    )
    added_elapsed = time.monotonic() - started
    trace["planner_elapsed_s"] = prior_elapsed + added_elapsed
    if resume_entry is not None:
        resume_entry["added_elapsed_s"] = added_elapsed
    _atomic_write(output_path, trace)
    return _json_value(trace)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=OUTPUT_PATH)
    parser.add_argument(
        "--stop-after", choices=STAGE_NAMES + ("FULL",), default="FULL"
    )
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args(argv)
    trace = build_trace(args.output, args.stop_after, args.resume)
    print(
        "{}: {} ({:.3f}s)".format(
            trace["status"], trace.get("blocked", {}).get("stage", ""),
            trace["planner_elapsed_s"],
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
