#!/usr/bin/env python3
"""Generate the numeric-only D4/M2 simulation-candidate suffix.

The strict back trace remains the authority for its dense-clear prefix through
B1.  This helper reuses the fixed back planner's D4/M2 stages and adaptive IK
leaves, but deliberately defers their exact visual validation to Isaac.  It is
therefore a simulation candidate, never a strict MODEL_PATH_FOUND receipt.
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

from utils import climb_collision as visual
from utils import climb
import control
import kinematics as k
from tools import plan_climb_back_trajectory as back
from tools import plan_climb_trajectory as front


ROOT = SCRIPTS_DIR.parents[2]
STRICT_TRACE_PATH = (
    ROOT / "src/grasp_hexapod_control/config/climb_back_trace.json"
)
OUTPUT_PATH = (
    ROOT / "src/grasp_hexapod_control/config/climb_back_sim_trace.json"
)
COMPACT_PATH = ROOT / "src/grasp_hexapod_control/config/climb_compact.json"
SIM_SETTLE_MAX_FOOT_TARGET_ERROR_M = 0.010
SIM_STAGE_NAMES = ("D4_LB_LOW", "M2_LM_LOW")
SIM_CONTACT_ENGINEERING_TOLERANCE_M = 0.001
PREVIEW_SUFFIX_NAMES = (
    "RM_RIGHT_SYMMETRY",
    "LM_VERTICAL_PREP",
    "LB_LF_CLEARANCE_RAISE",
    "LB_LF_LOW_LAND_PAIR",
    "LM_LOW_LAND_LAST",
    "FINAL_HOLD",
    "BODY_RIGHT_LOAD_PREP",
    "TRIPOD_A_RIGHT_STEP",
    "TRIPOD_B_RIGHT_STEP",
    "BODY_RIGHT_SETTLE",
    "LM_POSTURE_RESET",
    "RB_RF_TOP_ENTRY",
    "TRIPOD_A_TOP_APPROACH",
    "LM_FINAL_POSTURE_WITH_BODY",
    "STAND_FINAL_HOLD",
)

# 这两次查询发生在采用仅数值生成器之前。保留其精确来源作为诊断，
# 不重跑，也不升级为全部叶节点的视觉证明。
DEFERRED_VISUAL_DIAGNOSTICS = (
    {
        "stage": "D4_LB_LOW",
        "time": 0.5,
        "outcome": "EXACT_VISUAL_CLEAR",
        "elapsed_s": 27.74694001500029,
        "not_rechecked_in_this_run": True,
    },
    {
        "stage": "D4_LB_LOW",
        "time": 0.95,
        "outcome": "INTENDED_LOW_FOOTPAD_CONTACT_DIAGNOSTIC",
        "leg": "lb",
        "visual_component": "ankle_component_10_physical_footpad",
        "xiaolan_triangle_ids": [17160],
        "plane_error_m": 5.6237583369644906e-05,
        "polygon_inside": True,
        "boundary_distance_m": 0.021953271189548096,
        "within_sim_contact_engineering_tolerance": True,
        "elapsed_s": 30.694681619999756,
        "not_rechecked_in_this_run": True,
    },
)


def _support_margin(model, sample, support_legs):
    world_com = back.world_points(
        sample["transform"], model.center_of_mass_base(sample["q_rad"])[None]
    )[0]
    support = climb.gravity_projected_support(
        world_com,
        sample["anchors"][list(support_legs)],
        np.array((0.0, 0.0, -9.81)),
    )
    return float(support.raw_margin_m)


def _analytic_false_runs(controller, model, samples):
    observations = []
    for index, sample in enumerate(samples):
        q_value = sample["q_rad"]
        link_clear = controller._link_collision_free(q_value)
        foot_clear = controller._foot_collision_free(
            model.forward_base(q_value)
        )
        observations.append(
            {
                "sample_index": index,
                "time": sample["t"],
                "clear": bool(np.all(link_clear) and np.all(foot_clear)),
                "link_collision_free": link_clear.tolist(),
                "foot_collision_free": foot_clear.tolist(),
            }
        )
    runs = []
    index = 0
    while index < len(observations):
        if observations[index]["clear"]:
            index += 1
            continue
        first = index
        while (
            index + 1 < len(observations)
            and not observations[index + 1]["clear"]
        ):
            index += 1
        last = index
        runs.append(
            {
                "first": observations[first],
                "middle": observations[(first + last) // 2],
                "last": observations[last],
            }
        )
        index += 1
    return runs


def _numeric_stage(model, controller, state, stage):
    started = time.monotonic()
    try:
        samples = back._adaptive_samples(model, state["q"], stage)
    except (ValueError, RecursionError) as exc:
        return None, {
            "name": stage["name"],
            "clear": False,
            "reason": "adaptive_sample_cap",
            "witness": {"detail": str(exc), "sample_cap": back.MAX_SAMPLES},
            "elapsed_s": time.monotonic() - started,
        }
    metrics = back._new_metrics()
    previous = None
    for index, sample in enumerate(samples):
        sample["stage_active_legs"] = stage["active_legs"]
        q_value = sample["q_rad"]
        margin = float(np.min(model.joint_limit_margins(q_value)))
        support = _support_margin(model, sample, stage["support_legs"])
        back._update_metrics(metrics, model, sample, previous, support)
        if (
            sample["residual_m"] > back.MAX_RESIDUAL_M
            or margin < back.MIN_JOINT_MARGIN_RAD
            or support < back.MIN_SUPPORT_MARGIN_M
        ):
            return None, {
                "name": stage["name"],
                "clear": False,
                "reason": "numeric_or_support_gate",
                "witness": {
                    "sample": index,
                    "time": sample["t"],
                    "residual_m": sample["residual_m"],
                    "joint_margin_rad": margin,
                    "support_margin_m": support,
                },
                "numeric_leaf_count": len(samples),
                "metrics": metrics,
                "elapsed_s": time.monotonic() - started,
            }
        previous = sample

    maximum_metrics = {
        "max_ik_residual_m": back.MAX_RESIDUAL_M,
        "max_adjacent_joint_delta_rad": back.MAX_JOINT_STEP_RAD,
        "max_active_foot_step_m": back.MAX_FOOT_STEP_M,
        "max_base_translation_step_m": back.MAX_BASE_STEP_M,
        "max_roll_step_rad": back.MAX_ANGLE_STEP_RAD,
        "max_pitch_step_rad": back.MAX_ANGLE_STEP_RAD,
    }
    if any(
        metrics[name] > limit + 1e-12
        for name, limit in maximum_metrics.items()
    ):
        return None, {
            "name": stage["name"],
            "clear": False,
            "reason": "adaptive_step_gate",
            "witness": {"metrics": metrics, "limits": maximum_metrics},
            "numeric_leaf_count": len(samples),
            "metrics": metrics,
            "elapsed_s": time.monotonic() - started,
        }

    analytic_runs = _analytic_false_runs(controller, model, samples)
    endpoint = samples[-1]
    new_state = {
        "q": endpoint["q_rad"].copy(),
        "pose": endpoint["pose"].copy(),
        "anchors": endpoint["anchors"].copy(),
        "contacts": dict(stage["contacts_end"]),
    }
    return new_state, {
        "name": stage["name"],
        "clear": True,
        "active_legs": [back.LEG_NAMES[leg] for leg in stage["active_legs"]],
        "support_legs": [back.LEG_NAMES[leg] for leg in stage["support_legs"]],
        "minimum_samples": stage["minimum_samples"],
        "numeric_leaf_count": len(samples),
        "verification_leaves_are_not_optimized_poses": True,
        "numeric_hard_gates_all_leaves": True,
        "analytic_guard_is_diagnostic_not_path_authority": True,
        "analytic_guard_all_clear": not analytic_runs,
        "analytic_guard_false_runs": analytic_runs,
        "dense_visual_all_leaves": False,
        "visual_validation_deferred_to_isaac": True,
        "metrics": metrics,
        "pose_start": stage["pose_start"].tolist(),
        "pose_end": stage["pose_end"].tolist(),
        "anchor_knots_world_m": stage["anchor_knots"].tolist(),
        "q_end_rad": endpoint["q_rad"].tolist(),
        "elapsed_s": time.monotonic() - started,
    }


def build_sim_finish(output_path=OUTPUT_PATH):
    started = time.monotonic()
    strict_bytes = STRICT_TRACE_PATH.read_bytes()
    strict = json.loads(strict_bytes)
    source_bytes = back.SOURCE_TRACE_PATH.read_bytes()
    source = json.loads(source_bytes)
    config = json.loads(back.CONFIG_PATH.read_text())
    _, extraction, normal, _ = front.selector_geometry(config)
    scene = visual.default_visual_scene(ROOT, back.WORLD_FROM_XIAOLAN)
    visual.require_ankle_visual_footpad_binding(scene.link_meshes["ankle"])
    model = k.GraspKinematic()
    controller = control.GraspController(
        0.02, enable_link_collision_check=True
    )
    stages = back.fixed_stages(source, extraction, normal)
    prepared = back._prepare_resume_artifact(strict, stages)
    state, start_index, strict_elapsed = back._resume_preflight(
        prepared,
        SIM_STAGE_NAMES[-1],
        source_bytes,
        scene,
        stages,
        model,
    )
    selected = stages[start_index : start_index + len(SIM_STAGE_NAMES)]
    if (
        strict.get("status") != "PARTIAL_STAGE_COMPLETE"
        or strict.get("stop_after") != "B1"
        or start_index != back.STAGE_NAMES.index(SIM_STAGE_NAMES[0])
        or tuple(stage["name"] for stage in selected) != SIM_STAGE_NAMES
    ):
        raise ValueError("sim finish requires the strict clear B1 prefix")

    trace = {
        "schema": "SIMULATION_ONLY_BACK_HALF_FAST_FINISH_V1",
        "status": "BLOCKED",
        "simulation_candidate_only": True,
        "dense_visual_all_leaves": False,
        "visual_validation_deferred_to_isaac": True,
        "strict_prefix_stop": "B1",
        "source_strict_trace": {
            "path": STRICT_TRACE_PATH.name,
            "sha256": hashlib.sha256(strict_bytes).hexdigest(),
            "status": strict["status"],
            "stop_after": strict["stop_after"],
            "planner_elapsed_s_diagnostic": strict_elapsed,
        },
        "source_front_trace_sha256": hashlib.sha256(source_bytes).hexdigest(),
        "configured_stage_order": list(SIM_STAGE_NAMES),
        "rotation_order": "T_Ry_pitch_Rx_roll",
        "numeric_thresholds": back._trace_contract(
            source_bytes, scene, stages
        )["thresholds"],
        "sim_contact_engineering_tolerance_m": (
            SIM_CONTACT_ENGINEERING_TOLERANCE_M
        ),
        "controller_analytic_guards_diagnostic_only": True,
        "URDF_COLLISION_GEOMETRY_NOT_USED_AS_VISUAL_AUTHORITY": True,
        "model_only_not_contact_or_load_proof": True,
        "REAL_CLEARANCE_UNRESOLVED": True,
        "deferred_visual_diagnostics": list(DEFERRED_VISUAL_DIAGNOSTICS),
        "fixed_endpoints": {
            stage["name"]: {
                "pose_world": stage["pose_end"].tolist(),
                "anchors_world_m": stage["anchor_knots"][-1].tolist(),
                "anchor_knots_world_m": stage["anchor_knots"].tolist(),
            }
            for stage in selected
        },
        "nodes": {"B1": strict["nodes"]["B1"]},
        "stages": [],
    }
    for stage in selected:
        new_state, record = _numeric_stage(model, controller, state, stage)
        trace["stages"].append(record)
        if new_state is None:
            trace["status"] = "BLOCKED"
            trace["blocked"] = {
                "stage": stage["name"],
                "reason": record["reason"],
                "earliest_witness": record["witness"],
            }
            trace["planner_elapsed_s"] = time.monotonic() - started
            back._atomic_write(output_path, trace)
            return back._json_value(trace)
        state = new_state
        trace["nodes"][stage["name"]] = back._node(state)

    trace["status"] = "SIM_CANDIDATE_PATH_FOUND"
    trace["result"] = (
        "D4/M2 adaptive numeric and geometry-support gates passed; exact "
        "visual validation is deferred to Isaac and no strict path is claimed"
    )
    trace["planner_elapsed_s"] = time.monotonic() - started
    back._atomic_write(output_path, trace)
    return back._json_value(trace)


def _pose_from_front_node(node):
    return [*node["base_world"], 0.0, node["pitch_rad"]]


def _front_compact_stages(front_trace, legacy):
    p0 = legacy["p0"]
    prep = legacy["prep"]
    rm = legacy["rm"]
    body = legacy["body"]
    pair = legacy["pair"]
    durations = legacy["durations_s"]
    settle = legacy["settle_gate"]["persistence_s"]
    p0_pose = [*p0["base"][:3], 0.0, p0["base"][3]]
    prep_pose = [*prep["base"][:3], 0.0, prep["base"][3]]
    body_pose = [*body["base"][:3], 0.0, body["base"][3]]
    p0_anchors = np.asarray(p0["anchors_world_m"], dtype=np.float64)
    rm_knots = []
    for waypoint in rm["waypoints_world_m"]:
        anchors = p0_anchors.copy()
        anchors[rm["leg_index"]] = waypoint
        rm_knots.append(anchors.tolist())
    rm_end = np.asarray(rm_knots[-1], dtype=np.float64)
    pair_knots = []
    for waypoints in pair["waypoints_world_m"]:
        anchors = rm_end.copy()
        anchors[pair["leg_indices"]] = waypoints
        pair_knots.append(anchors.tolist())
    def stage(
        name,
        pose_start,
        pose_end,
        anchor_knots,
        active_legs,
        segment_durations,
    ):
        return {
            "name": name,
            "pose_start": pose_start,
            "pose_end": pose_end,
            "anchor_knots": anchor_knots,
            "active_legs": active_legs,
            "segment_durations_s": segment_durations,
            "pose_curve": "quintic_full_stage",
            "anchor_curve": "piecewise_quintic",
            "settle_s": settle,
        }

    return [
        stage(
            "PREP",
            p0_pose,
            prep_pose,
            [p0_anchors.tolist(), p0_anchors.tolist()],
            [],
            [durations["PREP"]],
        ),
        stage(
            "RM",
            prep_pose,
            prep_pose,
            rm_knots,
            [rm["leg_index"]],
            list(durations["RM"]),
        ),
        stage(
            "BODY",
            prep_pose,
            body_pose,
            [rm_end.tolist(), rm_end.tolist()],
            [],
            [durations["BODY"]],
        ),
        stage(
            "PAIR",
            body_pose,
            body_pose,
            pair_knots,
            list(pair["leg_indices"]),
            list(durations["PAIR"]),
        ),
    ]


def _preview_back_stages(fixed_stages, low_surface_normal_local):
    """以仿真预览的 pair-first 序列替换 C21-C26。"""
    fixed = list(fixed_stages)
    by_name = {stage["name"]: stage for stage in fixed}
    rm_b_index = next(
        index for index, stage in enumerate(fixed) if stage["name"] == "RM_B"
    )
    stages = fixed[:rm_b_index]
    source = fixed[rm_b_index]
    pose = source["pose_start"].copy()
    anchors = source["anchor_knots"][0].copy()
    contacts = dict(source["contacts_start"])

    def add(stage):
        nonlocal pose, anchors, contacts
        stages.append(stage)
        pose = stage["pose_end"].copy()
        anchors = stage["anchor_knots"][-1].copy()
        contacts = dict(stage["contacts_end"])

    # C21：保持外侧 pair 近似镜像，并将 RM 放在 y 方向中点。
    rm_middle = np.array((0.465, -0.105, 0.2065), dtype=np.float64)
    rm_end = np.array((0.51, -0.08, 0.1965), dtype=np.float64)
    middle = anchors.copy()
    target = anchors.copy()
    middle[back.RM] = rm_middle
    target[back.RM] = rm_end
    end_contacts = dict(contacts)
    end_contacts[back.RM] = "top"
    add(back._make_stage(
        "RM_RIGHT_SYMMETRY",
        pose,
        pose,
        (anchors.copy(), middle, target),
        (back.RM,),
        (back.LB, back.LF, back.LM, back.RB, back.RF),
        contacts,
        end_contacts,
        31,
    ))

    old_lm = by_name["LM_LOW_LAND_FIRST"]
    old_lf = by_name["LF_LOW_LAND_SECOND"]
    old_lb = by_name["LB_LOW_LAND_LAST"]
    lm_goal = old_lm["anchor_knots"][-1, back.LM].copy()
    lf_goal = old_lf["anchor_knots"][-1, back.LF].copy()
    lb_goal = old_lb["anchor_knots"][-1, back.LB].copy()

    # C22：外侧 pair 离地前，将 LM 放到其髋关节正下方。
    lm_hip = back.world_points(
        back.pose_matrix(pose), k.HIP_XYZ[[back.LM]]
    )[0]
    lm_ground = np.array((lm_hip[0], pose[1], k.FOOT_RADIUS))
    add(back._swing_stage(
        "LM_VERTICAL_PREP",
        pose,
        anchors,
        (lm_ground,),
        (back.LM,),
        (back.LB, back.LF, back.RB, back.RF, back.RM),
        contacts,
        (None,),
        0.03,
    ))

    # C23：机身获得净空时，同时抬起两条外侧腿。
    normal_world = (
        back.WORLD_FROM_XIAOLAN[:3, :3]
        @ np.asarray(low_surface_normal_local, dtype=np.float64)
    )
    low_slope_dz_dx = -normal_world[0] / normal_world[2]
    clearance = 0.010
    clear_start = anchors.copy()
    for leg, goal in ((back.LB, lb_goal), (back.LF, lf_goal)):
        clear_start[leg, 2] = (
            goal[2]
            + low_slope_dz_dx * (anchors[leg, 0] - goal[0])
            + clearance
        )
    raised_pose = np.array((0.23, pose[1], 0.22, 0.0, -0.20))
    add(back._make_stage(
        "LB_LF_CLEARANCE_RAISE",
        pose,
        raised_pose,
        (anchors.copy(), clear_start),
        (back.LB, back.LF),
        (back.LM, back.RB, back.RF, back.RM),
        contacts,
        contacts,
        61,
    ))

    # C24：沿低面平行移动，再让 pair 同时落足。
    hover = anchors.copy()
    landed = anchors.copy()
    hover[back.LB] = lb_goal + np.array((0.0, 0.0, clearance))
    hover[back.LF] = lf_goal + np.array((0.0, 0.0, clearance))
    landed[back.LB] = lb_goal
    landed[back.LF] = lf_goal
    pair_contacts = dict(contacts)
    pair_contacts[back.LB] = "low"
    pair_contacts[back.LF] = "low"
    add(back._make_stage(
        "LB_LF_LOW_LAND_PAIR",
        pose,
        pose,
        (anchors.copy(), hover, landed),
        (back.LB, back.LF),
        (back.LM, back.RB, back.RF, back.RM),
        contacts,
        pair_contacts,
        61,
    ))

    # C25：pair 落足后 LM 才离地。LM 腾空时抬高并前移机身，使其以可用姿态
    # 接近低面，而不是在固定低机身下折叠。
    lm_land_pose = pose + np.array((0.03, 0.0, 0.06, 0.0, 0.0))
    add(back._swing_stage(
        "LM_LOW_LAND_LAST",
        pose,
        anchors,
        (lm_goal,),
        (back.LM,),
        (back.LB, back.LF, back.RB, back.RF, back.RM),
        contacts,
        ("low",),
        0.04,
        pose_end=lm_land_pose,
    ))
    add(back._body_stage(
        "FINAL_HOLD", pose, pose.copy(), anchors, tuple(range(6)), contacts
    ))

    # 六足全部上台后执行一个普通向右行走周期。
    walk_prep_pose = pose + np.array((0.05, 0.0, -0.01, 0.0, 0.0))
    add(back._body_stage(
        "BODY_RIGHT_LOAD_PREP",
        pose,
        walk_prep_pose,
        anchors,
        tuple(range(6)),
        contacts,
    ))

    tripod_a_targets = []
    for leg, distance in ((back.LB, 0.03), (back.LF, 0.03), (back.RM, 0.02)):
        point = anchors[leg].copy()
        point[0] += distance
        if leg in (back.LB, back.LF):
            point[2] += low_slope_dz_dx * distance
        tripod_a_targets.append(point)
    add(back._swing_stage(
        "TRIPOD_A_RIGHT_STEP",
        pose,
        anchors,
        tripod_a_targets,
        (back.LB, back.LF, back.RM),
        (back.LM, back.RB, back.RF),
        contacts,
        ("low", "low", "top"),
        0.03,
    ))

    tripod_b_targets = []
    for leg, distance in ((back.LM, 0.03), (back.RB, 0.04), (back.RF, 0.04)):
        point = anchors[leg].copy()
        point[0] += distance
        if leg == back.LM:
            point[2] += low_slope_dz_dx * distance
        tripod_b_targets.append(point)
    add(back._swing_stage(
        "TRIPOD_B_RIGHT_STEP",
        pose,
        anchors,
        tripod_b_targets,
        (back.LM, back.RB, back.RF),
        (back.LB, back.LF, back.RM),
        contacts,
        ("low", "top", "top"),
        0.03,
    ))

    centered_pose = pose + np.array((0.01, 0.0, 0.0, 0.0, 0.0))
    add(back._body_stage(
        "BODY_RIGHT_SETTLE",
        pose,
        centered_pose,
        anchors,
        tuple(range(6)),
        contacts,
    ))

    # 继续执行三个协调 tripod 步。结点路径来自连续关节插值，使足端随机身进入
    # 紧凑的顶面工作空间，而不跨越 thigh 关节分支。
    preview_model = k.GraspKinematic()
    preview_q, residual = front.solve_anchors(
        preview_model,
        anchors,
        back.pose_matrix(pose),
        k.Q_STAND,
        1e-9,
    )
    if residual > 1e-4:
        raise ValueError("preview continuation start IK failed")

    def guided_tripod(name, legs, target, pose_end, knot_count, lift_m):
        nonlocal preview_q
        q_start, start_residual = front.solve_anchors(
            preview_model,
            anchors,
            back.pose_matrix(pose),
            preview_q,
            1e-9,
        )
        target_anchors = anchors.copy()
        target_anchors[list(legs)] = target[list(legs)]
        q_end, end_residual = front.solve_anchors(
            preview_model,
            target_anchors,
            back.pose_matrix(pose_end),
            q_start,
            1e-9,
        )
        if max(start_residual, end_residual) > 1e-4:
            raise ValueError(name + " endpoint IK failed")

        knots = []
        for index in range(knot_count + 1):
            if index == 0:
                knots.append(anchors.copy())
                continue
            if index == knot_count:
                knots.append(target_anchors.copy())
                continue
            weight = front.smoothstep(index / knot_count)
            knot_pose = pose * (1.0 - weight) + pose_end * weight
            knot_q = q_start.copy()
            knot_q[list(legs)] = (
                q_start[list(legs)] * (1.0 - weight)
                + q_end[list(legs)] * weight
            )
            knot = anchors.copy()
            knot[list(legs)] = back.world_points(
                back.pose_matrix(knot_pose),
                preview_model.forward_base(knot_q),
            )[list(legs)]
            knot[list(legs), 2] += lift_m * 4.0 * weight * (1.0 - weight)
            knots.append(knot)

        end_contacts = dict(contacts)
        for leg in legs:
            end_contacts[leg] = "top"
        stage = back._make_stage(
            name,
            pose,
            pose_end,
            knots,
            legs,
            tuple(leg for leg in range(6) if leg not in legs),
            contacts,
            end_contacts,
            knot_count * 8 + 1,
        )
        stage["preview_segment_duration_s"] = 0.25
        preview_q = q_end
        return stage

    # C31：其余支撑和机身固定时重置 LM。明确的四个结点保证可见顺序无歧义：
    # 垂直抬起、在低面上方向外张开、再将足端放回低面。
    lm_reset = anchors.copy()
    lm_reset[back.LM, 0] -= 0.040
    lm_reset[back.LM, 1] = -0.070
    lm_reset[back.LM, 2] += low_slope_dz_dx * -0.040
    lm_reset_lift = anchors.copy()
    lm_reset_lift[back.LM, 2] += 0.030
    lm_reset_hover = lm_reset.copy()
    lm_reset_hover[back.LM, 2] += 0.030
    add(back._make_stage(
        "LM_POSTURE_RESET",
        pose,
        pose,
        (anchors.copy(), lm_reset_lift, lm_reset_hover, lm_reset),
        (back.LM,),
        (back.LB, back.LF, back.RB, back.RF, back.RM),
        contacts,
        contacts,
        31,
    ))
    stages[-1]["preview_segment_duration_s"] = 0.25

    # C32：仅将右侧外 pair 转移到顶面。LM 保持重置后的低面姿态，不被带入旧
    # tripod 动作。
    top_entry = anchors.copy()
    top_entry[back.RB] = (0.518, -0.159, 0.1965)
    top_entry[back.RF] = (0.530, 0.113, 0.1965)
    add(guided_tripod(
        "RB_RF_TOP_ENTRY",
        (back.RB, back.RF),
        top_entry,
        np.array((0.36, -0.030, 0.28, 0.0, -0.12)),
        16,
        0.030,
    ))

    # C33：外侧左 tripod 落到最终顶面接触点，LM 保持重置后的低面接触。只有
    # 机身执行中间 65% 位移，使每个活动足端最终位于真实支撑面。
    stand_targets = np.array(
        (
            (0.382, -0.159, 0.1965),
            (0.370, 0.113, 0.1965),
            (0.330, 0.003, 0.1965),
            (0.518, -0.159, 0.1965),
            (0.530, 0.113, 0.1965),
            (0.570, 0.003, 0.1965),
        ),
        dtype=np.float64,
    )
    approach_legs = (back.LB, back.LF, back.RM)
    add(guided_tripod(
        "TRIPOD_A_TOP_APPROACH",
        approach_legs,
        stand_targets,
        np.array((0.4185, -0.0235, 0.267, 0.0, -0.042)),
        16,
        0.030,
    ))

    # C34：机身完成最后小位移时，在宽低面再次重置 LM：垂直抬起、低面内侧悬停、
    # 垂直落足。设计上它仍是低面支撑。
    lm_final_low = anchors[back.LM].copy()
    lm_final_low += np.array((0.050, 0.030, low_slope_dz_dx * 0.050))
    lm_final_lift = anchors.copy()
    lm_final_lift[back.LM, 2] += 0.030
    lm_final_hover = anchors.copy()
    lm_final_hover[back.LM] = lm_final_low + np.array((0.0, 0.0, 0.030))
    lm_final_anchors = anchors.copy()
    lm_final_anchors[back.LM] = lm_final_low
    add(back._make_stage(
        "LM_FINAL_POSTURE_WITH_BODY",
        pose,
        np.array((0.45, -0.020, 0.260, 0.0, 0.0)),
        (anchors.copy(), lm_final_lift, lm_final_hover, lm_final_anchors),
        (back.LM,),
        (back.LB, back.LF, back.RB, back.RF, back.RM),
        contacts,
        contacts,
        31,
    ))
    stages[-1]["preview_segment_duration_s"] = 0.25

    # 五个顶面足端现已到达最终接触点；水平机身和竖直 LM 低面支撑构成下一阶段
    # 的站立姿态。
    add(back._body_stage(
        "STAND_FINAL_HOLD", pose, pose.copy(), anchors, tuple(range(6)), contacts
    ))
    return stages


def build_compact(sim_trace_path=OUTPUT_PATH, compact_path=COMPACT_PATH):
    front_bytes = back.SOURCE_TRACE_PATH.read_bytes()
    strict_bytes = STRICT_TRACE_PATH.read_bytes()
    sim_bytes = Path(sim_trace_path).read_bytes()
    front_trace = json.loads(front_bytes)
    strict_trace = json.loads(strict_bytes)
    sim_trace = json.loads(sim_bytes)
    current = json.loads(Path(compact_path).read_text())
    if current.get("schema") == "SIMULATION_ONLY_CLIMB_COMPACT_V1":
        legacy = {
            name: current[name]
            for name in (
                "p0",
                "prep",
                "rm",
                "body",
                "pair",
                "durations_s",
                "settle_gate",
            )
        }
    elif current.get("schema") == "SIMULATION_ONLY_CLIMB_COMPACT_V2":
        legacy = current["front_v1_receipt"]
    else:
        raise ValueError("compact upgrade requires the audited V1/V2 receipt")
    if (
        front_trace.get("status") != "MODEL_PATH_FOUND"
        or strict_trace.get("status") != "PARTIAL_STAGE_COMPLETE"
        or strict_trace.get("stop_after") != "B1"
        or sim_trace.get("status") != "SIM_CANDIDATE_PATH_FOUND"
    ):
        raise ValueError("compact sources do not contain the required results")
    config = json.loads(back.CONFIG_PATH.read_text())
    _, extraction, normal, _ = front.selector_geometry(config)
    fixed = _preview_back_stages(
        back.fixed_stages(front_trace, extraction, normal), normal
    )
    front_stages = _front_compact_stages(front_trace, legacy)
    pair_node = strict_trace["nodes"]["PAIR_SOURCE"]
    q_nodes = {
        **strict_trace["nodes"],
        **sim_trace["nodes"],
    }
    model = k.GraspKinematic()
    q_expected = np.asarray(pair_node["q_rad"], dtype=np.float64)
    back_stages = []
    for stage in fixed:
        active = list(stage["active_legs"])
        durations = (
            [stage.get("preview_segment_duration_s", 2.0)]
            * stage["segments"]
            if active
            else [4.0]
        )
        if stage["name"] in q_nodes:
            q_expected = np.asarray(
                q_nodes[stage["name"]]["q_rad"], dtype=np.float64
            )
        else:
            q_expected, residual = front.solve_anchors(
                model,
                stage["anchor_knots"][-1],
                back.pose_matrix(stage["pose_end"]),
                q_expected,
                1e-8,
            )
            if residual > 1e-6:
                raise ValueError(
                    "compact endpoint IK failed: {} residual={}".format(
                        stage["name"], residual
                    )
                )
        back_stages.append(
            {
                "name": stage["name"],
                "pose_start": stage["pose_start"].tolist(),
                "pose_end": stage["pose_end"].tolist(),
                "anchor_knots": stage["anchor_knots"].tolist(),
                "active_legs": active,
                "segment_durations_s": durations,
                "pose_curve": "quintic_full_stage",
                "anchor_curve": "piecewise_quintic",
                "settle_s": 0.5,
            }
        )
    stages = front_stages + back_stages
    front_end = front_stages[-1]
    back_start = back_stages[0]
    if (
        not np.allclose(
            front_end["pose_end"], back_start["pose_start"], atol=1e-12
        )
        or not np.allclose(
            front_end["anchor_knots"][-1],
            back_start["anchor_knots"][0],
            atol=1e-12,
        )
    ):
        raise ValueError("front/back compact boundary mismatch")
    settle_gate = dict(legacy["settle_gate"])
    # 仅仿真工程容差，按当前 PhysX 回放测得（6.65273439 mm）；仍只是命令目标
    # 跟踪，绝非接触或承载证明。
    settle_gate["max_foot_target_error_m"] = (
        SIM_SETTLE_MAX_FOOT_TARGET_ERROR_M
    )
    settle_gate["preview_time_only_stage_advance"] = True
    settle_gate["tracking_errors_diagnostic_only"] = True
    compact = {
        "schema": "SIMULATION_ONLY_CLIMB_COMPACT_V2",
        "simulation_only": True,
        "simulation_candidate_only": True,
        "source_traces": {
            "front": {
                "path": back.SOURCE_TRACE_PATH.name,
                "sha256": hashlib.sha256(front_bytes).hexdigest(),
            },
            "strict_back_prefix": {
                "path": STRICT_TRACE_PATH.name,
                "sha256": hashlib.sha256(strict_bytes).hexdigest(),
                "stop_after": "B1",
            },
            "sim_finish": {
                "path": Path(sim_trace_path).name,
                "sha256": hashlib.sha256(sim_bytes).hexdigest(),
            },
        },
        "xiaolan_translation": [0.45, -0.03, 0.0],
        "p0": legacy["p0"],
        "front_v1_receipt": legacy,
        "settle_gate": settle_gate,
        "stage_count": len(stages),
        "stages": stages,
        "visual_validation_deferred_for_sim_finish": [
            *PREVIEW_SUFFIX_NAMES,
        ],
    }
    back._atomic_write(Path(compact_path), compact)
    return back._json_value(compact)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=OUTPUT_PATH)
    parser.add_argument(
        "--refresh-sim-trace",
        action="store_true",
        help="压缩前重新生成历史仿真 trace",
    )
    args = parser.parse_args(argv)
    if args.refresh_sim_trace:
        result = build_sim_finish(args.output)
        if result["status"] != "SIM_CANDIDATE_PATH_FOUND":
            print(
                "SIM_TRACE_REFRESH={} elapsed={:.3f}s output={}".format(
                    result["status"], result["planner_elapsed_s"], args.output
                )
            )
            return
        build_compact(args.output, COMPACT_PATH)
        print(
            "COMPACT_BUILT refresh={} elapsed={:.3f}s output={}".format(
                result["status"], result["planner_elapsed_s"], COMPACT_PATH
            )
        )
        return

    compact = build_compact(args.output, COMPACT_PATH)
    print(
        "COMPACT_BUILT existing_sim_trace stages={} output={}".format(
            compact["stage_count"], COMPACT_PATH
        )
    )


if __name__ == "__main__":
    main()
