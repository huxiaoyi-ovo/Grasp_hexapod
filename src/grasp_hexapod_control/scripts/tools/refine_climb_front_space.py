#!/usr/bin/env python3
"""Build the frozen 19-stage simulation-only front-space candidate.

execution_owner: Luna. Sol owns physical interpretation and acceptance.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = ROOT / "src/grasp_hexapod_control/scripts"
sys.path[:0] = [str(SCRIPTS), str(SCRIPTS / "tools")]
from build_climb_front import (  # noqa: E402
    MESH, TRANSLATION, foot_sphere_signed_gap, load_triangles, solve_ik,
    surface_height,
)
from kinematics import GraspKinematic, JOINT_LOWER, JOINT_UPPER  # noqa: E402
from optimize_climb_front import _refresh_active_base_knots  # noqa: E402
from replan_climb_front import make_stage  # noqa: E402

PATCH_HELPER_PATH = ROOT / "src/docs/evidence/climb_front_terminal_20260924/front_terminal_audit.py"
spec = importlib.util.spec_from_file_location("front_patch_helper", PATCH_HELPER_PATH)
patch = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = patch
spec.loader.exec_module(patch)
LEGS = ("LB", "LF", "LM", "RB", "RF", "RM")
SOURCE = ROOT / "src/grasp_hexapod_control/config/climb_front_direct.json"
OUTPUT = ROOT / "src/grasp_hexapod_control/config/climb_front_space.json"
EVIDENCE = ROOT / "src/docs/evidence/climb_front_space_20260925"
# Contract-authorized local retries only; each value multiplies that stage's
# frozen base duration after the original C10 1.15 correction.
LOCAL_STAGE_DURATION_FACTORS = {"BODY_TO_105": 1.1309624348962446}


def target_on_surface(local_triangles, world_triangles, x, y):
    point = np.array([x, y, surface_height(local_triangles, x, y)], dtype=float)
    for _ in range(3):
        gap = foot_sphere_signed_gap(world_triangles, point)
        # 0.1 mm CAD discretization tolerance, stricter than the CPU audit's
        # 0.5 mm active-foot rejection threshold; this is not contact evidence.
        if gap >= -1e-4:
            return point
        normal = np.asarray(patch.inspect_nearest_patch(world_triangles, point)[0]["face_normal_world"], dtype=float)
        if normal[2] <= .1:
            raise ValueError(
                f"surface target at x={x:.6f}, y={y:.6f} has unsuitable face normal z={normal[2]:.6f}"
            )
        point[2] += (-gap + 1e-6) / normal[2]
    gap = foot_sphere_signed_gap(world_triangles, point)
    if gap < -1e-4:
        raise ValueError(f"surface target at x={x:.6f}, y={y:.6f} remains gap={gap:.9g} after 3 normal corrections")
    return point


def build(source, build_metrics=None):
    baseline_bytes = SOURCE.read_bytes()
    baseline_sha = hashlib.sha256(baseline_bytes).hexdigest()
    expected = "4421aff133035f358e69ed33ddaa649d580750d868f2318736efc17e797bf60c"
    if baseline_sha != expected:
        raise ValueError(f"source SHA mismatch: {baseline_sha}")
    cfg = copy.deepcopy(source)
    original = source["stages"]
    names = [s["name"] for s in original]
    required_prefix = ["PREP", "FRONT_LOW", "BODY_A", "M_GROUND_FIRST_MINUS02", "FRONT_TO_029", "SIX_SUPPORT_BODY_ADVANCE", "BACK_TO_MINUS09"]
    if names[:7] != required_prefix:
        raise ValueError("baseline C1-C7 stage chain changed")
    stages = copy.deepcopy(original[:7])
    pose = np.asarray(stages[-1]["pose_end"], dtype=float)
    anchors = np.asarray(stages[-1]["anchor_knots"][-1], dtype=float)
    local_triangles = load_triangles(MESH)
    world_triangles = local_triangles + TRANSLATION

    def append(name, end_pose, end, active, durations, curve="quintic_full_stage", knots=None):
        nonlocal pose, anchors
        stage = make_stage(name, pose, end_pose, anchors, end, active, durations, curve, knots)
        stage["settle_s"] = .10
        stages.append(stage)
        pose = np.asarray(stage["pose_end"], dtype=float)
        anchors = np.asarray(stage["anchor_knots"][-1], dtype=float)

    def pose_to(x, z, roll, pitch):
        result = pose.copy()
        result[0], result[2], result[3], result[4] = x, z, roll, pitch
        return result

    # C8: retain original pose and timing; only LM/RM endpoint advances to x=.080.
    old = original[7]
    end = anchors.copy()
    for leg in (2, 5):
        end[leg, 0] = .080
    append("MID_GROUND_TO_080", old["pose_end"], end, [2, 5], old["segment_durations_s"])

    # C9: preserve timing and pose; transmit the moved middle anchors through its fixed legs.
    old = original[8]
    end = anchors.copy()
    for leg in (1, 4):
        end[leg] = np.asarray(old["anchor_knots"][-1][leg], dtype=float)
    append("FRONT_TO_034", old["pose_end"], end, [1, 4], old["segment_durations_s"])

    # C10: lengthen only the observed velocity-clipping body move by 15%.
    append("BODY_TO_105", pose_to(.105, .150, .15, 0.0), anchors.copy(), [], [original[9]["segment_durations_s"][0] * 1.15 * LOCAL_STAGE_DURATION_FACTORS.get("BODY_TO_105", 1.0)])

    # C11: one rear leg at a time is not used; both back feet take the single long ground step.
    end = anchors.copy()
    end[[0, 3], 0] = .055
    append("BACK_TO_055", pose_to(.130, .170, .15, 0.0), end, [0, 3], [.6, .9, .4])

    # C12 lifts vertically at x=.080 before crossing the low protrusion.
    start = anchors.copy(); end = start.copy()
    knots = [start.copy() for _ in range(6)]
    for leg in (2, 5):
        y = start[leg, 1]
        land = target_on_surface(local_triangles, world_triangles, .285, y)
        end[leg] = land
        knots[1][leg] = (.080, y, .140)
        knots[2][leg] = (.220, y, .180)
        knots[3][leg] = (.250, y, .205)
        knots[4][leg] = (.285, y, land[2] + .025)
        knots[5][leg] = land
    knots[2][5][2] = .183
    knots[3][5][2] = .208
    pose_knots = [
        pose.copy(),
        np.array([.145, pose[1], .205, .15, 0.0, pose[5]]),
        np.array([.185, pose[1], .215, .35, 0.0, pose[5]]),
        np.array([.190, pose[1], .215, .40, 0.0, pose[5]]),
        np.array([.195, pose[1], .215, .40, 0.0, pose[5]]),
        np.array([.195, pose[1], .215, .40, 0.0, pose[5]]),
    ]
    append("MID_PLATFORM_285", pose_knots[-1], end, [2, 5], [.85, .75, .40, .45, .35], "quintic_full_stage", knots)
    # These are world-space knots, so the first leg segment remains vertical
    # while independently timed body knots raise the chassis before crossing.
    stages[-1]["anchor_curve"] = "piecewise_quintic"
    stages[-1]["pose_curve"] = "piecewise_quintic"
    stages[-1]["pose_knots"] = [row.tolist() for row in pose_knots]
    stages[-1].pop("active_base_knots_m", None)
    stages[-1].pop("active_base_velocities_m_s", None)

    # C13: front pair advances across the platform surface to x=.400.
    end = anchors.copy()
    for leg in (1, 4): end[leg] = target_on_surface(local_triangles, world_triangles, .400, anchors[leg, 1])
    append("FRONT_TO_0400", pose_to(.195, .215, .400, 0.0), end, [1, 4], [.45, .65, .35])

    # C14-C16: ordinary 20 mm apex swing references.
    end = anchors.copy(); end[3] = np.array([.155, anchors[3, 1], .0065])
    append("RB_GROUND_TO_155", pose_to(.225, .205, .35, 0.0), end, [3], [.5, .7, .4])
    end = anchors.copy(); end[0] = np.array([.155, anchors[0, 1], .0065])
    append("LB_GROUND_TO_155", pose_to(.225, .210, .35, 0.0), end, [0], [.5, .7, .4])
    end = anchors.copy()
    for leg in (1, 4): end[leg] = target_on_surface(local_triangles, world_triangles, .480, anchors[leg, 1])
    append("FRONT_TO_0480", pose_to(.240, .215, .35, 0.0), end, [1, 4], [.5, .7, .4])

    # C17-C18 reuse the already measured baseline terminal STL seeds at B225.
    # Their x/y are frozen; the edge-adjacent vertical rib makes surface_height
    # unsuitable here, while these exact anchors have a documented -38 um gap.
    baseline_terminal = np.asarray(original[-1]["anchor_knots"][-1], dtype=float)
    end = anchors.copy(); rb = baseline_terminal[3].copy()
    if not np.allclose(rb[:2], [.225, anchors[3, 1]], rtol=0, atol=1e-12):
        raise ValueError(f"RB B225 baseline XY changed: {rb[:2].tolist()}")
    rb_gap = foot_sphere_signed_gap(world_triangles, rb)
    if rb_gap < -1e-4:
        raise ValueError(f"RB B225 baseline STL gap {rb_gap:.9g} m is below -0.1 mm")
    end[3] = rb
    k = [anchors.copy() for _ in range(4)]
    k[1][3] = (.155, anchors[3, 1], .180); k[2][3] = (.195, anchors[3, 1], .180); k[3][3] = rb
    append("RB_PLATFORM_225", pose_to(.290, .243, .240, -.250), end, [3], [1.3, .8, .4], "quintic_first_segment", k)
    end = anchors.copy(); lb = baseline_terminal[0].copy()
    if not np.allclose(lb[:2], [.225, anchors[0, 1]], rtol=0, atol=1e-12):
        raise ValueError(f"LB B225 baseline XY changed: {lb[:2].tolist()}")
    lb_gap = foot_sphere_signed_gap(world_triangles, lb)
    if lb_gap < -1e-4:
        raise ValueError(f"LB B225 baseline STL gap {lb_gap:.9g} m is below -0.1 mm")
    end[0] = lb
    k = [anchors.copy() for _ in range(4)]
    k[1][0] = (.155, anchors[0, 1], .185); k[2][0] = (.225, anchors[0, 1], .185); k[3][0] = lb
    append("LB_PLATFORM_225", pose_to(.318, .265, 0.0, 0.0), end, [0], [1.1, .8, .4], "quintic_first_segment", k)
    append("FINAL_HOLD", pose.copy(), anchors.copy(), [], [.6])

    if len(stages) != 19:
        raise ValueError(f"expected 19 stages, got {len(stages)}")
    cfg["stages"] = stages
    cfg["stage_count"] = len(stages)
    kinematic = GraspKinematic()
    final_pose = np.asarray(stages[-1]["pose_end"], dtype=float)
    final_anchors = np.asarray(stages[-1]["anchor_knots"][-1], dtype=float)
    terminal_q, terminal_error = solve_ik(
        kinematic, final_pose, final_anchors,
        np.asarray(source["terminal_q_rad"], dtype=float), tolerance_m=1e-5,
    )
    terminal_margin = float(np.min(np.minimum(
        terminal_q - JOINT_LOWER, JOINT_UPPER - terminal_q
    )))
    if terminal_error > .002 or terminal_margin <= 0.0:
        raise ValueError(
            f"terminal IK failed: residual={terminal_error:.9g} m, "
            f"joint_margin={terminal_margin:.9g} rad"
        )
    cfg["terminal_q_rad"] = terminal_q.tolist()
    for stale in (
        "direct_timing_selection", "mid_inset_refinement",
        "terminal_rear_inset_refinement", "rear_body_advance_refinement",
    ):
        cfg.pop(stale, None)
    cfg["direct_replan_source"] = {"source_config_sha256": baseline_sha}
    if build_metrics is not None:
        build_metrics.update({
            "terminal_ik_residual_m": terminal_error,
            "terminal_joint_margin_rad": terminal_margin,
        })
    return cfg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=SOURCE)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    source_bytes = args.source.read_bytes()
    source = json.loads(source_bytes)
    build_metrics = {}
    cfg = build(source, build_metrics)
    out = (json.dumps(cfg, indent=2, allow_nan=False) + "\n").encode()
    args.output.write_bytes(out)
    receipt = {
        "execution_owner": "Luna",
        "source_path": str(args.source),
        "source_sha256": hashlib.sha256(source_bytes).hexdigest(),
        "target_path": str(args.output),
        "target_sha256": hashlib.sha256(out).hexdigest(),
        "stage_count": len(cfg["stages"]),
        "stage_names": [s["name"] for s in cfg["stages"]],
        "total_duration_s": float(sum(sum(s["segment_durations_s"]) + s["settle_s"] for s in cfg["stages"])),
        **build_metrics,
        "builder_reproducible": None,
        "first_cpu_failure": None,
        "status": "built; CPU audit pending",
    }
    (EVIDENCE / "build_receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt, indent=2))


if __name__ == "__main__":
    main()
