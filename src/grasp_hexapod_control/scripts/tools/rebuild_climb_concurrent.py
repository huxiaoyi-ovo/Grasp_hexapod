#!/usr/bin/env python3
"""Build bounded concurrent-climb candidates from the frozen 68f compact plan."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
BASELINE_SHA256 = "68f60b97b167ca644f0cebe5fd4de1016b9705764a0376b0431459b0e818346c"
sys.path.insert(0, str(ROOT / "src/grasp_hexapod_control/scripts"))
from climb_mode import ClimbMode
from control import GraspController
from kinematics import JOINT_LOWER, JOINT_UPPER
from utils.climb import gravity_projected_support
from rebuild_climb_tail import original_tangents, stage_pose


def source_world(stage, leg, time_s):
    mode = ClimbMode(None)
    mode.config = {"stages": [stage]}
    mode.stage_index = 0
    mode.phase_time = float(time_s)
    return mode._stage_reference()[1][leg].tolist()


def base_knots(stage):
    times = np.r_[0.0, np.cumsum(stage["segment_durations_s"])]
    world = np.asarray(stage["anchor_knots"], float)
    values = []
    for time_s, knot in zip(times, world):
        inverse = np.linalg.inv(ClimbMode._world_from_base(stage_pose(stage, time_s)))
        values.append([(inverse @ np.r_[knot[leg], 1.0])[:3].tolist()
                       for leg in stage["active_legs"]])
    return values


def merge(stages, start, stop, name, durations, paths):
    """Merge contiguous source stages with supplied world-foot knot paths."""
    source = stages[start:stop + 1]
    first, last = source[0], source[-1]
    if len(durations) + 1 != len(next(iter(paths.values()))):
        raise ValueError(name + " knot/duration shape")
    if any(len(path) != len(durations) + 1 for path in paths.values()):
        raise ValueError(name + " unequal active knot path")
    item = copy.deepcopy(last)
    item["name"] = name
    item["pose_start"] = copy.deepcopy(first["pose_start"])
    item["pose_end"] = copy.deepcopy(last["pose_end"])
    item["pose_curve"] = "quintic_full_stage"
    item["anchor_curve"] = "piecewise_base_quintic"
    item["active_legs"] = sorted(paths)
    item["segment_durations_s"] = [float(x) for x in durations]
    initial = np.asarray(first["anchor_knots"][0], float)
    knots = np.repeat(initial[None, :, :], len(durations) + 1, axis=0)
    for leg, path in paths.items():
        knots[:, leg] = np.asarray(path, float)
    if not np.allclose(knots[-1], np.asarray(last["anchor_knots"][-1], float),
                       rtol=0.0, atol=1e-12):
        raise ValueError(name + " final anchor mismatch")
    item["anchor_knots"] = knots.tolist()
    item["active_base_knots_m"] = base_knots(item)
    item.pop("continuous_air_transition", None)
    item.pop("active_base_velocities_m_s", None)
    original_tangents(item)
    settle = max(float(stage.get("settle_s", 0.0)) for stage in source)
    persistence = max(float(stage.get("settle_persistence_s", 0.0)) for stage in source)
    if settle:
        item["settle_s"] = settle
    else:
        item.pop("settle_s", None)
    if persistence:
        item["settle_persistence_s"] = persistence
    else:
        item.pop("settle_persistence_s", None)
    return item


def replace(data, blocks):
    stages = data["stages"]
    output = []
    index = 0
    for start, stop, item in blocks:
        if index != start:
            output.extend(copy.deepcopy(stages[index:start]))
        output.append(item)
        index = stop + 1
    output.extend(copy.deepcopy(stages[index:]))
    data["stages"] = output
    data["stage_count"] = len(output)
    return data


def build_case(baseline, case):
    data = copy.deepcopy(baseline)
    s = baseline["stages"]
    required = {
        "A": ((9, "BODY3"), (10, "RB_RF_SHIFT1"),
              (11, "LM_GROUND_SHIFT1"), (12, "RM_PRE_ADVANCE")),
        "B": ((15, "LM_EDGE_STAGE"), (16, "BODY_A"),
              (17, "RM_RIGHT_SYMMETRY")),
        "C": ((27, "RM_DIRECT_FINAL"), (28, "BODY_REPOSITION"),
              (29, "LB_LF_DIRECT_FINAL"), (30, "BODY_DOCK_FINAL")),
    }
    if case == "AB1C":
        required = required["A"] + required["B"] + required["C"]
    elif case in ("AC", "ABC"):
        required = tuple(item for name in case for item in required[name])
    else:
        required = required[case]
    if any(index >= len(s) or s[index]["name"] != name for index, name in required):
        raise ValueError(case + " source stage map differs from frozen 68f")
    if case == "C":
        c28 = merge(s, 27, 28, "RM_BODY_REPOSITION", s[27]["segment_durations_s"],
                    {5: [k[5] for k in s[27]["anchor_knots"]]})
        c30 = merge(s, 29, 30, "LB_LF_DOCK_TRANSFER", s[29]["segment_durations_s"],
                    {0: [k[0] for k in s[29]["anchor_knots"]],
                     1: [k[1] for k in s[29]["anchor_knots"]]})
        return replace(data, [(27, 28, c28), (29, 30, c30)])
    if case == "A":
        rb = merge(s, 9, 10, "RB_BODY_ADVANCE", s[10]["segment_durations_s"],
                   {3: [k[3] for k in s[10]["anchor_knots"]]})
        lm = [s[11]["anchor_knots"][0][2],
              source_world(s[11], 2, .175),
              s[11]["anchor_knots"][1][2], s[11]["anchor_knots"][2][2],
              s[11]["anchor_knots"][-1][2]]
        pair = merge(s, 11, 12, "LM_RM_PRE_ADVANCE", [.35, .35, .82, .5],
                     {2: lm, 5: [k[5] for k in s[12]["anchor_knots"]]})
        return replace(data, [(9, 10, rb), (11, 12, pair)])
    if case == "B":
        end = np.asarray(s[17]["anchor_knots"][-1][5], float)
        apex = np.asarray(s[17]["anchor_knots"][1][5], float)
        extra = np.array([end[0], end[1], max(apex[2], end[2] + .03)])
        item = merge(s, 15, 17, "LM_RM_BODY_TRANSFER", [.45, .95, 1.22],
                     {2: [k[2] for k in s[15]["anchor_knots"]],
                      5: [s[17]["anchor_knots"][0][5], apex, extra, end]})
        return replace(data, [(15, 17, item)])
    if case == "ABC":
        rb = merge(s, 9, 10, "RB_BODY_ADVANCE", s[10]["segment_durations_s"],
                   {3: [k[3] for k in s[10]["anchor_knots"]]})
        lm = [s[11]["anchor_knots"][0][2], source_world(s[11], 2, .175),
              s[11]["anchor_knots"][1][2], s[11]["anchor_knots"][2][2],
              s[11]["anchor_knots"][-1][2]]
        pair = merge(s, 11, 12, "LM_RM_PRE_ADVANCE", [.35, .35, .82, .5],
                     {2: lm, 5: [k[5] for k in s[12]["anchor_knots"]]})
        end = np.asarray(s[17]["anchor_knots"][-1][5], float)
        apex = np.asarray(s[17]["anchor_knots"][1][5], float)
        extra = np.array([end[0], end[1], max(apex[2], end[2] + .03)])
        lm_rm = merge(s, 15, 17, "LM_RM_BODY_TRANSFER", [.45, .95, 1.22],
                      {2: [k[2] for k in s[15]["anchor_knots"]],
                       5: [s[17]["anchor_knots"][0][5], apex, extra, end]})
        rm = merge(s, 27, 28, "RM_BODY_REPOSITION", s[27]["segment_durations_s"],
                   {5: [k[5] for k in s[27]["anchor_knots"]]})
        left = merge(s, 29, 30, "LB_LF_DOCK_TRANSFER", s[29]["segment_durations_s"],
                     {0: [k[0] for k in s[29]["anchor_knots"]],
                      1: [k[1] for k in s[29]["anchor_knots"]]})
        return replace(data, [(9, 10, rb), (11, 12, pair), (15, 17, lm_rm),
                              (27, 28, rm), (29, 30, left)])
    if case == "AC":
        rb = merge(s, 9, 10, "RB_BODY_ADVANCE", s[10]["segment_durations_s"],
                   {3: [k[3] for k in s[10]["anchor_knots"]]})
        lm = [s[11]["anchor_knots"][0][2], source_world(s[11], 2, .175),
              s[11]["anchor_knots"][1][2], s[11]["anchor_knots"][2][2],
              s[11]["anchor_knots"][-1][2]]
        pair = merge(s, 11, 12, "LM_RM_PRE_ADVANCE", [.35, .35, .82, .5],
                     {2: lm, 5: [k[5] for k in s[12]["anchor_knots"]]})
        rm = merge(s, 27, 28, "RM_BODY_REPOSITION", s[27]["segment_durations_s"],
                   {5: [k[5] for k in s[27]["anchor_knots"]]})
        left = merge(s, 29, 30, "LB_LF_DOCK_TRANSFER", s[29]["segment_durations_s"],
                     {0: [k[0] for k in s[29]["anchor_knots"]],
                      1: [k[1] for k in s[29]["anchor_knots"]]})
        return replace(data, [(9, 10, rb), (11, 12, pair),
                              (27, 28, rm), (29, 30, left)])
    if case == "AB1C":
        rb = merge(s, 9, 10, "RB_BODY_ADVANCE", s[10]["segment_durations_s"],
                   {3: [k[3] for k in s[10]["anchor_knots"]]})
        lm = [s[11]["anchor_knots"][0][2], source_world(s[11], 2, .175),
              s[11]["anchor_knots"][1][2], s[11]["anchor_knots"][2][2],
              s[11]["anchor_knots"][-1][2]]
        pair = merge(s, 11, 12, "LM_RM_PRE_ADVANCE", [.35, .35, .82, .5],
                     {2: lm, 5: [k[5] for k in s[12]["anchor_knots"]]})
        lm_body = merge(s, 15, 16, "LM_BODY_TRANSFER", s[15]["segment_durations_s"],
                        {2: [k[2] for k in s[15]["anchor_knots"]]})
        rm = merge(s, 27, 28, "RM_BODY_REPOSITION", s[27]["segment_durations_s"],
                   {5: [k[5] for k in s[27]["anchor_knots"]]})
        left = merge(s, 29, 30, "LB_LF_DOCK_TRANSFER", s[29]["segment_durations_s"],
                     {0: [k[0] for k in s[29]["anchor_knots"]],
                      1: [k[1] for k in s[29]["anchor_knots"]]})
        return replace(data, [(9, 10, rb), (11, 12, pair), (15, 16, lm_body),
                              (27, 28, rm), (29, 30, left)])
    raise ValueError("unknown case " + case)


def solve(kinematic, q, desired):
    for _ in range(1200):
        error = kinematic.base_to_hip(desired) - kinematic.forward(q)
        if float(np.max(np.linalg.norm(error, axis=1))) <= 5e-8:
            break
        delta = (kinematic.damped_inverse_jacobian(q, .001) @ error[..., None]).squeeze(-1)
        q = np.clip(q + np.clip(delta, -.02, .02), JOINT_LOWER, JOINT_UPPER)
    return q, np.linalg.norm(desired - kinematic.forward_base(q), axis=1)


def sparse_report(data):
    kinematic = GraspController(1 / 30).kinematic
    mode = ClimbMode(None); mode.config = data
    q = np.asarray(data["p0"]["q_rad"], float)
    report = {"min_joint_margin_rad": np.inf, "min_support_margin_m": np.inf,
              "min_support_source": None, "invalid_support_source": None,
              "max_sparse_secant_joint_speed_rad_s": 0.0, "samples": 0}
    for index, stage in enumerate(data["stages"]):
        previous_q = None; previous_t = None
        total = float(sum(stage["segment_durations_s"]))
        for time_s in (0.0, total / 2.0, total):
            mode.stage_index = index; mode.phase_time = time_s
            pose, anchors, _ = mode._stage_reference()
            inverse = np.linalg.inv(ClimbMode._world_from_base(pose))
            desired = (np.c_[anchors, np.ones(6)] @ inverse.T)[:, :3]
            q, residual = solve(kinematic, q, desired)
            residual_value = float(np.max(residual))
            if residual_value > 5e-8:
                raise ValueError("sparse IK failure stage=%s time_s=%.9f leg=%d residual_m=%.3e" % (
                    stage["name"], time_s, int(np.argmax(residual)), residual_value))
            report["samples"] += 1
            report["min_joint_margin_rad"] = min(report["min_joint_margin_rad"], float(np.min(kinematic.joint_limit_margins(q))))
            fixed = [leg for leg in range(6) if leg not in stage["active_legs"]]
            com = (ClimbMode._world_from_base(pose) @ np.r_[kinematic.center_of_mass_base(q), 1.0])[:3]
            support = gravity_projected_support(com, anchors[fixed], (0., 0., -1.))
            if support.valid:
                value = float(support.raw_margin_m)
                if value < report["min_support_margin_m"]:
                    report["min_support_margin_m"] = value
                    report["min_support_source"] = {"stage": stage["name"], "time_s": time_s}
            elif report["invalid_support_source"] is None:
                report["invalid_support_source"] = {"stage": stage["name"], "time_s": time_s,
                                                    "fixed_legs": fixed}
            if previous_q is not None and time_s > previous_t:
                report["max_sparse_secant_joint_speed_rad_s"] = max(report["max_sparse_secant_joint_speed_rad_s"], float(np.max(np.abs(q - previous_q) / (time_s - previous_t))))
            previous_q, previous_t = q.copy(), time_s
    if not np.isfinite(report["min_support_margin_m"]):
        report["min_support_margin_m"] = None
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--case", choices=("A", "B", "C", "AC", "ABC", "AB1C"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    raw = args.input.read_bytes()
    if hashlib.sha256(raw).hexdigest() != BASELINE_SHA256:
        raise ValueError("input must be frozen 68f baseline")
    baseline = json.loads(raw)
    candidate = build_case(baseline, args.case)
    ClimbMode(None)._validate_config(candidate)
    report = sparse_report(candidate)
    report.update({"case": args.case, "stage_count": candidate["stage_count"],
                   "planned_duration_s": float(sum(sum(x["segment_durations_s"]) for x in candidate["stages"]))})
    args.output.write_text(json.dumps(candidate, indent=2) + "\n")
    args.report.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
