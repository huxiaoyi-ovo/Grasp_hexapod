#!/usr/bin/env python3
"""Build the deterministic, simulation-only front-facing climb candidate.

Landing centres are projected onto the exact static Xiaolan STL triangles.  The
result is a kinematic candidate, not contact, load, stability, or hardware
evidence.
"""
import argparse
import json
import struct
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = ROOT / "src/grasp_hexapod_control/scripts"
sys.path[:0] = [str(SCRIPTS)]

from climb_mode import ClimbMode, COMPACT_SCHEMA
from kinematics import FOOT_RADIUS, GraspKinematic, JOINT_LOWER, JOINT_UPPER, Q_STAND

MESH = ROOT / "src/grasp_hexapod_description/meshes/xiaolan/base_link_xiaolan.STL"
TRANSLATION = np.array((.45, -.03, 0.), dtype=np.float64)
YAW = -np.pi / 2.0
P0 = np.array((-.1, -.067694497636, .069884680528, 0., 0., YAW))
PAIR_GROUPS = {"front": [1, 4], "mid": [2, 5], "back": [0, 3]}
ENTRY_PREFLIGHT_MAX_ERROR_M = .002


def load_triangles(path):
    raw = path.read_bytes()
    count = struct.unpack_from("<I", raw, 80)[0]
    dtype = np.dtype([
        ("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)),
        ("attribute", "<u2"),
    ])
    if len(raw) != 84 + count * dtype.itemsize:
        raise ValueError("front builder requires the binary Xiaolan STL")
    return np.frombuffer(raw, dtype=dtype, count=count, offset=84)["vertices"].astype(float)


def surface_height(triangles, world_x, world_y):
    """Return the highest exact triangle height below one XY landing centre."""
    point = np.array((world_x, world_y), dtype=float) - TRANSLATION[:2]
    heights = []
    for triangle in triangles:
        origin = triangle[0]
        matrix = np.column_stack((triangle[1, :2] - origin[:2], triangle[2, :2] - origin[:2]))
        determinant = np.linalg.det(matrix)
        if abs(determinant) < 1e-12:
            continue
        uv = np.linalg.solve(matrix, point - origin[:2])
        if np.all(uv >= -1e-9) and uv.sum() <= 1.0 + 1e-9:
            heights.append(origin[2] + uv @ (triangle[1:, 2] - origin[2]))
    if not heights:
        raise ValueError("no Xiaolan triangle under landing XY ({:.3f}, {:.3f})".format(world_x, world_y))
    return float(max(heights) + TRANSLATION[2] + FOOT_RADIUS)


def foot_sphere_signed_gap(world_triangles, foot_center_world):
    """Return nearest STL distance minus the physical foot hemisphere radius."""
    triangles = np.asarray(world_triangles, dtype=float)
    point = np.asarray(foot_center_world, dtype=float)
    if triangles.ndim != 3 or triangles.shape[1:] != (3, 3) or point.shape != (3,):
        raise ValueError("ground sphere clearance requires triangle (N,3,3) and point (3,)")
    if len(triangles) == 0 or not (np.all(np.isfinite(triangles)) and np.all(np.isfinite(point))):
        raise ValueError("ground sphere clearance requires finite non-empty geometry")

    # Vectorized closest-point-on-triangle regions (Ericson).  This remains a
    # local per-foot query: it is not a whole-robot collision scan.
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    ab, ac, ap = b - a, c - a, point - a
    d1 = np.einsum("ij,ij->i", ab, ap)
    d2 = np.einsum("ij,ij->i", ac, ap)
    closest = np.empty_like(a)
    done = np.zeros(len(a), dtype=bool)
    mask = (d1 <= 0.0) & (d2 <= 0.0)
    closest[mask] = a[mask]; done |= mask
    bp = point - b
    d3 = np.einsum("ij,ij->i", ab, bp)
    d4 = np.einsum("ij,ij->i", ac, bp)
    mask = (~done) & (d3 >= 0.0) & (d4 <= d3)
    closest[mask] = b[mask]; done |= mask
    vc = d1 * d4 - d3 * d2
    mask = (~done) & (vc <= 0.0) & (d1 >= 0.0) & (d3 <= 0.0)
    weight = np.zeros(len(a)); weight[mask] = d1[mask] / (d1[mask] - d3[mask])
    closest[mask] = a[mask] + weight[mask, None] * ab[mask]; done |= mask
    cp = point - c
    d5 = np.einsum("ij,ij->i", ab, cp)
    d6 = np.einsum("ij,ij->i", ac, cp)
    mask = (~done) & (d6 >= 0.0) & (d5 <= d6)
    closest[mask] = c[mask]; done |= mask
    vb = d5 * d2 - d1 * d6
    mask = (~done) & (vb <= 0.0) & (d2 >= 0.0) & (d6 <= 0.0)
    weight = np.zeros(len(a)); weight[mask] = d2[mask] / (d2[mask] - d6[mask])
    closest[mask] = a[mask] + weight[mask, None] * ac[mask]; done |= mask
    va = d3 * d6 - d5 * d4
    mask = (~done) & (va <= 0.0) & ((d4 - d3) >= 0.0) & ((d5 - d6) >= 0.0)
    weight = np.zeros(len(a))
    weight[mask] = ((d4[mask] - d3[mask]) / ((d4[mask] - d3[mask]) + (d5[mask] - d6[mask])))
    closest[mask] = b[mask] + weight[mask, None] * (c[mask] - b[mask]); done |= mask
    mask = ~done
    denominator = va[mask] + vb[mask] + vc[mask]
    # Degenerate triangles have already been handled by their edge regions;
    # exclude any numerically singular leftovers from the face calculation.
    regular = np.abs(denominator) > 1e-15
    indices = np.flatnonzero(mask)
    if np.any(regular):
        face = indices[regular]
        v = vb[face] / denominator[regular]
        w = vc[face] / denominator[regular]
        closest[face] = a[face] + v[:, None] * ab[face] + w[:, None] * ac[face]
    if np.any(~regular):
        closest[indices[~regular]] = a[indices[~regular]]
    return float(np.min(np.linalg.norm(closest - point, axis=1)) - FOOT_RADIUS)


def require_ground_sphere_clearance(world_triangles, foot_center_world):
    """Fail closed when the physical lower-foot sphere intersects Xiaolan."""
    gap = foot_sphere_signed_gap(world_triangles, foot_center_world)
    if gap < 0.0:
        raise ValueError(
            "ground foot sphere intersects Xiaolan STL by {:.6f}m".format(-gap)
        )
    return gap


def require_surface_sphere_clearance(world_triangles, foot_center_world):
    """Reject a planned Xiaolan landing beyond the local half-millimetre screen."""
    gap = foot_sphere_signed_gap(world_triangles, foot_center_world)
    if gap < -.0005:
        raise ValueError(
            "surface foot sphere enters Xiaolan STL by {:.6f}m".format(-gap)
        )
    return gap


def world_from_base(base):
    return ClimbMode._world_from_base(np.asarray(base, dtype=float))


def base_points(base, world_points):
    inverse = np.linalg.inv(world_from_base(base))
    return (np.column_stack((world_points, np.ones(len(world_points)))) @ inverse.T)[:, :3]


def preflight_synthetic_stage_entry(
    kinematic, pose, anchors_world, q_rad, foot_desired_base_prev,
    max_error_m=ENTRY_PREFLIGHT_MAX_ERROR_M,
):
    """Reject an offline entry whose joint or prior foot reference is stale.

    Synthetic local checks must derive the joint seed and controller previous
    foot reference from the same pose and world anchors.  Without this gate a
    state captured at an earlier runtime stage can produce an unrelated speed
    or IK result after anchors are substituted.
    """
    pose = np.asarray(pose, dtype=float)
    anchors = np.asarray(anchors_world, dtype=float)
    q = np.asarray(q_rad, dtype=float)
    previous = np.asarray(foot_desired_base_prev, dtype=float)
    if (pose.shape != (6,) or anchors.shape != (6, 3) or q.shape != (6, 3)
            or previous.shape != (6, 3)):
        raise ValueError("synthetic stage-entry requires pose (6,) and six-leg (6,3) arrays")
    if not (np.all(np.isfinite(pose)) and np.all(np.isfinite(anchors))
            and np.all(np.isfinite(q)) and np.all(np.isfinite(previous))
            and np.isfinite(max_error_m) and max_error_m > 0.0):
        raise ValueError("synthetic stage-entry requires finite arrays and positive tolerance")

    target = base_points(pose, anchors)
    fk_error = np.linalg.norm(kinematic.forward_base(q) - target, axis=1)
    previous_error = np.linalg.norm(previous - target, axis=1)
    max_fk_error = float(np.max(fk_error))
    max_previous_error = float(np.max(previous_error))
    if max_fk_error > max_error_m:
        raise ValueError(
            "synthetic stage-entry FK mismatch {:.6f}m exceeds {:.6f}m"
            .format(max_fk_error, max_error_m)
        )
    if max_previous_error > max_error_m:
        raise ValueError(
            "synthetic stage-entry previous-reference mismatch {:.6f}m exceeds {:.6f}m"
            .format(max_previous_error, max_error_m)
        )
    return target


def quintic_weight(phase):
    """Runtime-compatible full-stage quintic interpolation weight."""
    phase = float(np.clip(phase, 0.0, 1.0))
    return 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5


def piecewise_quintic(knots, durations, elapsed):
    """Match ClimbMode's zero-velocity piecewise base reference."""
    knots = np.asarray(knots, dtype=float)
    durations = np.asarray(durations, dtype=float)
    remaining = float(np.clip(elapsed, 0.0, float(durations.sum())))
    for index, duration in enumerate(durations):
        if remaining <= duration:
            weight = quintic_weight(remaining / duration)
            return (1.0 - weight) * knots[index] + weight * knots[index + 1]
        remaining -= duration
    return knots[-1].copy()


def ground_pair_sphere_sweep(
    world_triangles, pose_start, pose_end, world_knots, active_base_knots,
    durations, legs, max_vertical_step_m=.005,
):
    """Sample the actual base-relative ground-pair reference against the STL."""
    world_knots = np.asarray(world_knots, dtype=float)
    base_knots = np.asarray(active_base_knots, dtype=float)
    durations = np.asarray(durations, dtype=float)
    legs = np.asarray(legs, dtype=int)
    if (world_knots.shape != (len(durations) + 1, 6, 3)
            or base_knots.shape != (len(durations) + 1, len(legs), 3)
            or np.asarray(pose_start).shape != (6,) or np.asarray(pose_end).shape != (6,)
            or not np.all(np.isfinite(durations)) or np.any(durations <= 0.0)
            or max_vertical_step_m <= 0.0):
        raise ValueError("ground sphere sweep received invalid stage reference")
    sample_times = [0.0]
    elapsed = 0.0
    for index, duration in enumerate(durations):
        vertical_delta = np.max(np.abs(world_knots[index + 1, legs, 2] - world_knots[index, legs, 2]))
        count = max(int(np.ceil(duration * 30.0)), int(np.ceil(vertical_delta / max_vertical_step_m)), 1)
        sample_times.extend(elapsed + duration * sample / count for sample in range(1, count + 1))
        elapsed += duration
    gaps = []
    first_failure = None
    total = float(durations.sum())
    for elapsed in sample_times:
        pose = ((1.0 - quintic_weight(elapsed / total)) * np.asarray(pose_start, dtype=float)
                + quintic_weight(elapsed / total) * np.asarray(pose_end, dtype=float))
        active_base = piecewise_quintic(base_knots, durations, elapsed)
        active_world = (
            np.column_stack((active_base, np.ones(len(legs)))) @ world_from_base(pose).T
        )[:, :3]
        for leg, foot_center in zip(legs, active_world):
            gap = foot_sphere_signed_gap(world_triangles, foot_center)
            gaps.append((float(elapsed), int(leg), float(gap)))
            if first_failure is None and gap < -.0005:
                first_failure = gaps[-1]
    minimum = min(gaps, key=lambda item: item[2])
    return {
        "sample_count": len(gaps), "min_signed_sphere_gap_m": minimum[2],
        "min_gap_time_s": minimum[0], "min_gap_leg": minimum[1],
        "first_below_negative_half_mm": first_failure,
    }


def require_ground_pair_sphere_sweep(*args, **kwargs):
    """Fail closed when the sampled physical ground-pair reference enters STL."""
    receipt = ground_pair_sphere_sweep(*args, **kwargs)
    if receipt["first_below_negative_half_mm"] is not None:
        time_s, leg, gap = receipt["first_below_negative_half_mm"]
        raise ValueError(
            "ground pair sphere sweep enters Xiaolan STL: leg {} at {:.3f}s, gap {:.6f}m"
            .format(leg, time_s, gap)
        )
    return receipt


def support_margin(point, support):
    """Signed hull-edge distance for the model COM projection."""
    points = np.unique(np.asarray(support, dtype=float)[:, :2], axis=0)
    if len(points) < 3:
        return -np.inf
    points = points[np.lexsort((points[:, 1], points[:, 0]))]
    def cross(o, a, b):
        return np.cross(a - o, b - o)
    lower, upper = [], []
    for candidate in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], candidate) <= 0:
            lower.pop()
        lower.append(candidate)
    for candidate in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], candidate) <= 0:
            upper.pop()
        upper.append(candidate)
    hull = np.asarray(lower[:-1] + upper[:-1])
    signed = []
    for index, a in enumerate(hull):
        b = hull[(index + 1) % len(hull)]
        edge = b - a
        signed.append(np.cross(edge, point - a) / np.linalg.norm(edge))
    return float(min(signed))


def solve_ik(kinematic, pose, anchors, seed, tolerance_m=.004):
    """Independent DLS endpoint diagnostic; it does not certify contact."""
    target = base_points(pose, anchors)
    target_hip = kinematic.base_to_hip(target)
    q = np.asarray(seed, dtype=float).copy()
    for _ in range(700):
        error = target_hip - kinematic.forward(q)
        if np.max(np.linalg.norm(error, axis=1)) < tolerance_m:
            break
        update = np.einsum("lij,lj->li", kinematic.damped_inverse_jacobian(q, .015), error)
        q = np.clip(q + np.clip(update, -.10, .10), JOINT_LOWER, JOINT_UPPER)
    errors = np.linalg.norm(target_hip - kinematic.forward(q), axis=1)
    return q, float(np.max(errors))


def build(stop_after=None):
    if stop_after not in (None, "FRONT_INWARD", "FRONT_REFOOT", "MID_LOW", "BODY_D"):
        raise ValueError("unsupported front prefix endpoint: " + str(stop_after))
    triangles = load_triangles(MESH)
    world_triangles = triangles + TRANSLATION
    kinematic = GraspKinematic()
    anchors = (np.column_stack((kinematic.forward_base(Q_STAND), np.ones(6))) @ world_from_base(P0).T)[:, :3]
    stages, receipts, ground_sphere_receipts, ground_sweep_receipts, surface_sphere_receipts = [], [], [], [], []
    current_pose = P0.copy()
    q_seed = Q_STAND.copy()

    def endpoint_check(name, pose, active):
        nonlocal q_seed
        q_seed, ik_error = solve_ik(kinematic, pose, anchors, q_seed)
        com = (world_from_base(pose) @ np.r_[kinematic.center_of_mass_base(q_seed), 1.])[:3]
        margin = support_margin(com[:2], anchors[[leg for leg in range(6) if leg not in active]])
        receipts.append({"stage": name, "max_ik_endpoint_error_m": ik_error,
                         "support_com_margin_m": margin})
        if ik_error > .025:
            raise ValueError("{} endpoint DLS error {:.3f}m".format(name, ik_error))
        if active and margin < .010:
            raise ValueError("{} support COM margin {:.3f}m".format(name, margin))

    def body(name, target_pose, duration=.70):
        nonlocal current_pose
        stage = {"name": name, "pose_start": current_pose.tolist(),
                 "pose_end": target_pose.tolist(), "active_legs": [],
                 "anchor_knots": [anchors.tolist(), anchors.tolist()],
                 "segment_durations_s": [duration], "anchor_curve": "piecewise_quintic",
                 "pose_curve": "quintic_full_stage", "settle_s": .12}
        stages.append(stage)
        current_pose = target_pose.copy()
        endpoint_check(name, current_pose, [])

    def move_pair(
        name, legs, endpoints, duration=.95, clearance_m=.060,
        segment_durations_s=None, pose_end=None, pose_curve="quintic_full_stage",
    ):
        """Move a pair, optionally synchronized to a full-stage body pose."""
        nonlocal current_pose
        start = anchors.copy()
        end = anchors.copy()
        end[legs] = endpoints
        clearance = max(np.max(start[legs, 2]), np.max(endpoints[:, 2])) + clearance_m
        air_a = start[legs].copy(); air_a[:, 2] = clearance
        air_b = endpoints.copy(); air_b[:, 2] = clearance
        world_knots = [start, start.copy(), start.copy(), end]
        world_knots[1][legs] = air_a
        world_knots[2][legs] = air_b
        durations = (
            [duration * .25, duration * .5, duration * .25]
            if segment_durations_s is None else list(segment_durations_s)
        )
        if len(durations) != 3 or min(durations) <= 0.0:
            raise ValueError(name + " requires three positive segment durations")
        target_pose = current_pose.copy() if pose_end is None else np.asarray(pose_end, dtype=float)
        if target_pose.shape != (6,) or not np.all(np.isfinite(target_pose)):
            raise ValueError(name + " requires a finite six-dimensional pose_end")
        total = float(sum(durations))
        elapsed = np.r_[0.0, np.cumsum(durations)]
        if pose_curve == "quintic_full_stage":
            fractions = elapsed / total
        elif pose_curve == "quintic_first_segment":
            fractions = np.minimum(elapsed / durations[0], 1.0)
        else:
            raise ValueError(name + " has unsupported pose_curve")
        pose_knots = [
            current_pose * (1.0 - quintic_weight(fraction))
            + target_pose * quintic_weight(fraction)
            for fraction in fractions
        ]
        base_knots = [
            base_points(pose, knot[legs])
            for pose, knot in zip(pose_knots, world_knots)
        ]
        is_ground_pair = (np.allclose(start[legs, 2], FOOT_RADIUS, rtol=0.0, atol=1e-12)
                          and np.allclose(end[legs, 2], FOOT_RADIUS, rtol=0.0, atol=1e-12))
        if is_ground_pair:
            sweep = require_ground_pair_sphere_sweep(
                world_triangles, current_pose, target_pose, world_knots, base_knots,
                durations, legs,
            )
            sweep["stage"] = name
            ground_sweep_receipts.append(sweep)
        stage = {"name": name, "pose_start": current_pose.tolist(),
                 "pose_end": target_pose.tolist(), "active_legs": list(legs),
                 "anchor_knots": [knot.tolist() for knot in world_knots],
                 "active_base_knots_m": [knot.tolist() for knot in base_knots],
                 "active_base_velocities_m_s": np.zeros((4, len(legs), 3)).tolist(),
                 "segment_durations_s": durations,
                 "anchor_curve": "piecewise_base_quintic", "pose_curve": pose_curve,
                 "settle_s": .15}
        stages.append(stage)
        anchors[:] = end
        current_pose = target_pose.copy()
        endpoint_check(name, current_pose, legs)

    def ground_targets(legs, x):
        targets = np.array([[x, anchors[leg, 1], FOOT_RADIUS] for leg in legs])
        gaps = [require_ground_sphere_clearance(world_triangles, target) for target in targets]
        ground_sphere_receipts.append({
            "x_m": float(x), "legs": list(legs),
            "per_leg_signed_sphere_gap_m": gaps,
            "min_signed_sphere_gap_m": float(min(gaps)),
        })
        return targets

    def surface_targets(legs, x):
        targets = np.array([
            [x, anchors[leg, 1], surface_height(triangles, x, anchors[leg, 1])]
            for leg in legs
        ])
        gaps = [require_surface_sphere_clearance(world_triangles, target) for target in targets]
        if min(gaps) < -.0005:
            raise ValueError(
                "surface foot sphere enters Xiaolan STL at x {:.3f}: {:.6f}m"
                .format(x, min(gaps))
            )
        surface_sphere_receipts.append({
            "x_m": float(x), "legs": list(legs),
            "per_leg_signed_sphere_gap_m": [float(gap) for gap in gaps],
            "min_signed_sphere_gap_m": float(min(gaps)),
        })
        return targets

    def finish(prefix_preview_only):
        terminal_q = q_seed.copy()
        if not prefix_preview_only:
            terminal_q, terminal_error = solve_ik(
                kinematic, current_pose, anchors, Q_STAND, tolerance_m=.00001,
            )
            if terminal_error > .00001:
                raise ValueError("full front terminal DLS residual {:.6f}m".format(terminal_error))
        config = {
            "schema": COMPACT_SCHEMA, "simulation_only": True, "simulation_candidate_only": True,
            "climb_orientation": "front", "prefix_preview_only": prefix_preview_only,
            "xiaolan_translation": TRANSLATION.tolist(),
            "p0": {"q_rad": Q_STAND.tolist(), "base": P0.tolist(), "anchors_world_m": ((np.column_stack((kinematic.forward_base(Q_STAND), np.ones(6))) @ world_from_base(P0).T)[:, :3]).tolist()},
            "terminal_q_rad": terminal_q.tolist(),
            "stage_count": len(stages), "stages": stages,
            "settle_gate": {"command_tracking_only_not_contact_proof": True, "entry_max_joint_error_rad": .08,
                             "max_foot_target_error_m": .04, "max_joint_tracking_error_rad": .08,
                             "persistence_s": .25, "timeout_s": 5., "preview_time_only_stage_advance": True,
                             "tracking_errors_diagnostic_only": True},
            "surface_receipt": {"mesh": str(MESH.relative_to(ROOT)), "foot_radius_m": FOOT_RADIUS,
                                "landing_margin_policy_m": .012, "endpoint_diagnostics": receipts,
                                "ground_sphere_clearance": ground_sphere_receipts,
                                "surface_sphere_clearance": surface_sphere_receipts,
                                "ground_pair_sweep_clearance": ground_sweep_receipts},
            "visual_validation_deferred_for_sim_finish": [stage["name"] for stage in stages],
        }
        ClimbMode(None)._validate_config(config)
        return config

    body("PREP", np.array((-.12, P0[1], .15, .15, 0., YAW)), 1.0)
    move_pair("FRONT_LOW", PAIR_GROUPS["front"], surface_targets(PAIR_GROUPS["front"], .19), clearance_m=.020,
              segment_durations_s=[1.35, .85, .2375])
    body("BODY_A", np.array((-.03, P0[1], .15, .12, 0., YAW)))
    move_pair("MID_GROUND_SHIFT", PAIR_GROUPS["mid"], ground_targets(PAIR_GROUPS["mid"], .04), clearance_m=.020,
              segment_durations_s=[.475, .95, .475])
    body("BODY_B", np.array((.00, P0[1], .15, .25, 0., YAW)))
    move_pair("FRONT_INWARD", PAIR_GROUPS["front"], surface_targets(PAIR_GROUPS["front"], .29), clearance_m=.020,
              segment_durations_s=[.525, .725, .2375])
    if stop_after == "FRONT_INWARD":
        body("PREVIEW_HOLD", current_pose.copy(), .35)
        return finish(True)

    # A--F are the only post-C6 route accepted for the front prefix preview.
    # Each active base knot uses the body pose at its cumulative segment time.
    move_pair(
        "MID_BACK_ROLL_SYNC", PAIR_GROUPS["mid"],
        ground_targets(PAIR_GROUPS["mid"], -.02), clearance_m=.020,
        segment_durations_s=[.65, .85, .65],
        pose_end=np.array((.00, P0[1], .15, .15, 0., YAW)),
    )
    move_pair(
        "BACK_NEG17_SYNC", PAIR_GROUPS["back"],
        ground_targets(PAIR_GROUPS["back"], -.17), clearance_m=.020,
        segment_durations_s=[.75, .95, .75],
        pose_end=np.array((.01, P0[1], .15, .15, 0., YAW)),
    )
    move_pair(
        "MID_ZERO_SYNC", PAIR_GROUPS["mid"],
        ground_targets(PAIR_GROUPS["mid"], .00), clearance_m=.020,
        segment_durations_s=[.55, .70, .55],
        pose_end=np.array((.02, P0[1], .15, .15, 0., YAW)),
    )
    move_pair(
        "BACK_NEG09_SYNC", PAIR_GROUPS["back"],
        ground_targets(PAIR_GROUPS["back"], -.09), clearance_m=.020,
        segment_durations_s=[.55, .75, .55],
        pose_end=np.array((.029, P0[1], .15, .15, 0., YAW)),
    )
    move_pair(
        "MID_FORWARD_SYNC", PAIR_GROUPS["mid"],
        ground_targets(PAIR_GROUPS["mid"], .08), clearance_m=.020,
        segment_durations_s=[.55, .75, .55],
        pose_end=np.array((.039, P0[1], .15, .15, 0., YAW)),
    )
    move_pair(
        "FRONT_REFOOT", PAIR_GROUPS["front"],
        surface_targets(PAIR_GROUPS["front"], .34), clearance_m=.020,
        segment_durations_s=[.70, .90, .70],
    )
    if stop_after == "FRONT_REFOOT":
        body("PREVIEW_HOLD", current_pose.copy(), .35)
        return finish(True)

    # G--Q extend the accepted C13 prefix by short, alternating ground relays.
    # The G--O poses and timing are the bounded 30 Hz preflight receipt; do not
    # substitute the legacy downstream sketch below for this preview route.
    relay_durations = [.55, .75, .55]
    move_pair(
        "G_MID_BACK_SYNC", PAIR_GROUPS["mid"],
        ground_targets(PAIR_GROUPS["mid"], .02), clearance_m=.020,
        segment_durations_s=relay_durations,
        pose_end=np.array((.049, P0[1], .15, .15, 0., YAW)),
    )
    move_pair(
        "H_BACK_NEG03_SYNC", PAIR_GROUPS["back"],
        ground_targets(PAIR_GROUPS["back"], -.03), clearance_m=.020,
        segment_durations_s=relay_durations,
        pose_end=np.array((.059, P0[1], .15, .15, 0., YAW)),
    )
    move_pair(
        "I_MID_FORWARD_SYNC", PAIR_GROUPS["mid"],
        ground_targets(PAIR_GROUPS["mid"], .08), clearance_m=.020,
        segment_durations_s=relay_durations,
        pose_end=np.array((.065, P0[1], .15, .15, 0., YAW)),
    )
    move_pair(
        "J_MID_BACK_SYNC", PAIR_GROUPS["mid"],
        ground_targets(PAIR_GROUPS["mid"], .04), clearance_m=.020,
        segment_durations_s=relay_durations,
        pose_end=np.array((.071, P0[1], .15, .15, 0., YAW)),
    )
    move_pair(
        "K_BACK_POS03_SYNC", PAIR_GROUPS["back"],
        ground_targets(PAIR_GROUPS["back"], .03), clearance_m=.020,
        segment_durations_s=relay_durations,
        pose_end=np.array((.081, P0[1], .15, .15, 0., YAW)),
    )
    move_pair(
        "L_MID_FORWARD_SYNC", PAIR_GROUPS["mid"],
        ground_targets(PAIR_GROUPS["mid"], .08), clearance_m=.020,
        segment_durations_s=relay_durations,
        pose_end=np.array((.085, P0[1], .15, .15, 0., YAW)),
    )
    move_pair(
        "M_MID_PREP_SYNC", PAIR_GROUPS["mid"],
        ground_targets(PAIR_GROUPS["mid"], .04), clearance_m=.020,
        segment_durations_s=relay_durations,
        pose_end=np.array((.089, P0[1], .15, .15, 0., YAW)),
    )
    move_pair(
        "N_BACK_POS05_SYNC", PAIR_GROUPS["back"],
        ground_targets(PAIR_GROUPS["back"], .05), clearance_m=.020,
        segment_durations_s=relay_durations,
        pose_end=np.array((.098, P0[1], .15, .15, 0., YAW)),
    )
    move_pair(
        "O_MID_FORWARD_SYNC", PAIR_GROUPS["mid"],
        ground_targets(PAIR_GROUPS["mid"], .08), clearance_m=.020,
        segment_durations_s=relay_durations,
        pose_end=np.array((.1017, P0[1], .15, .15, 0., YAW)),
    )
    body("P_BODY_PREP", np.array((.1017, P0[1], .155, .15, 0., YAW)), .60)

    # Q has a locally checked five-knot world path.  Its base knots are reconstructed
    # at each cumulative full-stage pose, with zero interior base velocity.
    q_legs = PAIR_GROUPS["mid"]
    q_start = anchors.copy()
    q_surface = surface_targets(q_legs, .19)
    q_world_knots = [q_start, q_start.copy(), q_start.copy(), q_start.copy(), anchors.copy()]
    q_world_knots[1][q_legs, 2] = .0865
    q_world_knots[2][q_legs] = np.column_stack((np.full(2, .14), q_start[q_legs, 1], np.full(2, .12)))
    q_world_knots[3][q_legs] = np.column_stack((np.full(2, .17), q_start[q_legs, 1], q_surface[:, 2] + .010))
    q_world_knots[4][q_legs] = q_surface
    q_durations = [1.2, 1.0, 1.0, .5]
    q_base_knots = [base_points(current_pose, knot[q_legs]) for knot in q_world_knots]
    q_sweep = require_ground_pair_sphere_sweep(
        world_triangles, current_pose, current_pose, q_world_knots, q_base_knots,
        q_durations, q_legs,
    )
    q_sweep["stage"] = "Q_MID_LOW"
    ground_sweep_receipts.append(q_sweep)
    q_stage = {
        "name": "Q_MID_LOW", "pose_start": current_pose.tolist(),
        "pose_end": current_pose.tolist(), "active_legs": q_legs,
        "anchor_knots": [knot.tolist() for knot in q_world_knots],
        "active_base_knots_m": [knot.tolist() for knot in q_base_knots],
        "active_base_velocities_m_s": np.zeros((5, len(q_legs), 3)).tolist(),
        "segment_durations_s": q_durations,
        "anchor_curve": "piecewise_base_quintic", "pose_curve": "quintic_full_stage",
        "settle_s": .15,
    }
    stages.append(q_stage)
    anchors[:] = q_world_knots[-1]
    endpoint_check("Q_MID_LOW", current_pose, q_legs)
    if stop_after == "MID_LOW":
        body("PREVIEW_HOLD", current_pose.copy(), .35)
        return finish(True)

    # R--AH are the accepted tail: alternating ground relays preserve the
    # continuous controller branch before the final middle/back transfers.
    move_pair("R_FRONT_40_SYNC", PAIR_GROUPS["front"], surface_targets(PAIR_GROUPS["front"], .40),
              clearance_m=.020, segment_durations_s=[.70, .90, .70],
              pose_end=np.array((.11, P0[1], .16, .09, 0., YAW)))
    body("S0_BODY_LIFT", np.array((.11, P0[1], .17, .09, 0., YAW)), .50)
    move_pair("S_MID_20_SYNC", PAIR_GROUPS["mid"], surface_targets(PAIR_GROUPS["mid"], .20),
              clearance_m=.015, segment_durations_s=[.70, .90, .70],
              pose_end=np.array((.1233, P0[1], .17, 0., 0., YAW)))
    move_pair("LB_RESET_NEG03", [0], ground_targets([0], -.03), clearance_m=.020,
              segment_durations_s=[.50, .70, .50])
    move_pair("RB_RESET_NEG03", [3], ground_targets([3], -.03), clearance_m=.020,
              segment_durations_s=[.50, .70, .50])
    body("T0_BODY_LIFT", np.array((.1233, P0[1], .18, 0., 0., YAW)), .50)
    move_pair("T_MID_21_SYNC", PAIR_GROUPS["mid"], surface_targets(PAIR_GROUPS["mid"], .21),
              clearance_m=.015, segment_durations_s=[.70, .90, .70],
              pose_end=np.array((.137, P0[1], .18, 0., 0., YAW)))
    move_pair("U_LB_ZERO_SYNC", [0], ground_targets([0], .00), clearance_m=.020,
              segment_durations_s=[.55, .75, .55], pose_end=np.array((.145, P0[1], .18, 0., 0., YAW)))
    move_pair("V_RB_ZERO_SYNC", [3], ground_targets([3], .00), clearance_m=.020,
              segment_durations_s=[.55, .75, .55], pose_end=np.array((.153, P0[1], .185, 0., 0., YAW)))
    move_pair("W_MID_21_RELAND_SYNC", PAIR_GROUPS["mid"], surface_targets(PAIR_GROUPS["mid"], .21),
              clearance_m=.010, segment_durations_s=[.70, .90, .70],
              pose_end=np.array((.165, P0[1], .19, 0., 0., YAW)))
    move_pair("X_LB_035_SYNC", [0], ground_targets([0], .035), clearance_m=.020,
              segment_durations_s=[.55, .75, .55], pose_end=np.array((.173, P0[1], .19, 0., 0., YAW)))
    move_pair("Y_RB_035_SYNC", [3], ground_targets([3], .035), clearance_m=.020,
              segment_durations_s=[.55, .75, .55], pose_end=np.array((.181, P0[1], .19, 0., 0., YAW)))
    move_pair("Z_FRONT_48_SYNC", PAIR_GROUPS["front"], surface_targets(PAIR_GROUPS["front"], .48),
              clearance_m=.020, segment_durations_s=[.70, .90, .70],
              pose_end=np.array((.1885, P0[1], .19, 0., 0., YAW)))
    relay = [.55, .75, .55]
    move_pair("AA_LB_055_SYNC", [0], ground_targets([0], .055), clearance_m=.020,
              segment_durations_s=relay, pose_end=np.array((.1965, P0[1], .19, 0., 0., YAW)))
    move_pair("AB_RB_055_SYNC", [3], ground_targets([3], .055), clearance_m=.020,
              segment_durations_s=relay, pose_end=np.array((.2045, P0[1], .19, 0., 0., YAW)))
    move_pair("AC_LB_075_SYNC", [0], ground_targets([0], .075), clearance_m=.020,
              segment_durations_s=relay, pose_end=np.array((.2125, P0[1], .195, 0., 0., YAW)))
    move_pair("AD_RB_075_SYNC", [3], ground_targets([3], .075), clearance_m=.020,
              segment_durations_s=[.790625, 1.078125, .790625], pose_end=np.array((.2205, P0[1], .195, 0., 0., YAW)))
    move_pair("AE_LB_090_SYNC", [0], ground_targets([0], .09), clearance_m=.020,
              segment_durations_s=relay, pose_end=np.array((.2285, P0[1], .195, 0., 0., YAW)))
    move_pair("AF_RB_090_SYNC", [3], ground_targets([3], .09), clearance_m=.020,
              segment_durations_s=relay, pose_end=np.array((.2365, P0[1], .195, 0., 0., YAW)))
    move_pair("AG_MID_21_RELAND_SYNC", PAIR_GROUPS["mid"], surface_targets(PAIR_GROUPS["mid"], .21),
              clearance_m=.010, segment_durations_s=[.70, .90, .70], pose_end=np.array((.245, P0[1], .195, 0., 0., YAW)))
    move_pair("AGa_LB_11_STATIC_BODY", [0], ground_targets([0], .11), clearance_m=.020,
              segment_durations_s=[.825, 1.125, .825])
    move_pair("AGb_RB_11_SYNC", [3], ground_targets([3], .11), clearance_m=.020,
              segment_durations_s=relay, pose_end=np.array((.253, P0[1], .195, 0., 0., YAW)))
    move_pair("AGc_LB_13_SYNC", [0], ground_targets([0], .13), clearance_m=.020,
              segment_durations_s=relay, pose_end=np.array((.261, P0[1], .195, 0., 0., YAW)))
    move_pair("AGd_RB_13_SYNC", [3], ground_targets([3], .13), clearance_m=.020,
              segment_durations_s=relay, pose_end=np.array((.269, P0[1], .195, 0., 0., YAW)))
    move_pair("AG2_MID_235_SYNC", PAIR_GROUPS["mid"], surface_targets(PAIR_GROUPS["mid"], .235),
              clearance_m=.005, segment_durations_s=[.70, .90, .70], pose_end=np.array((.277, P0[1], .195, 0., 0., YAW)))
    if stop_after == "BODY_D":
        body("PREVIEW_HOLD", current_pose.copy(), .35)
        return finish(True)
    move_pair("AH_BACK_LOW_FIRST_SEGMENT", PAIR_GROUPS["back"], surface_targets(PAIR_GROUPS["back"], .19),
              clearance_m=.015, segment_durations_s=[1.40, 1.00, .60],
              pose_end=np.array((.285, P0[1], .245, 0., 0., YAW)),
              pose_curve="quintic_first_segment")
    body("FINAL_HOLD", current_pose.copy(), 2.0)

    return finish(False)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stop-after", choices=("FRONT_INWARD", "FRONT_REFOOT", "MID_LOW", "BODY_D"))
    args = parser.parse_args()
    config = build(stop_after=args.stop_after)
    args.output.write_text(json.dumps(config, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
