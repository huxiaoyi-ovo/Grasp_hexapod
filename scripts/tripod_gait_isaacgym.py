#!/usr/bin/env python3
"""Run a simple open-loop tripod gait in Isaac Gym."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from isaacgym import gymapi

from view_isaacgym import STAND_TARGETS, build_view_urdf


TRIPOD_A = {"lf", "rm", "lb"}
TRIPOD_B = {"rf", "lm", "rb"}

# Sign that moves each foot roughly toward body +Y during swing.
HIP_FORWARD_SIGNS = {
    "lf": 1.0,
    "lm": 1.0,
    "lb": 1.0,
    "rf": -1.0,
    "rm": -1.0,
    "rb": -1.0,
}

KNEE_LIFT_SIGNS = {"lf": -1.0, "lm": -1.0, "lb": -1.0, "rf": 1.0, "rm": 1.0, "rb": 1.0}
ANKLE_LIFT_SIGNS = {"lf": -1.0, "lm": -1.0, "lb": -1.0, "rf": 1.0, "rm": 1.0, "rb": 1.0}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a simple tripod gait in Isaac Gym.")
    parser.add_argument("--source-urdf", default="urdf/hexapod_collision.urdf")
    parser.add_argument("--compute-device-id", type=int, default=0)
    parser.add_argument("--graphics-device-id", type=int, default=0)
    parser.add_argument("--spawn-height", type=float, default=0.10)
    parser.add_argument("--warmup-time", type=float, default=1.0)
    parser.add_argument("--period", type=float, default=1.0, help="Tripod gait period in seconds.")
    parser.add_argument("--hip-amplitude", type=float, default=0.25, help="Thigh swing amplitude in radians.")
    parser.add_argument("--knee-lift", type=float, default=0.28, help="Extra knee bend during swing.")
    parser.add_argument("--ankle-lift", type=float, default=0.25, help="Extra ankle bend during swing.")
    parser.add_argument("--motor-stiffness", type=float, default=1200.0)
    parser.add_argument("--motor-damping", type=float, default=120.0)
    parser.add_argument("--motor-effort", type=float, default=150.0)
    parser.add_argument("--reverse", action="store_true", help="Reverse the walking direction.")
    parser.add_argument("--fix-base", action="store_true", help="Debug gait with fixed base.")
    return parser.parse_args()


def stand_vector(dof_names: list[str]):
    targets = []
    for name in dof_names:
        targets.append(STAND_TARGETS.get(name, 0.0))
    return targets


def gait_target_for_leg(leg: str, local_phase: float, args: argparse.Namespace) -> dict[str, float]:
    """Return joint offsets from standing pose for one leg."""
    direction = -1.0 if args.reverse else 1.0
    swing = local_phase < 0.5
    if swing:
        progress = local_phase / 0.5
        sweep = -args.hip_amplitude + 2.0 * args.hip_amplitude * progress
        lift = math.sin(math.pi * progress)
    else:
        progress = (local_phase - 0.5) / 0.5
        sweep = args.hip_amplitude - 2.0 * args.hip_amplitude * progress
        lift = 0.0

    hip = direction * HIP_FORWARD_SIGNS[leg] * sweep
    knee = KNEE_LIFT_SIGNS[leg] * args.knee_lift * lift
    ankle = ANKLE_LIFT_SIGNS[leg] * args.ankle_lift * lift
    return {
        f"{leg}_thigh_joint": hip,
        f"{leg}_knee_joint": knee,
        f"{leg}_ankle_joint": ankle,
    }


def build_gait_targets(dof_names: list[str], stand_targets, gait_time: float, args: argparse.Namespace):
    phase = (gait_time / args.period) % 1.0
    target_by_name = {name: stand_targets[index] for index, name in enumerate(dof_names)}

    for leg in sorted(TRIPOD_A | TRIPOD_B):
        leg_phase = phase if leg in TRIPOD_A else (phase + 0.5) % 1.0
        for joint_name, offset in gait_target_for_leg(leg, leg_phase, args).items():
            if joint_name in target_by_name:
                target_by_name[joint_name] += offset

    targets = stand_targets.copy()
    for index, name in enumerate(dof_names):
        targets[index] = target_by_name[name]
    return targets


def main() -> None:
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    view_urdf = build_view_urdf(repo_root, repo_root / args.source_urdf)

    gym = gymapi.acquire_gym()
    sim_params = gymapi.SimParams()
    sim_params.dt = 1.0 / 60.0
    sim_params.up_axis = gymapi.UP_AXIS_Z
    sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 8
    sim_params.physx.num_velocity_iterations = 2

    sim = gym.create_sim(args.compute_device_id, args.graphics_device_id, gymapi.SIM_PHYSX, sim_params)
    if sim is None:
        raise RuntimeError("Failed to create Isaac Gym simulation.")

    plane = gymapi.PlaneParams()
    plane.normal = gymapi.Vec3(0.0, 0.0, 1.0)
    plane.static_friction = 1.5
    plane.dynamic_friction = 1.2
    gym.add_ground(sim, plane)

    viewer = gym.create_viewer(sim, gymapi.CameraProperties())
    if viewer is None:
        raise RuntimeError("Failed to create Isaac Gym viewer.")

    asset_options = gymapi.AssetOptions()
    asset_options.fix_base_link = args.fix_base
    asset_options.collapse_fixed_joints = False
    asset_options.default_dof_drive_mode = gymapi.DOF_MODE_POS
    asset_options.use_mesh_materials = True
    asset_options.armature = 0.01

    robot_asset = gym.load_asset(sim, str(repo_root), str(view_urdf.relative_to(repo_root)), asset_options)
    if robot_asset is None:
        raise RuntimeError(f"Failed to load asset: {view_urdf}")

    env = gym.create_env(sim, gymapi.Vec3(-1.0, -1.0, 0.0), gymapi.Vec3(1.0, 1.0, 1.0), 1)
    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0.0, 0.0, args.spawn_height)
    actor = gym.create_actor(env, robot_asset, pose, "hexapod", 0, 1)

    dof_names = gym.get_actor_dof_names(env, actor)
    dof_props = gym.get_actor_dof_properties(env, actor)
    dof_props["driveMode"].fill(gymapi.DOF_MODE_POS)
    dof_props["stiffness"].fill(args.motor_stiffness)
    dof_props["damping"].fill(args.motor_damping)
    if "effort" in dof_props.dtype.names:
        dof_props["effort"].fill(args.motor_effort)
    gym.set_actor_dof_properties(env, actor, dof_props)

    dof_states = gym.get_actor_dof_states(env, actor, gymapi.STATE_ALL)
    stand_targets = dof_states["pos"].copy()
    for index, value in enumerate(stand_vector(dof_names)):
        stand_targets[index] = value
    dof_states["pos"][:] = stand_targets
    dof_states["vel"].fill(0.0)
    gym.set_actor_dof_states(env, actor, dof_states, gymapi.STATE_ALL)
    gym.set_actor_dof_position_targets(env, actor, stand_targets)

    gym.viewer_camera_look_at(
        viewer,
        None,
        gymapi.Vec3(0.65, -0.9, 0.45),
        gymapi.Vec3(0.0, 0.0, 0.08),
    )

    print(f"Loaded {view_urdf}")
    print(f"DOFs ({len(dof_names)}): {', '.join(dof_names)}")
    print("Tripod A: lf, rm, lb")
    print("Tripod B: rf, lm, rb")
    print("Close the Isaac Gym viewer window to exit.")

    sim_time = 0.0
    while not gym.query_viewer_has_closed(viewer):
        if sim_time < args.warmup_time:
            targets = stand_targets
        else:
            targets = build_gait_targets(dof_names, stand_targets, sim_time - args.warmup_time, args)

        gym.set_actor_dof_position_targets(env, actor, targets)
        gym.simulate(sim)
        gym.fetch_results(sim, True)
        gym.step_graphics(sim)
        gym.draw_viewer(viewer, sim, True)
        gym.sync_frame_time(sim)
        sim_time += sim_params.dt

    gym.destroy_viewer(viewer)
    gym.destroy_sim(sim)


if __name__ == "__main__":
    main()
