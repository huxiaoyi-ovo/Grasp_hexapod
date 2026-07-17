#!/usr/bin/env python3
"""Open this hexapod URDF in the Isaac Gym viewer."""

from __future__ import annotations

import argparse
from pathlib import Path

from isaacgym import gymapi


STAND_TARGETS = {
    "lf_thigh_joint": 0.0,
    "lf_knee_joint": -0.5,
    "lf_ankle_joint": 0.8,
    "lm_thigh_joint": 0.0,
    "lm_knee_joint": -0.5,
    "lm_ankle_joint": 0.8,
    "lb_thigh_joint": 0.0,
    "lb_knee_joint": -0.5,
    "lb_ankle_joint": 0.8,
    "rf_thigh_joint": 0.0,
    "rf_knee_joint": 0.5,
    "rf_ankle_joint": -0.8,
    "rm_thigh_joint": 0.0,
    "rm_knee_joint": 0.5,
    "rm_ankle_joint": -0.8,
    "rb_thigh_joint": 0.0,
    "rb_knee_joint": 0.5,
    "rb_ankle_joint": -0.8,
}


def build_view_urdf(repo_root: Path, source_urdf: Path) -> Path:
    """Create an Isaac Gym friendly URDF with relative mesh paths."""
    text = source_urdf.read_text(encoding="utf-8")
    text = text.replace("package://hexapod_robot/meshes/", "../meshes/")
    text = text.replace("package://抓取机器人export_urdf.SLDASM/meshes/", "../meshes/")

    output = repo_root / "urdf" / "hexapod_isaacgym_view.urdf"
    output.write_text(text, encoding="utf-8")
    return output


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="View the hexapod URDF in Isaac Gym.")
    parser.add_argument(
        "--source-urdf",
        default="urdf/hexapod_collision.urdf",
        help="URDF path relative to the repository root.",
    )
    parser.add_argument("--compute-device-id", type=int, default=0)
    parser.add_argument("--graphics-device-id", type=int, default=0)
    parser.add_argument("--fix-base", action="store_true", help="Fix the base link for static inspection.")
    parser.add_argument("--unlock-motors", action="store_true", help="Disable position-servo motor holding.")
    parser.add_argument("--motor-stiffness", type=float, default=1000.0, help="Position drive stiffness.")
    parser.add_argument("--motor-damping", type=float, default=100.0, help="Position drive damping.")
    parser.add_argument("--motor-effort", type=float, default=100.0, help="Maximum motor effort.")
    parser.add_argument(
        "--initial-pose",
        choices=("stand", "zero"),
        default="stand",
        help="Initial joint target pose.",
    )
    parser.add_argument(
        "--spawn-height",
        type=float,
        default=0.10,
        help="Initial base_link height above the ground.",
    )
    parser.add_argument("--no-ground", action="store_true", help="Do not add the ground plane.")
    return parser.parse_args()


def fill_initial_pose(dof_names: list[str], dof_positions, pose_name: str) -> None:
    dof_positions.fill(0.0)
    if pose_name == "zero":
        return

    for index, name in enumerate(dof_names):
        dof_positions[index] = STAND_TARGETS.get(name, 0.0)


def main() -> None:
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    source_urdf = repo_root / args.source_urdf
    if not source_urdf.exists():
        raise FileNotFoundError(f"URDF not found: {source_urdf}")

    view_urdf = build_view_urdf(repo_root, source_urdf)

    gym = gymapi.acquire_gym()
    sim_params = gymapi.SimParams()
    sim_params.dt = 1.0 / 60.0
    sim_params.up_axis = gymapi.UP_AXIS_Z
    sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 8
    sim_params.physx.num_velocity_iterations = 1

    sim = gym.create_sim(
        args.compute_device_id,
        args.graphics_device_id,
        gymapi.SIM_PHYSX,
        sim_params,
    )
    if sim is None:
        raise RuntimeError("Failed to create Isaac Gym simulation.")

    if not args.no_ground:
        plane = gymapi.PlaneParams()
        plane.normal = gymapi.Vec3(0.0, 0.0, 1.0)
        gym.add_ground(sim, plane)

    viewer = gym.create_viewer(sim, gymapi.CameraProperties())
    if viewer is None:
        raise RuntimeError("Failed to create Isaac Gym viewer.")

    asset_options = gymapi.AssetOptions()
    asset_options.fix_base_link = args.fix_base
    asset_options.collapse_fixed_joints = False
    asset_options.default_dof_drive_mode = (
        gymapi.DOF_MODE_NONE if args.unlock_motors else gymapi.DOF_MODE_POS
    )
    asset_options.use_mesh_materials = True

    robot_asset = gym.load_asset(
        sim,
        str(repo_root),
        str(view_urdf.relative_to(repo_root)),
        asset_options,
    )
    if robot_asset is None:
        raise RuntimeError(f"Failed to load asset: {view_urdf}")

    env = gym.create_env(
        sim,
        gymapi.Vec3(-1.0, -1.0, 0.0),
        gymapi.Vec3(1.0, 1.0, 1.0),
        1,
    )
    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0.0, 0.0, args.spawn_height)
    actor = gym.create_actor(env, robot_asset, pose, "hexapod", 0, 1)

    dof_props = gym.get_actor_dof_properties(env, actor)
    dof_names = gym.get_actor_dof_names(env, actor)
    hold_targets = None
    if len(dof_props) > 0 and not args.unlock_motors:
        dof_props["driveMode"].fill(gymapi.DOF_MODE_POS)
        dof_props["stiffness"].fill(args.motor_stiffness)
        dof_props["damping"].fill(args.motor_damping)
        if "effort" in dof_props.dtype.names:
            dof_props["effort"].fill(args.motor_effort)
        gym.set_actor_dof_properties(env, actor, dof_props)

        dof_states = gym.get_actor_dof_states(env, actor, gymapi.STATE_ALL)
        fill_initial_pose(dof_names, dof_states["pos"], args.initial_pose)
        dof_states["vel"].fill(0.0)
        gym.set_actor_dof_states(env, actor, dof_states, gymapi.STATE_ALL)
        hold_targets = dof_states["pos"].copy()
        gym.set_actor_dof_position_targets(env, actor, hold_targets)

    gym.viewer_camera_look_at(
        viewer,
        None,
        gymapi.Vec3(0.55, -0.75, 0.45),
        gymapi.Vec3(0.0, 0.0, 0.08),
    )

    print(f"Loaded {view_urdf}")
    print(f"DOFs ({len(dof_names)}): {', '.join(dof_names)}")
    if hold_targets is not None:
        print(
            f"Motors locked in {args.initial_pose!r} position mode "
            f"(stiffness={args.motor_stiffness}, damping={args.motor_damping}, "
            f"effort={args.motor_effort})."
        )
    print("Close the Isaac Gym viewer window to exit.")

    while not gym.query_viewer_has_closed(viewer):
        if hold_targets is not None:
            gym.set_actor_dof_position_targets(env, actor, hold_targets)
        gym.simulate(sim)
        gym.fetch_results(sim, True)
        gym.step_graphics(sim)
        gym.draw_viewer(viewer, sim, True)
        gym.sync_frame_time(sim)

    gym.destroy_viewer(viewer)
    gym.destroy_sim(sim)


if __name__ == "__main__":
    main()
