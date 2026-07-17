
from pathlib import Path
from isaacgym import gymapi





def main() -> None:

    gym = gymapi.acquire_gym()
    sim_params = gymapi.SimParams() # create a sim params object

    sim_params.dt = 1 / 60.0 # set the time step to 1/60 seconds
    sim_params.substeps = 2 # set the number of substeps to 2
    sim_params.use_gpu_pipeline = False # set the use GPU pipeline to False
    sim_params.up_axis = gymapi.UP_AXIS_Z
    sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)
    sim_params.physx.solver_type = 1

    sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)
    if sim is None:
        raise RuntimeError("Failed to create Isaac Gym simulation.")
    
    plane = gymapi.PlaneParams()
    plane.normal = gymapi.Vec3(0.0, 0.0, 1.0)
    gym.add_ground(sim, plane)

    viewer = gym.create_viewer(sim, gymapi.CameraProperties())
    if viewer is None:
        raise RuntimeError("Failed to create Isaac Gym viewer.")
    # get the asset path
    repo_root = Path(__file__).resolve().parents[1]
    asset_root = str(repo_root)
    asset_file = "urdf/hexapod_isaacgym_view.urdf"
    asset_options = gymapi.AssetOptions()
    asset_options.fix_base_link = True
    asset_options.collapse_fixed_joints = False
    asset_options.default_dof_drive_mode = gymapi.DOF_MODE_POS
    asset_options.use_mesh_materials = True
    
    robot_asset = gym.load_asset(sim, asset_root, asset_file, asset_options)
    if robot_asset is None:
        raise RuntimeError("Failed to load robot asset.")

    lower = gymapi.Vec3(-1.0, -1.0, 0.0)
    upper = gymapi.Vec3(1.0, 1.0, 1.0)
    num_per_row = 1
    env = gym.create_env(sim, lower, upper, num_per_row)

    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0.0, 0.0, 0.15)

    actor = gym.create_actor(env, robot_asset, pose, "grasp_hexapod", 0, 1)

    dof_names = gym.get_actor_dof_names(env, actor)
    print(f"DOFs ({len(dof_names)}): {', '.join(dof_names)}")
    dof_count = gym.get_actor_dof_count(env, actor)
    print(f"DOF count: {dof_count}")

    gym.viewer_camera_look_at(viewer, None, gymapi.Vec3(0.55, -0.75, 0.45), gymapi.Vec3(0.0, 0.0, 0.15))

    while not gym.query_viewer_has_closed(viewer):
        gym.simulate(sim)
        gym.fetch_results(sim, True)
        gym.step_graphics(sim)
        gym.draw_viewer(viewer, sim, True)
        gym.sync_frame_time(sim)
    gym.destroy_viewer(viewer)
    gym.destroy_sim(sim)


if __name__ == "__main__":
    main()
