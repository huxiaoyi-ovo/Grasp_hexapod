
from pathlib import Path
from isaacgym import gymapi
from control import (
    build_dof_indices,
    isaac_to_control,
    GraspController,
    control_to_isaac
)
import numpy as np


def print_model_info(gym, env, actor) :
    # 后续控制器要靠名称建立索引，不能默认Isaac Gym的数组顺序
    rigid_body_names = gym.get_actor_rigid_body_names(env, actor)
    dof_names = gym.get_actor_dof_names(env, actor)
    print(f"Robot loaded: {len(rigid_body_names)} rigid bodies, {len(dof_names)} DOFs")

    return dof_names  # 主循环需要用它解释q_cur中每个元素的含义



def main() -> None:
    # get the asset path
    repo_root = Path(__file__).resolve().parents[1]
    gym = gymapi.acquire_gym()
    # create a simulator
    sim_params = gymapi.SimParams() # create a sim params object

    sim_params.dt = 1 / 60.0 # set the time step to 1/60 seconds
    sim_params.substeps = 2 # set the number of substeps to 2
    sim_params.use_gpu_pipeline = False # set the use GPU pipeline to False
    sim_params.up_axis = gymapi.UP_AXIS_Z
    sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)
    # TGS求解器， 0: PGS, 1: TGS, 2: TGS with warm start
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 8
    sim_params.physx.num_velocity_iterations = 2

    # create the simulation
    sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)
    if sim is None:
        raise RuntimeError("Failed to create Isaac Gym simulation.")
    #创建地面和viewer
    plane = gymapi.PlaneParams()
    plane.normal = gymapi.Vec3(0.0, 0.0, 1.0)
    plane.static_friction = 1.5
    plane.dynamic_friction = 1.2

    gym.add_ground(sim, plane)

    viewer = gym.create_viewer(sim, gymapi.CameraProperties())
    if viewer is None:
        raise RuntimeError("Failed to create Isaac Gym viewer.")
    #加载urdf
    asset_root = str(repo_root)
    asset_file = "urdf/hexapod_isaacgym_view.urdf"
    asset_options = gymapi.AssetOptions()
    asset_options.fix_base_link = True
    asset_options.collapse_fixed_joints = False
    asset_options.default_dof_drive_mode = int(gymapi.DOF_MODE_POS) #有三种模式：位置、速度、力矩
    asset_options.use_mesh_materials = True
    
    robot_asset = gym.load_asset(sim, asset_root, asset_file, asset_options)
    if robot_asset is None:
        raise RuntimeError("Failed to load robot asset.")

    lower = gymapi.Vec3(-1.0, -1.0, 0.0)
    upper = gymapi.Vec3(1.0, 1.0, 1.0)
    num_per_row = 1
    #创建环境和actor
    env = gym.create_env(sim, lower, upper, num_per_row)

    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0.0, 0.0, 0.075)

    actor = gym.create_actor(env, robot_asset, pose, "grasp_hexapod", 0, 1)

    dof_names = print_model_info(gym, env, actor)
    dof_indices = build_dof_indices(dof_names)
    print(f"Control DOF mapping ready: {len(dof_indices)} joints")

    controller = GraspController(dt=sim_params.dt)
    dof_properties = gym.get_actor_dof_properties(env, actor)
    dof_properties["driveMode"].fill(int(gymapi.DOF_MODE_POS))
    dof_properties["stiffness"].fill(100.0)
    dof_properties["damping"].fill(0.8)

    gym.set_actor_dof_properties(
        env,
        actor,
        dof_properties,
    )
    #做一系列控制器和isaac的顺序转换
    lower_control = isaac_to_control(
        dof_properties["lower"],
        dof_indices,
    )
    upper_control = isaac_to_control(
        dof_properties["upper"],
        dof_indices,
    )
    velocity_control = isaac_to_control(
        dof_properties["velocity"],
        dof_indices,
    )
    q_init_isaac = control_to_isaac(
        controller.q_init,
        dof_indices
    )
    dof_states = gym.get_actor_dof_states(env, actor, gymapi.STATE_ALL)
    dof_states["pos"][:] = q_init_isaac
    dof_states["vel"][:] = 0.0

    gym.set_actor_dof_states(
        env,
        actor,
        dof_states,
        gymapi.STATE_ALL,
    )
    # 位置驱动的目标也必须同时设置成Q_INIT
    gym.set_actor_dof_position_targets(
        env,
        actor,
        q_init_isaac,
    )

    gym.viewer_camera_look_at(viewer, None, gymapi.Vec3(0.55, -0.75, 0.45), gymapi.Vec3(0.0, 0.0, 0.15))

    #状态读取主循环
    while not gym.query_viewer_has_closed(viewer):
        dof_states = gym.get_actor_dof_states(env, actor, gymapi.STATE_ALL)
        # Isaac一维顺序 → 控制器(6,3)顺序
        q_control = isaac_to_control(
            dof_states["pos"],
            dof_indices,
        )
        q_dot_control = isaac_to_control(
            dof_states["vel"],
            dof_indices,
        )
         # 足端位置闭环计算关节目标
        q_des_control = controller.cal_joint_poses(
            q_control,
            q_dot_control,
        )
         # 限制单个控制周期内允许变化的最大关节角
        max_joint_step = velocity_control * sim_params.dt
        q_target_control = (
            q_control
            + np.clip(
                q_des_control - q_control,
                -max_joint_step,
                max_joint_step,
            )
        )
        # 限制在URDF机械角度范围内
        q_target_control = np.clip(
            q_target_control,
            lower_control,
            upper_control,
        )
         # 控制器顺序 → Isaac Gym顺序
        q_target_isaac = control_to_isaac(
            q_target_control,
            dof_indices,
        )
        # 把位置目标真正发送给关节驱动器
        gym.set_actor_dof_position_targets(
            env,
            actor,
            q_target_isaac,
        )

        
        gym.simulate(sim)
        gym.fetch_results(sim, True)
        gym.step_graphics(sim)
        gym.draw_viewer(viewer, sim, True)
        gym.sync_frame_time(sim)

    gym.destroy_viewer(viewer)
    gym.destroy_sim(sim)


if __name__ == "__main__":
    main()
