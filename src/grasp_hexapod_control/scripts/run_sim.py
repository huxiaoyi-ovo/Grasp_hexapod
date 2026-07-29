#!/usr/bin/env python3
"""Isaac Gym仿真运行入口。

功能：
    配置CUDA/PhysX，加载六足、小蓝和地面，读取手柄与仿真关节状态，
    调用GraspController，并把18个关节位置目标发送给Isaac Gym。
输入：
    手柄归一化指令；Isaac Gym关节状态，内部转换为q_cur.shape=(6,3)，单位rad。
输出：
    Isaac顺序的18关节位置目标；viewer画面和必要的启动状态信息。
结构：
    创建仿真与资产 -> 建立关节顺序映射 -> 主控制循环 -> 资源释放。
约定：
    base_link中+x向右、+y向前、+z向上；长度单位m，角度单位rad。
"""

from pathlib import Path
import struct
import time

from isaacgym import gymapi
import numpy as np
import pygame

from control import GraspController
from utils import (
    build_dof_indices,
    control_to_external,
    external_to_control,
)


class JoyStick:
    """仿真手柄输入；输出归一化的右移、前进、升降和偏航指令。"""

    def __init__(self):
        pygame.init()
        pygame.joystick.init()

        printed_waiting = False
        while pygame.joystick.get_count() == 0:
            pygame.event.pump()
            if not printed_waiting:
                print("Waiting for joystick to connect...")
                printed_waiting = True
            time.sleep(0.2)

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"Joystick connected: {self.joystick.get_name()}")

    @staticmethod
    def _deadzone(value):
        return 0.0 if abs(value) < 0.1 else value

    def get_commands(self):
        pygame.event.pump()
        reset = self.joystick.get_button(1)
        axis_right = self._deadzone(self.joystick.get_axis(0))
        axis_forward = self._deadzone(-self.joystick.get_axis(1))
        axis_up = self._deadzone(-self.joystick.get_axis(4))
        axis_yaw = self._deadzone(-self.joystick.get_axis(3))
        return reset, axis_right, axis_forward, axis_up, axis_yaw


def add_static_stl_triangle_mesh(gym, sim, mesh_path, position):
    """把二进制STL原始三角面作为静态PhysX碰撞面加入仿真。"""
    data = mesh_path.read_bytes()
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    record_dtype = np.dtype(
        [
            ("normal", "<f4", (3,)),
            ("vertices", "<f4", (3, 3)),
            ("attribute", "<u2"),
        ]
    )
    expected_size = 84 + triangle_count * record_dtype.itemsize
    if len(data) != expected_size:
        raise ValueError(f"Only binary STL is supported: {mesh_path}")

    records = np.frombuffer(
        data,
        dtype=record_dtype,
        count=triangle_count,
        offset=84,
    )
    vertices = np.ascontiguousarray(
        records["vertices"].reshape(-1, 3)
    )
    triangles = np.arange(
        triangle_count * 3,
        dtype=np.uint32,
    ).reshape(-1, 3)

    mesh_params = gymapi.TriangleMeshParams()
    mesh_params.nb_vertices = vertices.shape[0]
    mesh_params.nb_triangles = triangles.shape[0]
    mesh_params.transform.p = position
    mesh_params.static_friction = 1.0
    mesh_params.dynamic_friction = 0.8

    gym.add_triangle_mesh(
        sim,
        vertices.ravel(),
        triangles.ravel(),
        mesh_params,
    )
    print(
        f"Xiaolan loaded: {triangle_count} exact static collision triangles"
    )


def print_model_info(gym, env, actor) :
    # 后续控制器要靠名称建立索引，不能默认Isaac Gym的数组顺序
    rigid_body_names = gym.get_actor_rigid_body_names(env, actor)
    dof_names = gym.get_actor_dof_names(env, actor)
    print(f"Robot loaded: {len(rigid_body_names)} rigid bodies, {len(dof_names)} DOFs")

    return dof_names  # 主循环需要用它解释q_cur中每个元素的含义



def main() -> None:
    # 仿真从源码树直接运行，不依赖ROS_PACKAGE_PATH或source devel/setup.bash。
    description_root = (
        Path(__file__).resolve().parents[2]
        / "grasp_hexapod_description"
    )
    gym = gymapi.acquire_gym()
    # create a simulator
    sim_params = gymapi.SimParams() # create a sim params object

    sim_params.dt = 1 / 60.0 # set the time step to 1/60 seconds
    sim_params.substeps = 2 # set the number of substeps to 2
    # PhysX明确使用cuda:0计算；控制器仍使用NumPy，因此保留CPU数据管线。
    sim_params.use_gpu_pipeline = False
    sim_params.up_axis = gymapi.UP_AXIS_Z
    sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)
    # TGS求解器， 0: PGS, 1: TGS, 2: TGS with warm start
    sim_params.physx.use_gpu = True
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 8
    sim_params.physx.num_velocity_iterations = 2

    # create the simulation
    compute_device_id = 0
    graphics_device_id = 0
    sim = gym.create_sim(
        compute_device_id,
        graphics_device_id,
        gymapi.SIM_PHYSX,
        sim_params,
    )
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
    asset_root = str(description_root)
    asset_file = "urdf/hexapod_isaacgym_view.urdf"
    asset_options = gymapi.AssetOptions()
    asset_options.fix_base_link = False
    asset_options.collapse_fixed_joints = False
    asset_options.use_mesh_materials = True

    robot_asset = gym.load_asset(sim, asset_root, asset_file, asset_options)
    if robot_asset is None:
        raise RuntimeError("Failed to load robot asset.")

    # 小蓝固定在六足的+y前方，直接使用原始三角面而不是凸包近似。
    # PhysX只允许静态物体使用这种凹三角网格，正适合当前固定的对接目标。
    add_static_stl_triangle_mesh(
        gym,
        sim,
        description_root
        / "meshes"
        / "xiaolan"
        / "base_link_xiaolan.STL",
        gymapi.Vec3(0.0, 0.8, 0.0),
    )

    lower = gymapi.Vec3(-1.0, -1.0, 0.0)
    upper = gymapi.Vec3(1.0, 1.0, 1.0)
    num_per_row = 1

    controller = GraspController(dt=sim_params.dt)
    #创建环境和actor
    env = gym.create_env(sim, lower, upper, num_per_row)

    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(
        0.0,
        0.0,
        # 直接按标准站姿的足端球半径落在地面上，不再额外悬空25 mm。
        float(controller.base_height_at_stand),
    )

    actor = gym.create_actor(env, robot_asset, pose, "grasp_hexapod", 0, 1)

    dof_names = print_model_info(gym, env, actor)
    dof_indices = build_dof_indices(dof_names)
    print(f"Control DOF mapping ready: {len(dof_indices)} joints")

    dof_properties = gym.get_actor_dof_properties(env, actor)
    dof_properties["driveMode"].fill(int(gymapi.DOF_MODE_POS))
    dof_properties["stiffness"].fill(100.0) #kp
    dof_properties["damping"].fill(0.8) #kd

    gym.set_actor_dof_properties(
        env,
        actor,
        dof_properties,
    )
    #做一系列控制器和isaac的顺序转换
    lower_control = external_to_control(
        dof_properties["lower"],
        dof_indices,
    )
    upper_control = external_to_control(
        dof_properties["upper"],
        dof_indices,
    )
    q_init_isaac = control_to_external(
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
    # 位置驱动目标必须与Q_STAND同步，否则仿真开始第一帧会发生关节跳变。
    gym.set_actor_dof_position_targets(
        env,
        actor,
        q_init_isaac,
    )

    # 镜头中心放在两个机器人之间，初始画面可以同时观察六足和小蓝。
    gym.viewer_camera_look_at(
        viewer,
        None,
        gymapi.Vec3(0.75, -0.75, 0.5),
        gymapi.Vec3(0.0, 0.38, 0.10),
    )

    joystick = JoyStick()

    # 所有平移方向统一为20 cm/s。
    max_linear_speed = 0.20
    max_vertical_speed = 0.02   # m/s
    # 令标准足端的旋转切向速度同样等于20 cm/s。
    nominal_foot_radius = np.mean(
        np.linalg.norm(controller.foot_init_base[:, :2], axis=1)
    )
    max_yaw_rate = max_linear_speed / nominal_foot_radius

    command = np.zeros(
        4,
        dtype=np.float64,
    )
    control_enabled = False
    button_a_was_down = False
    button_b_was_down = False
    print(
        "A: enable/pause motion | B: reset to stand | "
        "RT: body up | LT: body down"
    )

    #状态读取主循环
    while not gym.query_viewer_has_closed(viewer):
        dof_states = gym.get_actor_dof_states(env, actor, gymapi.STATE_POS)
        # Isaac一维顺序 → 控制器(6,3)顺序
        q_control = external_to_control(
            dof_states["pos"],
            dof_indices,
        )

        # reference手柄输出：[B, right, forward, up, yaw]。
        # A/B在这里转换成单次按下事件，避免长按时每帧重复触发。
        _reset, axis_right, axis_forward, _axis_up_unused, axis_yaw = (
            joystick.get_commands()
        )

        # 北通BTP-KP20的axis 4/5是两个扳机，静止值均为-1，
        # 不能直接把axis 4取反，否则A使能后会持续收到最大上升指令。
        left_trigger = 0.5 * (
            joystick.joystick.get_axis(4) + 1.0
        )
        right_trigger = 0.5 * (
            joystick.joystick.get_axis(5) + 1.0
        )
        axis_up = right_trigger - left_trigger
        if abs(axis_up) < 0.05:
            axis_up = 0.0

        button_a_down = bool(joystick.joystick.get_button(0))
        button_b_down = bool(joystick.joystick.get_button(1))
        button_a_pressed = button_a_down and not button_a_was_down
        button_b_pressed = button_b_down and not button_b_was_down
        button_a_was_down = button_a_down
        button_b_was_down = button_b_down

        if button_a_pressed:
            control_enabled = not control_enabled
            state = "ENABLED" if control_enabled else "PAUSED"
            print(f"Motion control: {state}")
            if control_enabled:
                # A使能后先平滑回到末段竖直的Q_STAND，再接收摇杆。
                controller.reset_to_stand(q_control)

        if button_b_pressed:
            control_enabled = False
            controller.reset_to_stand(q_control)
            print("Controller returning to stand")

        if control_enabled and not controller.reset_active:
            # 圆形归一化保证前后、左右和任意斜向的最大合速度一致。
            translation_axes = np.array(
                [axis_right, axis_forward],
                dtype=np.float64,
            )
            translation_norm = np.linalg.norm(translation_axes)
            if translation_norm > 1.0:
                translation_axes /= translation_norm

            command[:] = [
                max_linear_speed * translation_axes[0],
                max_linear_speed * translation_axes[1],
                max_vertical_speed * axis_up,
                max_yaw_rate * axis_yaw,
            ]
        else:
            # A暂停时给步态规划器零指令，使当前摆动腿先落地再停止。
            command[:] = 0.0

        # 总控制器选择当前mode，再统一完成足端规划和DLS关节解算。
        q_des_control = controller.update(q_control, command)
        # 不再对每周期关节目标做人为速度裁剪，让位置驱动直接跟踪DLS结果。
        # 机械角限位必须保留，避免控制目标越过URDF允许范围。
        q_target_control = np.clip(
            q_des_control,
            lower_control,
            upper_control,
        )

        # 正常情况下cal_joint_poses已经检查过碰撞；只有机械限位
        # 实际改变了目标，才需要对改变后的姿态重新检查。
        if not np.array_equal(q_target_control, q_des_control):
            q_target_control = controller.collision_guard(
                q_target_control,
                q_control,
            )

        # 控制器顺序 → Isaac Gym顺序
        q_target_isaac = control_to_external(
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
