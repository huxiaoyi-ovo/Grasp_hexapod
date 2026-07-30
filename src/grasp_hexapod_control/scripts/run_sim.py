#!/usr/bin/env python3
"""Isaac Gym仿真运行入口。

功能：
    配置CUDA/PhysX，加载六足、小蓝和地面，读取手柄与仿真关节状态，
    调用GraspController，并把18个关节位置目标发送给Isaac Gym；可选地
    自动执行固定行走序列并记录舵机目标和仿真反馈。
输入：
    手柄归一化指令；Isaac Gym关节状态，内部转换为q_cur.shape=(6,3)，单位rad。
输出：
    Isaac顺序的18关节位置目标；viewer画面；可选的CSV时序轨迹。
结构：
    创建仿真与资产 -> 建立关节顺序映射 -> 主控制循环 -> 资源释放。
约定：
    base_link中+x向右、+y向前、+z向上；长度单位m，角度单位rad。
"""

import argparse
import csv
from pathlib import Path
import struct
import time

from isaacgym import gymapi
import numpy as np
import pygame

from control import GraspController
from utils import (
    CONTROL_DOF_NAMES,
    build_dof_indices,
    control_to_external,
    external_to_control,
)


DEFAULT_TRACE_PATH = Path("logs/servo_walk_trace.csv")
TRACE_ACTION_DURATION = 5.0


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


def parse_arguments():
    """读取仿真入口参数；不启用录制时保持原来的手柄控制。"""

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--record-servo-trace",
        nargs="?",
        const=DEFAULT_TRACE_PATH,
        type=Path,
        help="run the fixed motion sequence and write its CSV trace",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="run trace recording without the Isaac Gym viewer",
    )
    parser.add_argument(
        "--control-rate",
        type=float,
        choices=(30.0, 60.0),
        default=60.0,
        help="controller update rate",
    )
    parser.add_argument(
        "--physics-rate",
        type=float,
        default=240.0,
        help="fixed Isaac Gym physics rate",
    )
    return parser.parse_args()


def build_servo_trace_script(controller, linear_speed, yaw_rate):
    """生成初始化、前进、右移和旋转的确定性测试时间轴。"""

    dt = controller.dt
    half_cycle = (
        controller.approach_mode.phase_duration
        + controller.approach_mode.transfer_duration
    )
    full_cycle = 2.0 * half_cycle
    action_cycles = max(
        1,
        int(round(TRACE_ACTION_DURATION / full_cycle)),
    )
    action_duration = action_cycles * full_cycle
    print(
        f"Trace action: {action_cycles} full gait cycles, "
        f"{action_duration:.2f} s"
    )
    stages = (
        ("initialize", 1.0, [0.0, 0.0, 0.0, 0.0]),
        ("forward", action_duration, [0.0, linear_speed, 0.0, 0.0]),
        ("settle", 2.0 * half_cycle, [0.0, 0.0, 0.0, 0.0]),
        ("right", action_duration, [linear_speed, 0.0, 0.0, 0.0]),
        ("settle", 2.0 * half_cycle, [0.0, 0.0, 0.0, 0.0]),
        ("rotate", action_duration, [0.0, 0.0, 0.0, yaw_rate]),
        ("settle", 2.0 * half_cycle, [0.0, 0.0, 0.0, 0.0]),
    )

    script = []
    for name, duration, command in stages:
        frame_count = int(round(duration / dt))
        script.extend(
            (name, np.asarray(command, dtype=np.float64))
            for _ in range(frame_count)
        )
    return script


def write_servo_trace(path, rows):
    """保存控制器顺序的目标角和同帧Isaac Gym反馈角。"""

    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    header = [
        "frame",
        "time_s",
        "stage",
        "vx",
        "vy",
        "vz",
        "wz",
    ]
    header.extend(f"target_{name}" for name in CONTROL_DOF_NAMES)
    header.extend(f"actual_{name}" for name in CONTROL_DOF_NAMES)

    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(header)
        writer.writerows(rows)

    print(f"Servo trace saved: {path}")
    duration = 0.0
    if len(rows) >= 2:
        duration = rows[-1][1] + rows[1][1] - rows[0][1]
    print(f"Recorded frames: {len(rows)}, duration: {duration:.2f} s")


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
    args = parse_arguments()
    if args.headless and args.record_servo_trace is None:
        raise ValueError("--headless is only used with --record-servo-trace")
    rate_ratio = args.physics_rate / args.control_rate
    if abs(rate_ratio - round(rate_ratio)) > 1e-9:
        raise ValueError("physics rate must be an integer multiple of control rate")
    control_interval = int(round(rate_ratio))
    render_interval = max(1, int(round(args.physics_rate / 60.0)))

    # 仿真从源码树直接运行，不依赖ROS_PACKAGE_PATH或source devel/setup.bash。
    description_root = (
        Path(__file__).resolve().parents[2]
        / "grasp_hexapod_description"
    )
    gym = gymapi.acquire_gym()
    # create a simulator
    sim_params = gymapi.SimParams() # create a sim params object

    sim_params.dt = 1.0 / args.physics_rate
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
    graphics_device_id = -1 if args.headless else 0
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

    viewer = None
    if not args.headless:
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

    # 控制器使用命令频率的双精度dt；不要从Isaac Gym的float32 dt反读，
    # 否则30 Hz下50 ms换相时间会因1.4999999取整成错误的1帧。
    controller = GraspController(dt=1.0 / args.control_rate)
    print(
        f"Physics: {args.physics_rate:.0f} Hz, controller: "
        f"{args.control_rate:.0f} Hz, update every "
        f"{control_interval} physics frames"
    )
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
    if viewer is not None:
        gym.viewer_camera_look_at(
            viewer,
            None,
            gymapi.Vec3(0.75, -0.75, 0.5),
            gymapi.Vec3(0.0, 0.38, 0.10),
        )

    # 所有平移方向统一为20 cm/s。
    max_linear_speed = 0.20
    max_vertical_speed = 0.02   # m/s
    # 令标准足端的旋转切向速度同样等于20 cm/s。
    nominal_foot_radius = np.mean(
        np.linalg.norm(controller.foot_init_base[:, :2], axis=1)
    )
    max_yaw_rate = max_linear_speed / nominal_foot_radius

    trace_script = None
    trace_rows = []
    script_frame = 0
    if args.record_servo_trace is not None:
        trace_script = build_servo_trace_script(
            controller,
            max_linear_speed,
            max_yaw_rate,
        )
        print(
            f"Recording {args.control_rate:.0f} Hz servo trace: "
            "initialize -> forward -> right -> rotate"
        )
    else:
        joystick = JoyStick()

    command = np.zeros(
        4,
        dtype=np.float64,
    )
    control_enabled = False
    button_a_was_down = False
    button_b_was_down = False
    if trace_script is None:
        print(
            "A: enable/pause motion | B: reset to stand | "
            "RT: body up | LT: body down"
        )

    physics_frame = 0

    # PhysX固定高频运行，控制器只在自己的更新帧读取状态并生成新目标。
    while (
        viewer is None
        or not gym.query_viewer_has_closed(viewer)
    ):
        if trace_script is not None and script_frame >= len(trace_script):
            break

        control_tick = physics_frame % control_interval == 0
        if control_tick:
            dof_states = gym.get_actor_dof_states(
                env,
                actor,
                gymapi.STATE_POS,
            )
            # Isaac一维顺序 → 控制器(6,3)顺序
            q_control = external_to_control(
                dof_states["pos"],
                dof_indices,
            )

            if trace_script is not None:
                stage, scripted_command = trace_script[script_frame]
                command[:] = scripted_command
            else:
                # reference手柄输出：[B, right, forward, up, yaw]。
                # A/B转换成单次按下事件，避免长按时每帧重复触发。
                (
                    _reset,
                    axis_right,
                    axis_forward,
                    _axis_up_unused,
                    axis_yaw,
                ) = joystick.get_commands()

                # 北通BTP-KP20的axis 4/5是两个扳机，静止值均为-1。
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
                button_a_pressed = (
                    button_a_down and not button_a_was_down
                )
                button_b_pressed = (
                    button_b_down and not button_b_was_down
                )
                button_a_was_down = button_a_down
                button_b_was_down = button_b_down

                if button_a_pressed:
                    control_enabled = not control_enabled
                    state = "ENABLED" if control_enabled else "PAUSED"
                    print(f"Motion control: {state}")
                    if control_enabled:
                        controller.reset_to_stand(q_control)

                if button_b_pressed:
                    control_enabled = False
                    controller.reset_to_stand(q_control)
                    print("Controller returning to stand")

                if control_enabled and not controller.reset_active:
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
                    # 暂停时让当前摆动腿先落地再停止。
                    command[:] = 0.0

            # 当前控制帧读取反馈、规划足端并执行DLS。
            q_des_control = controller.update(q_control, command)
            q_target_control = np.clip(
                q_des_control,
                lower_control,
                upper_control,
            )

            # 只有机械限位实际改变目标时才重新检查改变后的姿态。
            if not np.array_equal(q_target_control, q_des_control):
                q_target_control = controller.collision_guard(
                    q_target_control,
                    q_control,
                )

            q_target_isaac = control_to_external(
                q_target_control,
                dof_indices,
            )
            gym.set_actor_dof_position_targets(
                env,
                actor,
                q_target_isaac,
            )

            if trace_script is not None:
                trace_rows.append(
                    [
                        script_frame,
                        script_frame * controller.dt,
                        stage,
                        *command,
                        *q_target_control.reshape(18),
                        *q_control.reshape(18),
                    ]
                )
                script_frame += 1

        gym.simulate(sim)
        gym.fetch_results(sim, True)
        physics_frame += 1

        if viewer is not None:
            if physics_frame % render_interval == 0:
                gym.step_graphics(sim)
                gym.draw_viewer(viewer, sim, True)
            gym.sync_frame_time(sim)

    if args.record_servo_trace is not None:
        write_servo_trace(args.record_servo_trace, trace_rows)
    if viewer is not None:
        gym.destroy_viewer(viewer)
    gym.destroy_sim(sim)


if __name__ == "__main__":
    main()
