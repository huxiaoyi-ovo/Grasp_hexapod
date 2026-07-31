#!/usr/bin/env python3
"""Isaac Gym仿真运行入口。

功能：
    配置CUDA/PhysX，加载六足、小蓝和地面，读取手柄与仿真关节状态，
    调用GraspController，并把18个关节位置目标发送给Isaac Gym；可选地
    自动执行固定行走序列并记录舵机目标和仿真反馈。加--ros时从ROS读取
    Joy/导航输入，但控制器仍在Isaac控制帧内同步运行，避免跨节点时序抖动。
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
import sys
import time
from typing import Any

from isaacgym import gymapi as _gymapi
import numpy as np

# catkin转发脚本不与控制器模块同目录，ROS启动时需定位源码scripts。
scripts_dir = Path(__file__).resolve().parent
if not (scripts_dir / "control.py").exists():
    import rospkg

    scripts_dir = (
        Path(rospkg.RosPack().get_path("grasp_hexapod_control"))
        / "scripts"
    )
sys.path.insert(0, str(scripts_dir))

from control import GraspController
from kinematics import LEG_NAMES
from utils import (
    CONTROL_DOF_NAMES,
    build_dof_indices,
    control_to_external,
    external_to_control,
)

# Isaac Gym的C扩展没有完整类型声明，编辑器无法静态识别其动态属性。
gymapi: Any = _gymapi


DEFAULT_TRACE_PATH = Path("logs/servo_walk_trace.csv")
TRACE_ACTION_DURATION = 5.0


class JoyStick:
    """仿真手柄输入；输出归一化的右移、前进、升降和偏航指令。"""

    DEADZONE = 0.20

    def __init__(self):
        # ROS仿真不需要pygame；只在原直接控制链路中加载它。
        import pygame

        self.pygame = pygame
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
        magnitude = abs(value)
        if magnitude <= JoyStick.DEADZONE:
            return 0.0

        # 死区边缘映射到0，最大行程仍映射到1；
        # 因而速度随有效摇杆行程连续、线性变化。
        return np.sign(value) * (
            (magnitude - JoyStick.DEADZONE)
            / (1.0 - JoyStick.DEADZONE)
        )

    def get_commands(self):
        self.pygame.event.pump()
        axis_right = self._deadzone(self.joystick.get_axis(0))
        axis_forward = self._deadzone(-self.joystick.get_axis(1))
        axis_yaw = self._deadzone(-self.joystick.get_axis(3))
        return axis_right, axis_forward, axis_yaw


class RosSimTelemetry:
    """发布与实机相同的关节话题，但不让话题参与仿真控制闭环。"""

    def __init__(self):
        import rospy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray, Header

        self.rospy = rospy
        self.joint_state_type = JointState
        self.header_type = Header
        self.target_type = Float64MultiArray
        self.position_publishers = {
            leg: rospy.Publisher(
                f"/{leg}_pos",
                JointState,
                queue_size=1,
            )
            for leg in LEG_NAMES
        }
        self.target_publishers = {
            leg: rospy.Publisher(
                f"/{leg}_des",
                Float64MultiArray,
                queue_size=1,
            )
            for leg in LEG_NAMES
        }

    def publish_feedback(self, joint_position):
        joint_position = np.asarray(
            joint_position,
            dtype=np.float64,
        ).reshape(6, 3)
        for leg_index, leg_name in enumerate(LEG_NAMES):
            self.position_publishers[leg_name].publish(
                self.joint_state_type(
                    header=self.header_type(
                        stamp=self.rospy.Time.now()
                    ),
                    name=[
                        f"{leg_name}_thigh_joint",
                        f"{leg_name}_knee_joint",
                        f"{leg_name}_ankle_joint",
                    ],
                    position=joint_position[leg_index].tolist(),
                )
            )

    def publish_target(self, joint_target):
        joint_target = np.asarray(
            joint_target,
            dtype=np.float64,
        ).reshape(6, 3)
        for leg_index, leg_name in enumerate(LEG_NAMES):
            q_leg = joint_target[leg_index]
            self.target_publishers[leg_name].publish(
                self.target_type(
                    data=[
                        1.0,
                        float(q_leg[0]),
                        float(q_leg[1]),
                        float(q_leg[2]),
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                    ]
                )
            )


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
        help="run trace recording or ROS simulation without the viewer",
    )
    parser.add_argument(
        "--ros",
        action="store_true",
        help="use ROS Joy/navigation input with synchronous simulation control",
    )
    parser.add_argument(
        "--control-rate",
        type=float,
        choices=(30.0, 60.0),
        default=60.0,
        help="controller update rate",
    )
    parser.add_argument(
        "--actuator-rate",
        type=float,
        choices=(30.0, 60.0),
        default=60.0,
        help="rate at which joint targets are applied to Isaac Gym",
    )
    parser.add_argument(
        "--physics-rate",
        type=float,
        default=240.0,
        help="fixed Isaac Gym physics rate",
    )
    parser.add_argument(
        "--max-linear-speed",
        type=float,
        default=0.20,
        help="direct-control and trace planar speed in m/s",
    )
    parser.add_argument(
        "--max-vertical-speed",
        type=float,
        default=0.02,
        help="direct-control body-height speed in m/s",
    )
    # roslaunch会附加__name:=和__log:=；普通命令行参数仍严格检查。
    argv = [
        argument
        for argument in sys.argv[1:]
        if not argument.startswith("__")
    ]
    return parser.parse_args(argv)


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
    if args.ros and args.record_servo_trace is not None:
        raise ValueError("--ros and --record-servo-trace cannot be combined")
    if args.headless and not args.ros and args.record_servo_trace is None:
        raise ValueError(
            "--headless is only used with --ros or --record-servo-trace"
        )
    if args.ros:
        import rospy

        rospy.init_node("grasp_hexapod_sim")

    rate_ratio = args.physics_rate / args.control_rate
    if abs(rate_ratio - round(rate_ratio)) > 1e-9:
        raise ValueError("physics rate must be an integer multiple of control rate")
    control_interval = int(round(rate_ratio))
    actuator_ratio = args.control_rate / args.actuator_rate
    if abs(actuator_ratio - round(actuator_ratio)) > 1e-9:
        raise ValueError(
            "control rate must be an integer multiple of actuator rate"
        )
    actuator_interval = int(round(actuator_ratio))
    render_interval = max(1, int(round(args.physics_rate / 60.0)))

    # 仿真从源码树直接运行，不依赖ROS_PACKAGE_PATH或source devel/setup.bash。
    description_root = (
        Path(__file__).resolve().parents[2]
        / "grasp_hexapod_description"
    )
    if not description_root.is_dir():
        import rospkg

        description_root = Path(
            rospkg.RosPack().get_path("grasp_hexapod_description")
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

    # ROS仿真也在Isaac控制帧内同步运行控制器；ROS只缓存Joy/导航输入。
    ros_controller = None
    ros_telemetry = None
    if args.ros:
        from run_real import RosControlNode

        ros_controller = RosControlNode(
            local_execution=True,
            controller_rate_hz=args.control_rate,
        )
        ros_telemetry = RosSimTelemetry()
        controller = ros_controller.controller
    else:
        controller = GraspController(dt=1.0 / args.control_rate)
    print(
        f"Physics: {args.physics_rate:.0f} Hz, controller: "
        f"{args.control_rate:.0f} Hz, actuator: "
        f"{args.actuator_rate:.0f} Hz"
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

    # 直接仿真、轨迹录制和ROS实机默认使用同一速度上限。
    max_linear_speed = args.max_linear_speed
    max_vertical_speed = args.max_vertical_speed
    # 令标准足端的旋转切向速度等于平移速度上限。
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
        if ros_controller is None:
            joystick = JoyStick()

    command = np.zeros(
        4,
        dtype=np.float64,
    )
    motion_state = "HOLD" if trace_script is not None else "WAIT_B"
    button_a_was_down = False
    button_b_was_down = False
    button_x_was_down = False
    button_y_was_down = False
    q_des_control = controller.q_init.copy()
    if trace_script is None and ros_controller is None:
        print(
            "A: enable/pause motion | B: reset to stand | "
            "X: climb(reserved) | Y: dock(reserved)"
        )

    physics_frame = 0
    control_frame = 0

    # PhysX固定高频运行，控制器只在自己的更新帧读取状态并生成新目标。
    while (
        (
            viewer is None
            or not gym.query_viewer_has_closed(viewer)
        )
        and (
            ros_controller is None
            or not rospy.is_shutdown()
        )
    ):
        if trace_script is not None and script_frame >= len(trace_script):
            break

        control_tick = physics_frame % control_interval == 0
        if control_tick:
            actuator_tick = control_frame % actuator_interval == 0
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

            if ros_controller is not None:
                if actuator_tick:
                    ros_telemetry.publish_feedback(q_control)
                synchronous_target = ros_controller.update_from_feedback(
                    q_control
                )
                if synchronous_target is not None:
                    q_des_control = synchronous_target
                    ros_telemetry.publish_target(q_des_control)
            elif trace_script is not None:
                stage, scripted_command = trace_script[script_frame]
                command[:] = scripted_command
            else:
                (
                    axis_right,
                    axis_forward,
                    axis_yaw,
                ) = joystick.get_commands()

                # 北通BTP-KP20的axis 4/5是两个扳机，静止值均为-1。
                left_trigger = 0.5 * (
                    joystick.joystick.get_axis(4) + 1.0
                )
                right_trigger = 0.5 * (
                    joystick.joystick.get_axis(5) + 1.0
                )
                axis_body = right_trigger - left_trigger
                if abs(axis_body) < 0.05:
                    axis_body = 0.0

                button_a_down = bool(joystick.joystick.get_button(0))
                button_b_down = bool(joystick.joystick.get_button(1))
                button_x_down = bool(joystick.joystick.get_button(2))
                button_y_down = bool(joystick.joystick.get_button(3))
                button_a_pressed = (
                    button_a_down and not button_a_was_down
                )
                button_b_pressed = (
                    button_b_down and not button_b_was_down
                )
                button_x_pressed = (
                    button_x_down and not button_x_was_down
                )
                button_y_pressed = (
                    button_y_down and not button_y_was_down
                )
                button_a_was_down = button_a_down
                button_b_was_down = button_b_down
                button_x_was_down = button_x_down
                button_y_was_down = button_y_down

                if button_b_pressed:
                    motion_state = "RESETTING"
                    controller.reset_to_stand(q_control)
                    print("Controller returning to stand")
                elif button_x_pressed or button_y_pressed:
                    if motion_state == "RUNNING":
                        motion_state = "HOLD"
                    mode = "CLIMB" if button_x_pressed else "DOCK"
                    print(f"{mode} is reserved but not implemented")
                elif button_a_pressed:
                    if motion_state == "HOLD":
                        motion_state = "RUNNING"
                        print("Motion control: ENABLED")
                    elif motion_state == "RUNNING":
                        motion_state = "HOLD"
                        print("Motion control: PAUSED")
                    else:
                        print("A ignored: press B and wait for stand first")

                if motion_state == "RUNNING":
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
                        max_vertical_speed * axis_body,
                        max_yaw_rate * axis_yaw,
                    ]
                else:
                    # 暂停时让当前摆动腿先落地再停止。
                    command[:] = 0.0

            if ros_controller is None:
                # 直接模式仍由本文件读取手柄、规划足端并执行DLS。
                q_des_control = controller.update(q_control, command)
                if (
                    trace_script is None
                    and motion_state == "RESETTING"
                    and not controller.reset_active
                ):
                    motion_state = "HOLD"
                    print("Stand initialization complete; press A to move")

            q_target_control = np.clip(
                q_des_control,
                lower_control,
                upper_control,
            )

            # 只有机械限位实际改变目标时才重新检查改变后的姿态。
            if (
                not np.array_equal(q_target_control, q_des_control)
            ):
                q_target_control = controller.collision_guard(
                    q_target_control,
                    q_control,
                )

            q_target_isaac = control_to_external(
                q_target_control,
                dof_indices,
            )
            if actuator_tick:
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
            control_frame += 1

        gym.simulate(sim)
        gym.fetch_results(sim, True)
        physics_frame += 1

        if viewer is not None:
            if physics_frame % render_interval == 0:
                gym.step_graphics(sim)
                gym.draw_viewer(viewer, sim, True)
            gym.sync_frame_time(sim)
        elif ros_controller is not None:
            # ROS输入仿真按真实时间运行，避免无Viewer时无限加速。
            gym.sync_frame_time(sim)

    if args.record_servo_trace is not None:
        write_servo_trace(args.record_servo_trace, trace_rows)
    if viewer is not None:
        gym.destroy_viewer(viewer)
    gym.destroy_sim(sim)


if __name__ == "__main__":
    main()
