#!/usr/bin/env python3
"""六足、小蓝、底部相机与正式DockMode的单文件Isaac Gym联合仿真。

只复用工程公共的dock_mode、control、kinematics和utils；不再依赖任何
run_sim_dock_mode*或isaacgym_apriltag_camera仿真脚本。启动状态由compact
攀爬终态重建，确保对接首帧连续承接攀爬末帧。
"""

import csv
from dataclasses import dataclass
import json
import os
from pathlib import Path
import struct
import sys
import time

SCRIPT_DIR = Path(__file__).resolve().parent
ISAAC_GYM_ROOT = Path(os.environ.get(
    "ISAAC_GYM_ROOT", str(Path.home() / "robot_ws/isaacgym/python")
))
XIAOLAN_ROOT = Path(os.environ.get(
    "XIAOLAN_ASSET_ROOT",
    str(SCRIPT_DIR.parents[1] / "grasp_hexapod_description"),
))

if ISAAC_GYM_ROOT.is_dir():
    sys.path.insert(0, str(ISAAC_GYM_ROOT))
sys.path.insert(0, str(SCRIPT_DIR))
# 绝对路径启动Python时shell未必激活对应环境；确保gymtorch能找到ninja。
python_bin = str(Path(sys.executable).resolve().parent)
os.environ["PATH"] = python_bin + os.pathsep + os.environ.get("PATH", "")
os.environ.setdefault("__NV_PRIME_RENDER_OFFLOAD", "1")
os.environ.setdefault("__GLX_VENDOR_LIBRARY_NAME", "nvidia")

from isaacgym import gymapi, gymtorch
import cv2
import numpy as np
import rospy
import tf2_ros
import torch
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState

import dock_mode as body
from approach_mode import ApproachMode
from control import GraspController
from dock_mode import PerceptionResult, TAG_IDS
from kinematics import FOOT_RADIUS, Q_STAND
from utils import (
    build_dof_indices,
    control_to_external,
    external_to_control,
    package_config_path,
)

# 单文件仿真配置。路径可用上面的环境变量覆盖。
dt = 1.0 / 60.0
control_interval = 2
camera_width, camera_height = 1920, 1080
# 图像发布由30 Hz降为20 Hz，AprilTag仍按10 Hz检测，降低GPU/ROS负载。
camera_fov, camera_interval = 120.0, 3
dock_time_scale = 3.0

# 当前联合仿真实际使用的底部相机与AprilTag几何。
ISAAC_FROM_OPTICAL_ROTATION = np.array(
    ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0))
)
LOCK_FROM_CAMERA_ROTATION = np.diag((1.0, -1.0, -1.0))
CAMERA_POSITION_IN_LOCK = np.array((0.0, -0.065, -0.0325))
PIN_WORLD = np.array((0.0, -0.028, 0.228))
TAG_SIZE_M = 0.040
PIN_FROM_TAG = {
    0: np.array((0.0, 0.100, -0.037)),
    1: np.array((0.100, 0.0, -0.037)),
    2: np.array((0.0, -0.100, -0.037)),
    3: np.array((-0.100, 0.0, -0.037)),
}
PIN_FROM_OPENCV_TAG_ROTATION = np.diag((-1.0, -1.0, 1.0))
# 攀爬终态相机距标签面约38 mm，16:9画面的垂直视野无法容纳偏离光轴
# 35 mm的完整40 mm标签。对接请求后先把相机升到旧版已验证的260 mm，
# 对应标签面上方约69 mm；只有真实图像完整解码后才允许进入DockMode。
TAG_REACQUIRE_CAMERA_HEIGHT_M = 0.260
TAG_REACQUIRE_FRESH_S = 1.0
TAG_REACQUIRE_MIN_RAISE_M = 0.002
TAG_REACQUIRE_MAX_IK_ERROR_M = 0.001


def create_apriltag_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_APRILTAG_36h11
    )
    parameters = cv2.aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    parameters.cornerRefinementWinSize = 5
    return dictionary, parameters


def calculate_intrinsics(width, height, horizontal_fov):
    focal = 0.5 * width / np.tan(0.5 * np.deg2rad(horizontal_fov))
    cx, cy = 0.5 * (width - 1), 0.5 * (height - 1)
    return (
        np.array(((focal, 0.0, cx), (0.0, focal, cy), (0.0, 0.0, 1.0))),
        np.zeros(5),
    )


def quaternion_from_rotation(rotation):
    """将3×3旋转矩阵转换为Isaac Gym使用的[x,y,z,w]。"""
    matrix = np.asarray(rotation)
    values = np.array((
        1.0 + np.trace(matrix),
        1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2],
        1.0 - matrix[0, 0] + matrix[1, 1] - matrix[2, 2],
        1.0 - matrix[0, 0] - matrix[1, 1] + matrix[2, 2],
    ))
    index = int(np.argmax(values))
    quaternion = np.empty(4)
    if index == 0:
        w = 0.5 * np.sqrt(values[0])
        quaternion[:] = (
            (matrix[2, 1] - matrix[1, 2]) / (4.0 * w),
            (matrix[0, 2] - matrix[2, 0]) / (4.0 * w),
            (matrix[1, 0] - matrix[0, 1]) / (4.0 * w), w,
        )
    else:
        axis = index - 1
        component = 0.5 * np.sqrt(values[index])
        differences = np.array((
            matrix[2, 1] - matrix[1, 2],
            matrix[0, 2] - matrix[2, 0],
            matrix[1, 0] - matrix[0, 1],
        ))
        quaternion[axis] = component
        other = [value for value in range(3) if value != axis]
        quaternion[other[0]] = (
            matrix[axis, other[0]] + matrix[other[0], axis]
        ) / (4.0 * component)
        quaternion[other[1]] = (
            matrix[axis, other[1]] + matrix[other[1], axis]
        ) / (4.0 * component)
        quaternion[3] = differences[axis] / (4.0 * component)
    return quaternion / np.linalg.norm(quaternion)


@dataclass(frozen=True)
class ClimbTerminalState:
    """从compact完整攀爬末端重建的仿真对接交接状态。"""

    stage_name: str
    base_position: np.ndarray
    base_rotation: np.ndarray
    joints: np.ndarray
    feet_base: np.ndarray
    feet_xiaolan: np.ndarray
    camera_position: np.ndarray
    max_fk_error_m: float


def _transform_points(transform, points):
    """用4x4齐次矩阵变换一组行向量点。"""
    matrix = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    values = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    return values @ matrix[:3, :3].T + matrix[:3, 3]


def load_climb_terminal_state(controller):
    """读取攀爬配置，并通过正式ClimbMode回放得到终态关节分支。

    对接仿真中的小蓝位于世界原点，而compact使用自己的世界平移；这里
    统一转换到小蓝坐标系，使攀爬末帧和对接首帧使用同一相对几何。
    """
    compact_path = package_config_path("climb_compact.json")
    with compact_path.open(encoding="utf-8") as compact_file:
        compact = json.load(compact_file)
    controller.climb_mode._validate_config(compact)
    stages = compact["stages"]
    final_stage = stages[-1]

    p0_joints = np.asarray(compact["p0"]["q_rad"], dtype=np.float64)
    terminal_joints = controller.replay_climb_prefix(
        p0_joints,
        compact,
        len(stages) - 1,
        max_ticks=200000,
    )
    terminal_joints = np.asarray(
        terminal_joints, dtype=np.float64
    ).reshape(6, 3)

    world_from_base = controller.climb_mode._world_from_base(
        np.asarray(final_stage["pose_end"], dtype=np.float64)
    )
    world_from_xiaolan = np.eye(4, dtype=np.float64)
    world_from_xiaolan[:3, 3] = np.asarray(
        compact["xiaolan_translation"], dtype=np.float64
    )
    xiaolan_from_world = np.linalg.inv(world_from_xiaolan)
    xiaolan_from_base = xiaolan_from_world @ world_from_base

    terminal_feet_world = np.asarray(
        final_stage["anchor_knots"][-1], dtype=np.float64
    ).reshape(6, 3)
    terminal_feet_xiaolan = _transform_points(
        xiaolan_from_world, terminal_feet_world
    )
    target_feet_base = _transform_points(
        np.linalg.inv(xiaolan_from_base), terminal_feet_xiaolan
    )
    actual_feet_base = controller.kinematic.forward_base(terminal_joints)
    max_fk_error = float(np.max(np.linalg.norm(
        actual_feet_base - target_feet_base, axis=1
    )))
    max_allowed_error = float(
        compact["settle_gate"]["max_foot_target_error_m"]
    )
    if max_fk_error > max_allowed_error:
        raise RuntimeError(
            "攀爬终态回放与最终足端不连续：{:.3f} mm > {:.3f} mm".format(
                max_fk_error * 1000.0,
                max_allowed_error * 1000.0,
            )
        )

    base_position = xiaolan_from_base[:3, 3].copy()
    base_rotation = xiaolan_from_base[:3, :3].copy()
    camera_position = (
        base_position + base_rotation @ CAMERA_POSITION_IN_LOCK
    )
    values = (
        base_position,
        base_rotation,
        terminal_joints,
        actual_feet_base,
        terminal_feet_xiaolan,
        camera_position,
    )
    if not all(np.isfinite(value).all() for value in values):
        raise RuntimeError("攀爬终态包含非有限数值")
    return ClimbTerminalState(
        stage_name=final_stage["name"],
        base_position=base_position,
        base_rotation=base_rotation,
        joints=terminal_joints.copy(),
        feet_base=actual_feet_base.copy(),
        feet_xiaolan=terminal_feet_xiaolan.copy(),
        camera_position=camera_position,
        max_fk_error_m=max_fk_error,
    )


def initialize_from_climb_terminal(controller, terminal, surface):
    """把控制器基准同步为攀爬终态，避免对接首帧关节或足端跳变。"""
    surface_z = surface.heights(terminal.feet_xiaolan[:, :2])
    if not np.isfinite(surface_z).all():
        raise RuntimeError("攀爬终态有足端落在对接STL背部范围外")
    clearance = terminal.feet_xiaolan[:, 2] - surface_z
    clearance_error = np.abs(clearance - FOOT_RADIUS)
    if float(np.max(clearance_error)) > 0.002:
        raise RuntimeError(
            "攀爬终态与对接STL表面不连续：最大间隙误差{:.3f} mm".format(
                float(np.max(clearance_error)) * 1000.0
            )
        )

    joints = terminal.joints.copy()
    feet = terminal.feet_base.copy()
    controller.mission.cancel("simulation climb-to-dock handoff")
    controller.set_mode(controller.APPROACH)
    controller.reset_active = False
    controller.q_init = joints.copy()
    controller.q_des = joints.copy()
    controller.reset_start_q = joints.copy()
    controller.foot_init_base = feet.copy()
    controller.foot_desired_base = feet.copy()
    controller.foot_desired_base_prev = feet.copy()
    controller.foot_init_hip = controller.kinematic.base_to_hip(feet)
    controller.foot_desired_hip = controller.foot_init_hip.copy()
    controller.foot_current_hip = controller.kinematic.forward(joints)
    controller.base_height_at_stand = FOOT_RADIUS - np.mean(feet[:, 2])
    controller.actual_joints = joints.copy()
    controller.actual_feet_base = feet.copy()
    configure_workspace = getattr(
        controller, "configure_terrain_workspace", None
    )
    if configure_workspace is not None:
        configure_workspace(feet)
    controller.approach_mode.finish_reset()
    return joints, clearance_error


def start_tag_reacquisition_raise(
    controller,
    q_current,
    base_position,
    base_rotation,
):
    """锁定实际足端，并用五次关节曲线把相机平滑升到重捕获高度。"""
    q_current = np.asarray(q_current, dtype=np.float64).reshape(6, 3)
    base_position = np.asarray(
        base_position, dtype=np.float64
    ).reshape(3)
    base_rotation = np.asarray(
        base_rotation, dtype=np.float64
    ).reshape(3, 3)
    camera_position = (
        base_position + base_rotation @ CAMERA_POSITION_IN_LOCK
    )
    raise_distance = max(
        0.0,
        TAG_REACQUIRE_CAMERA_HEIGHT_M - float(camera_position[2]),
    )
    if raise_distance < TAG_REACQUIRE_MIN_RAISE_M:
        return 0.0

    actual_feet_base = controller.kinematic.forward_base(q_current)
    feet_world = (
        actual_feet_base @ base_rotation.T + base_position
    )
    target_position = base_position.copy()
    target_position[2] += raise_distance
    target_feet_base = (
        feet_world - target_position
    ) @ base_rotation
    target_joints, max_error = body.solve_joints(
        controller.kinematic,
        target_feet_base,
        q_current,
    )
    if max_error > TAG_REACQUIRE_MAX_IK_ERROR_M:
        raise RuntimeError(
            "标签重捕获升高IK误差过大：{:.3f} mm".format(
                max_error * 1000.0
            )
        )
    joint_margin = np.minimum(
        target_joints - body.joint_lower,
        body.joint_upper - target_joints,
    )
    if float(np.min(joint_margin)) <= 0.0:
        raise RuntimeError("标签重捕获升高目标超出关节限位")
    if not controller._foot_collision_free(target_feet_base).all():
        raise RuntimeError("标签重捕获升高目标存在足端碰撞")
    for phase in np.linspace(0.0, 1.0, 21):
        sample = (1.0 - phase) * q_current + phase * target_joints
        if not controller._link_collision_free(sample).all():
            raise RuntimeError("标签重捕获升高路径存在连杆碰撞")

    controller.q_init = target_joints.copy()
    controller.q_des = q_current.copy()
    controller.reset_start_q = q_current.copy()
    controller.reset_time = 0.0
    controller.foot_init_base = target_feet_base.copy()
    controller.foot_desired_base = actual_feet_base.copy()
    controller.foot_desired_base_prev = actual_feet_base.copy()
    controller.foot_init_hip = controller.kinematic.base_to_hip(
        target_feet_base
    )
    controller.foot_desired_hip = controller.kinematic.base_to_hip(
        actual_feet_base
    )
    controller.foot_current_hip = controller.kinematic.forward(q_current)
    controller.base_height_at_stand = (
        FOOT_RADIUS - np.mean(target_feet_base[:, 2])
    )
    configure_workspace = getattr(
        controller, "configure_terrain_workspace", None
    )
    if configure_workspace is not None:
        configure_workspace(target_feet_base)
    if hasattr(controller, "_dock_nominal_body_clearance"):
        controller._dock_nominal_body_clearance = float(
            controller.base_height_at_stand
        )
    controller._dock_entry_hold_active = False
    controller._reset_entry_settle_monitor()
    controller.reset_active = True
    return raise_distance


class RosCameraPublisher:
    """Publish Isaac Gym camera frames for apriltag_ros and rqt_image_view."""

    def __init__(
        self,
        width,
        height,
        horizontal_fov,
        image_topic,
        camera_info_topic,
        frame_id,
    ):
        try:
            import rospy
            from cv_bridge import CvBridge
            from sensor_msgs.msg import CameraInfo, Image
        except ImportError as error:
            raise RuntimeError(
                "ROS Python packages are unavailable. "
                "Run: source /opt/ros/noetic/setup.bash"
            ) from error

        self.rospy = rospy
        self.bridge = CvBridge()
        self.frame_id = frame_id
        if not rospy.core.is_initialized():
            rospy.init_node(
                "isaacgym_apriltag_camera",
                anonymous=False,
                disable_signals=False,
            )
        self.image_publisher = rospy.Publisher(
            image_topic,
            Image,
            queue_size=1,
        )
        self.camera_info_publisher = rospy.Publisher(
            camera_info_topic,
            CameraInfo,
            queue_size=1,
        )

        camera_matrix, distortion = calculate_intrinsics(
            width,
            height,
            horizontal_fov,
        )
        focal_length = camera_matrix[0, 0]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]

        self.camera_info = CameraInfo()
        self.camera_info.width = width
        self.camera_info.height = height
        self.camera_info.distortion_model = "plumb_bob"
        self.camera_info.D = distortion.tolist()
        self.camera_info.K = [
            focal_length,
            0.0,
            cx,
            0.0,
            focal_length,
            cy,
            0.0,
            0.0,
            1.0,
        ]
        self.camera_info.R = [
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
        self.camera_info.P = [
            focal_length,
            0.0,
            cx,
            0.0,
            0.0,
            focal_length,
            cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        print("ROS image topic:", image_topic)
        print("ROS camera info topic:", camera_info_topic)

    def publish(self, bgr):
        stamp = self.rospy.Time.now()
        image_message = self.bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
        image_message.header.stamp = stamp
        image_message.header.frame_id = self.frame_id
        self.camera_info.header.stamp = stamp
        self.camera_info.header.frame_id = self.frame_id
        self.image_publisher.publish(image_message)
        self.camera_info_publisher.publish(self.camera_info)

    def is_shutdown(self):
        return self.rospy.is_shutdown()




class JoyStick:
    """直接读取Linux手柄/dev/input/js0，避免依赖pygame。"""

    # Linux joystick标准轴：Z=左扳机，Rz=右扳机。Ry（轴4）是右摇杆
    # 纵轴，不能再当左扳机，否则只推左摇杆时也可能产生机身升降命令。
    LEFT_TRIGGER_AXIS = 2
    RIGHT_TRIGGER_AXIS = 5

    def __init__(self, device="/dev/input/js0"):
        self.device, self.file, self.next_retry = device, None, 0.0
        self.axes, self.buttons = np.zeros(8), np.zeros(16, dtype=bool)
        self.axis_neutral = np.zeros(8)
        self.axis_initialized = np.zeros(8, dtype=bool)
        self._connect()

    def _connect(self):
        try:
            self.file = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
            self.axes[:] = 0.0
            self.axis_neutral[:] = 0.0
            self.axis_initialized[:] = False
            print("手柄已连接：", self.device)
        except OSError:
            self.file, self.next_retry = None, time.monotonic() + 1.0

    def pump(self):
        if self.file is None:
            if time.monotonic() >= self.next_retry:
                self._connect()
            return
        while True:
            try:
                _, value, event_type, number = struct.unpack(
                    "IhBB", os.read(self.file, 8)
                )
            except BlockingIOError:
                return
            except OSError:
                os.close(self.file)
                self.file, self.next_retry = None, time.monotonic() + 1.0
                self.axes[:], self.buttons[:] = 0.0, False
                print("手柄已断开，等待重新连接")
                return
            is_initial = bool(event_type & 0x80)
            event_type &= 0x7F
            if event_type == 1 and number < len(self.buttons):
                self.buttons[number] = bool(value)
            elif event_type == 2 and number < len(self.axes):
                axis = np.clip(value / 32767.0, -1.0, 1.0)
                self.axes[number] = axis
                if is_initial or not self.axis_initialized[number]:
                    self.axis_neutral[number] = axis
                    self.axis_initialized[number] = True

    def get_axis(self, index):
        return float(self.axes[index])

    def get_button(self, index):
        return bool(self.buttons[index])

    def _trigger_value(self, index):
        """把静止在正/负端点的扳机转换为0~1；中位摇杆不参与升降。"""
        if not self.axis_initialized[index]:
            return 0.0
        neutral = self.axis_neutral[index]
        value = self.axes[index]
        if neutral <= -0.5:
            pressed = (value - neutral) / max(1.0 - neutral, 1e-6)
        elif neutral >= 0.5:
            pressed = (neutral - value) / max(1.0 + neutral, 1e-6)
        else:
            return 0.0
        return float(np.clip(pressed, 0.0, 1.0))

    def get_height_command(self):
        """返回右扳机上升、左扳机下降的归一化指令。"""
        up = (
            self._trigger_value(self.RIGHT_TRIGGER_AXIS)
            - self._trigger_value(self.LEFT_TRIGGER_AXIS)
        )
        return 0.0 if abs(up) < 0.08 else up

    def get_commands(self):
        self.pump()
        deadzone = lambda value: 0.0 if abs(value) < 0.1 else value
        return (
            self.get_button(1), deadzone(self.get_axis(0)),
            deadzone(-self.get_axis(1)), deadzone(-self.get_axis(4)),
            deadzone(-self.get_axis(3)),
        )


def load_asset(gym, sim, root, filename, fixed):
    options = gymapi.AssetOptions()
    options.fix_base_link = fixed
    options.disable_gravity = fixed
    options.collapse_fixed_joints = False
    options.use_mesh_materials = True
    if not fixed:
        options.linear_damping = 0.5
        options.angular_damping = 1.0
    asset = gym.load_asset(sim, str(root), filename, options)
    if asset is None:
        raise RuntimeError("无法加载模型：{}".format(Path(root) / filename))
    return asset


class BackSurface:
    """读取小蓝STL，并查询任意(x, y)正下方的最高真实表面。"""

    def __init__(self, mesh_path):
        data = Path(mesh_path).read_bytes()
        triangle_count = struct.unpack_from("<I", data, 80)[0]
        record = np.dtype([
            ("normal", "<f4", (3,)),
            ("vertices", "<f4", (3, 3)),
            ("attribute", "<u2"),
        ])
        self.triangles = np.frombuffer(
            data, dtype=record, count=triangle_count, offset=84
        )["vertices"].astype(np.float64)
        self.xy_min = self.triangles[:, :, :2].min(axis=1)
        self.xy_max = self.triangles[:, :, :2].max(axis=1)

    def heights(self, points_xy):
        """沿-z轴投射，返回每个世界坐标落脚点的STL表面高度。"""
        heights = np.full(len(points_xy), np.nan)
        for index, (x, y) in enumerate(points_xy):
            inside_box = (
                (self.xy_min[:, 0] <= x) & (x <= self.xy_max[:, 0])
                & (self.xy_min[:, 1] <= y) & (y <= self.xy_max[:, 1])
            )
            triangle = self.triangles[inside_box]
            if not len(triangle):
                continue
            a, b, c = triangle[:, 0], triangle[:, 1], triangle[:, 2]
            denominator = (
                (b[:, 1] - c[:, 1]) * (a[:, 0] - c[:, 0])
                + (c[:, 0] - b[:, 0]) * (a[:, 1] - c[:, 1])
            )
            valid = np.abs(denominator) > 1e-10
            a, b, c, denominator = (
                a[valid], b[valid], c[valid], denominator[valid]
            )
            u = (
                (b[:, 1] - c[:, 1]) * (x - c[:, 0])
                + (c[:, 0] - b[:, 0]) * (y - c[:, 1])
            ) / denominator
            v = (
                (c[:, 1] - a[:, 1]) * (x - c[:, 0])
                + (a[:, 0] - c[:, 0]) * (y - c[:, 1])
            ) / denominator
            valid = (u >= -1e-7) & (v >= -1e-7) & (u + v <= 1.0 + 1e-7)
            z = u * a[:, 2] + v * b[:, 2] + (1.0 - u - v) * c[:, 2]
            if valid.any():
                heights[index] = z[valid].max()
        return heights


def add_xiaolan_collision(gym, sim, surface):
    """使用原始STL三角面碰撞，保留整个崎岖背部的真实形状。"""
    vertices = np.ascontiguousarray(
        surface.triangles.reshape(-1, 3), dtype=np.float32
    )
    triangle_count = len(surface.triangles)
    indices = np.arange(len(vertices), dtype=np.uint32).reshape(-1, 3)
    params = gymapi.TriangleMeshParams()
    params.nb_vertices, params.nb_triangles = len(vertices), triangle_count
    params.static_friction, params.dynamic_friction = 5.0, 4.0
    gym.add_triangle_mesh(sim, vertices.ravel(), indices.ravel(), params)
    print("小蓝精确背部碰撞面：{}个三角形".format(triangle_count))


def _base_set_contact_friction(gym, env, actor, friction):
    """提高脚垫与小蓝背部摩擦，静止时抑制滑移。"""
    properties = gym.get_actor_rigid_shape_properties(env, actor)
    for prop in properties:
        prop.friction = friction
        prop.rolling_friction = 0.20
        prop.torsion_friction = 0.20
        prop.restitution = 0.0
    gym.set_actor_rigid_shape_properties(env, actor, properties)


def _base_tf_message(parent, child, position, rotation, stamp):
    message = TransformStamped()
    message.header.stamp, message.header.frame_id = stamp, parent
    message.child_frame_id = child
    message.transform.translation.x = float(position[0])
    message.transform.translation.y = float(position[1])
    message.transform.translation.z = float(position[2])
    quaternion = quaternion_from_rotation(rotation)
    message.transform.rotation.x = float(quaternion[0])
    message.transform.rotation.y = float(quaternion[1])
    message.transform.rotation.z = float(quaternion[2])
    message.transform.rotation.w = float(quaternion[3])
    return message


def current_camera_state(lock_from_pin):
    """由实时标签位姿得到相机相对插销的半径、高度和倾角。"""
    pin_from_lock = np.linalg.inv(lock_from_pin)
    camera_position = (
        pin_from_lock
        @ np.r_[CAMERA_POSITION_IN_LOCK, 1.0]
    )[:3]
    optical_axis = (
        pin_from_lock[:3, :3]
        @ LOCK_FROM_CAMERA_ROTATION[:, 2]
    )
    tilt = np.rad2deg(np.arccos(np.clip(-optical_axis[2], -1.0, 1.0)))
    return np.linalg.norm(camera_position[:2]), camera_position[2], tilt


def detect_pin_pose(
    image, camera_matrix, distortion, dictionary, parameters,
    sensor_position, sensor_rotation, body_position, body_rotation,
):
    """由仿真相机画面直接计算插销相对卡紧机构的位姿。"""
    corners, ids, _ = cv2.aruco.detectMarkers(
        cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),
        dictionary, parameters=parameters,
    )
    if ids is None:
        return None, (), corners, ids

    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners, TAG_SIZE_M, camera_matrix, distortion
    )
    world_from_optical = (
        sensor_rotation @ ISAAC_FROM_OPTICAL_ROTATION
    )
    positions, rotations, used_ids = [], [], []
    for tag_id, rvec, tvec in zip(
        ids.reshape(-1), rvecs.reshape(-1, 3), tvecs.reshape(-1, 3)
    ):
        tag_id = int(tag_id)
        if tag_id not in PIN_FROM_TAG:
            continue
        pin_rotation = (
            world_from_optical
            @ cv2.Rodrigues(rvec)[0]
            @ PIN_FROM_OPENCV_TAG_ROTATION
        )
        tag_position = sensor_position + world_from_optical @ tvec
        positions.append(
            tag_position - pin_rotation @ PIN_FROM_TAG[tag_id]
        )
        rotations.append(pin_rotation)
        used_ids.append(tag_id)
    if not positions:
        return None, (), corners, ids

    left, _, right = np.linalg.svd(np.mean(rotations, axis=0))
    left[:, -1] *= np.linalg.det(left @ right)
    result = np.eye(4)
    result[:3, :3] = body_rotation.T @ (left @ right)
    result[:3, 3] = body_rotation.T @ (
        np.mean(positions, axis=0) - body_position
    )
    return result, tuple(sorted(used_ids)), corners, ids


class SimulationResources:
    """集中释放Isaac Gym和OpenCV资源，异常退出时同样生效。"""

    def __init__(self):
        self.gym = self.sim = self.viewer = None

    def close(self):
        gym, sim, viewer = self.gym, self.sim, self.viewer
        self.gym = self.sim = self.viewer = None
        try:
            if gym is not None and viewer is not None:
                gym.destroy_viewer(viewer)
        finally:
            try:
                if gym is not None and sim is not None:
                    gym.destroy_sim(sim)
            finally:
                cv2.destroyAllWindows()


def _run_simulation(resources):
    rospy.init_node("run_sim_dock", anonymous=False, disable_signals=False)
    joint_publisher = rospy.Publisher("/joint_states", JointState, queue_size=1)
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    gym = resources.gym = gymapi.acquire_gym()
    params = gymapi.SimParams()
    params.dt, params.substeps = dt, 2
    params.up_axis = gymapi.UP_AXIS_Z
    params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)
    params.use_gpu_pipeline = True
    params.physx.use_gpu = True
    params.physx.solver_type = 1
    params.physx.num_position_iterations = 4
    params.physx.num_velocity_iterations = 1
    sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, params)
    resources.sim = sim
    if sim is None:
        raise RuntimeError("Isaac Gym仿真创建失败")

    plane = gymapi.PlaneParams()
    plane.normal = gymapi.Vec3(0.0, 0.0, 1.0)
    plane.static_friction, plane.dynamic_friction = 4.0, 3.0
    gym.add_ground(sim, plane)

    # 适度提高方向光和环境光，同时避免AprilTag白区过曝反光。
    for light in range(4):
        gym.set_light_parameters(
            sim, light,
            gymapi.Vec3(*(0.30, 0.30, 0.30) if light == 0 else (0.0, 0.0, 0.0)),
            gymapi.Vec3(*(0.75, 0.75, 0.75) if light == 0 else (0.0, 0.0, 0.0)),
            gymapi.Vec3(0.0, 0.0, 1.0),
        )

    xiaolan_root = XIAOLAN_ROOT
    # 模块级地形查询和PhysX碰撞生成复用同一份STL解析结果。
    back_surface = _back_surface
    add_xiaolan_collision(gym, sim, back_surface)
    viewer = gym.create_viewer(sim, gymapi.CameraProperties())
    resources.viewer = viewer
    if viewer is None:
        raise RuntimeError("Isaac Gym窗口创建失败")

    scripts = Path(__file__).resolve().parent
    description = scripts.parents[1] / "grasp_hexapod_description"
    robot_asset = load_asset(
        gym, sim, description, "urdf/hexapod_isaacgym_view.urdf", False
    )
    xiaolan_asset = load_asset(
        gym, sim, xiaolan_root, "urdf/newxiaolan2id_dock_mode5.urdf", True
    )

    env = gym.create_env(
        sim, gymapi.Vec3(-1.0, -1.0, 0.0), gymapi.Vec3(1.0, 1.0, 1.0), 1
    )
    xiaolan = gym.create_actor(
        env, xiaolan_asset, gymapi.Transform(), "xiaolan", 1, 1
    )
    set_contact_friction(gym, env, xiaolan, 5.0)

    controller = GraspController(dt=dt * control_interval)
    controller.approach_mode.step_height = 0.008
    controller.approach_mode.phase_duration = 0.45
    print("读取compact攀爬终态并重建对接初始关节分支...")
    climb_terminal = load_climb_terminal_state(controller)
    q_start, terminal_clearance_error = initialize_from_climb_terminal(
        controller, climb_terminal, back_surface
    )
    initial_rotation = climb_terminal.base_rotation
    base_position = climb_terminal.base_position
    robot_pose = gymapi.Transform()
    robot_pose.p = gymapi.Vec3(*base_position)
    robot_pose.r = gymapi.Quat(
        *quaternion_from_rotation(initial_rotation)
    )
    robot = gym.create_actor(
        env, robot_asset, robot_pose, "grasp_hexapod", 0, 1
    )
    set_contact_friction(gym, env, robot, 6.0)

    # 独立感知环境只渲染小蓝，避免机器人简化网格错误遮住实体镜头。
    camera_env = gym.create_env(
        sim, gymapi.Vec3(-1.0, -1.0, 0.0), gymapi.Vec3(1.0, 1.0, 1.0), 2
    )
    gym.create_actor(
        camera_env, xiaolan_asset, gymapi.Transform(), "camera_xiaolan", 2, 1
    )

    dof_names = gym.get_actor_dof_names(env, robot)
    dof_indices = build_dof_indices(dof_names)
    properties = gym.get_actor_dof_properties(env, robot)
    properties["driveMode"].fill(int(gymapi.DOF_MODE_POS))
    properties["stiffness"].fill(4000.0)
    properties["damping"].fill(80.0)
    properties["effort"].fill(150.0)
    gym.set_actor_dof_properties(env, robot, properties)
    lower = external_to_control(properties["lower"], dof_indices)
    upper = external_to_control(properties["upper"], dof_indices)

    if (
        np.any(q_start < lower - 1e-9)
        or np.any(q_start > upper + 1e-9)
    ):
        raise RuntimeError("攀爬终态关节角超出对接模型关节限位")
    states = gym.get_actor_dof_states(env, robot, gymapi.STATE_ALL)
    states["pos"][:] = control_to_external(q_start, dof_indices)
    states["vel"][:] = 0.0
    gym.set_actor_dof_states(env, robot, states, gymapi.STATE_ALL)
    gym.set_actor_dof_position_targets(
        env, robot, control_to_external(q_start, dof_indices)
    )

    # 底部相机固定在base_link，光轴始终沿机身-z方向。
    props = gymapi.CameraProperties()
    props.width, props.height = camera_width, camera_height
    props.horizontal_fov = camera_fov
    props.near_plane, props.far_plane = 0.001, 10.0
    sensor = gym.create_camera_sensor(camera_env, props)
    if sensor < 0:
        raise RuntimeError("底部相机创建失败")
    base_index = gym.find_actor_rigid_body_index(
        env, robot, "base_link", gymapi.DOMAIN_SIM
    )
    foot_indices = [
        gym.find_actor_rigid_body_index(
            env, robot, name + "_foot_link", gymapi.DOMAIN_SIM
        )
        for name in ("lb", "lf", "lm", "rb", "rf", "rm")
    ]
    if min(foot_indices) < 0:
        raise RuntimeError("无法找到六个足端刚体")
    mount_rotation = (
        LOCK_FROM_CAMERA_ROTATION
        @ ISAAC_FROM_OPTICAL_ROTATION.T
    )
    ros_camera = RosCameraPublisher(
        camera_width, camera_height, camera_fov,
        "/usb_cam/image_raw", "/usb_cam/camera_info", "isaac_camera",
    )
    dictionary, detector_parameters = create_apriltag_detector()
    camera_matrix, distortion = calculate_intrinsics(
        camera_width, camera_height, camera_fov
    )
    camera_window = "Bottom Camera - AprilTag"
    cv2.namedWindow(camera_window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(camera_window, 960, 540)

    gym.prepare_sim(sim)
    dof_state_tensor = gymtorch.wrap_tensor(
        gym.acquire_dof_state_tensor(sim)
    )
    rigid_body_tensor = gymtorch.wrap_tensor(
        gym.acquire_rigid_body_state_tensor(sim)
    )
    gym.refresh_dof_state_tensor(sim)
    gym.refresh_rigid_body_state_tensor(sim)
    position_targets = torch.zeros(
        gym.get_sim_dof_count(sim),
        dtype=torch.float32,
        device=dof_state_tensor.device,
    )
    gym.viewer_camera_look_at(
        viewer, None,
        gymapi.Vec3(0.65, -0.65, 0.55),
        gymapi.Vec3(0.0, 0.0, 0.20),
    )
    joystick = JoyStick()
    print("手柄移动默认暂停；A：启用原三角步态  B：回到站姿  X：进入对接")
    print(
        "攀爬终态已交接：阶段={}，底盘相对小蓝[mm]={}".format(
            climb_terminal.stage_name,
            np.round(base_position * 1000.0, 2).tolist(),
        )
    )
    print(
        "交接相机坐标 [mm]：{}，FK连续误差={:.6f} mm，"
        "STL间隙最大误差={:.3f} mm".format(
            np.round(climb_terminal.camera_position * 1000.0, 2).tolist(),
            climb_terminal.max_fk_error_m * 1000.0,
            float(np.max(terminal_clearance_error)) * 1000.0,
        )
    )

    max_linear_speed, max_vertical_speed = 0.05, 0.01
    foot_radius = np.mean(np.linalg.norm(controller.foot_init_base[:, :2], axis=1))
    max_yaw_rate = max_linear_speed / foot_radius
    command = np.zeros(4)
    enabled, dock_finished = False, False
    descent_announced, descent_mm = False, 0.0
    previous_a = previous_b = previous_x = False
    x_armed = False
    pending_dock_entry = None
    tag_reacquire_attempted = False
    tag_reacquire_wait_reported = False
    dock_plan, dock_start_step, step = None, 0, 0
    support_settle_announced = False
    q_target = q_start.copy()
    last_ids = []
    local_pin_pose, local_pin_ids, local_pin_stamp = None, (), -np.inf
    realtime_start = time.perf_counter()
    wall_start = time.perf_counter()
    camera_cost = 0.0
    control_cost = 0.0

    while not gym.query_viewer_has_closed(viewer) and not rospy.is_shutdown():
        gym.refresh_dof_state_tensor(sim)
        gym.refresh_rigid_body_state_tensor(sim)
        q_current = external_to_control(
            dof_state_tensor[:, 0].detach().cpu().numpy(), dof_indices
        )
        getattr(controller, "observe_actual_state", lambda _: None)(q_current)
        body_state = rigid_body_tensor[base_index].detach().cpu().numpy()
        body_position = body_state[:3]
        body_rotation = body.quaternion_matrix(body_state[3:7])[:3, :3]

        # RViz/AprilTag节点所需的仿真关节状态。
        joint_message = JointState()
        joint_message.header.stamp = rospy.Time.now()
        joint_message.name = list(body.joint_names)
        joint_message.position = q_current.reshape(-1).tolist()
        joint_publisher.publish(joint_message)

        _, right, forward, _, yaw = joystick.get_commands()
        up = joystick.get_height_command()
        a = joystick.get_button(0)
        b = joystick.get_button(1)
        # 当前北通手柄由Linux明确报告物理X为BtnX（按钮3）。
        x = joystick.get_button(3)
        if step > 30 and not x:
            x_armed = True
        press_a = a and not previous_a
        press_b = b and not previous_b
        # X只负责从APPROACH进入DOCK。对接轨迹执行或支撑末端等待期间
        # 完全忽略新的X边沿，避免重复入口请求重置稳定监视器。
        dock_entry_available = bool(
            controller.mode == controller.APPROACH
            and dock_plan is None
            and pending_dock_entry is None
        )
        manual_press_x = bool(
            dock_entry_available and x_armed and x and not previous_x
        )
        auto_press_x = bool(
            dock_entry_available
            and getattr(
                controller, "consume_dock_reposition_ready", lambda: False
            )()
        )
        request_dock_entry = manual_press_x or auto_press_x
        if request_dock_entry:
            x_armed = False
            pending_dock_entry = "manual" if manual_press_x else "auto"
            enabled = False
            if manual_press_x:
                getattr(controller, "reset_dock_reposition", lambda: None)()
                tag_reacquire_attempted = False
                tag_reacquire_wait_reported = False
            else:
                print("自主小步移动完成，等待步态结束并锁定真实足端")
            if (
                controller.approach_mode.gait_started
                or controller.approach_mode.transfer_active
            ):
                print("对接请求已接收：先完成当前摆腿并保持实际落脚状态")

        dock_entry_stable = bool(
            pending_dock_entry is not None
            and getattr(controller, "dock_entry_ready", lambda: (
                not controller.reset_active
                and not controller.approach_mode.gait_started
                and not controller.approach_mode.transfer_active
            ))()
        )
        complete_tag_ready = bool(
            local_pin_pose is not None
            and bool(local_pin_ids)
            and time.monotonic() - local_pin_stamp
            <= TAG_REACQUIRE_FRESH_S
        )
        press_x = bool(dock_entry_stable and complete_tag_ready)
        if dock_entry_stable and not complete_tag_ready:
            if not tag_reacquire_attempted:
                raise_distance = start_tag_reacquisition_raise(
                    controller,
                    q_current,
                    body_position,
                    body_rotation,
                )
                tag_reacquire_attempted = True
                tag_reacquire_wait_reported = False
                if raise_distance > 0.0:
                    print(
                        "未识别到完整AprilTag：保持六足落点，先将相机"
                        "平滑升高{:.1f} mm至约{:.1f} mm".format(
                            raise_distance * 1000.0,
                            TAG_REACQUIRE_CAMERA_HEIGHT_M * 1000.0,
                        )
                    )
                else:
                    print(
                        "相机已经达到标签重捕获高度，等待完整AprilTag"
                    )
            elif not controller.reset_active and not tag_reacquire_wait_reported:
                print(
                    "相机升高并已稳定：继续等待真实图像完整解码；"
                    "禁止使用仿真位姿补全直接进入对接"
                )
                tag_reacquire_wait_reported = True
        manual_dock_entry = press_x and pending_dock_entry == "manual"
        auto_dock_entry = press_x and pending_dock_entry == "auto"
        if press_x:
            pending_dock_entry = None
            print(
                "完整AprilTag已确认：ID={}，允许进入对接".format(
                    list(local_pin_ids)
                )
            )
        previous_a, previous_b, previous_x = a, b, x

        if press_a and controller.mode != controller.DOCK:
            enabled = not enabled
            print("手柄移动：", "启用" if enabled else "暂停")
            if enabled:
                controller.reset_to_stand(q_current)
        if press_b:
            pending_dock_entry = None
            tag_reacquire_attempted = False
            tag_reacquire_wait_reported = False
            getattr(controller, "cancel_dock_reposition", lambda: None)()
            if controller.mode == controller.DOCK:
                controller.set_mode(controller.APPROACH)
            enabled, dock_finished, dock_plan = False, False, None
            descent_announced, descent_mm = False, 0.0
            controller.reset_to_stand(q_current)
            print("正在回到初始站姿")
        if press_x and not press_b and controller.mode != controller.DOCK:
            if auto_dock_entry:
                print("真实足端已锁定且机身稳定，自主重新进入对接模式")
            elif manual_dock_entry:
                print("真实足端已锁定且机身稳定，进入对接模式")
            enabled = False
            controller.foot_desired_base = controller.kinematic.forward_base(q_current)
            controller.set_mode(controller.DOCK)
            controller.dock_mode.joints.values = q_current.copy()
            # press_x已由新鲜的完整本地解码门控；这里禁止调用latest()，
            # 因为SimPerception.latest()允许使用已知小蓝位姿补全残缺标签。
            pin = local_pin_pose.copy()
            controller.dock_mode.pin_pose = pin.copy()
            controller.dock_mode.perception.used_ids = local_pin_ids
            inject = getattr(
                controller.dock_mode.perception,
                "inject_complete",
                None,
            )
            if inject is not None:
                inject(pin, local_pin_ids)
            if pin is not None:
                print("识别标签ID：", controller.dock_mode.perception.used_ids)
                print("插销相对卡紧机构 [mm]：", np.round(pin[:3, 3] * 1000.0, 2).tolist())
                radius, height, tilt = current_camera_state(pin)
                descent_mm = -body.measured_pre_dock_pin(pin)[2] * 1000.0
                print(
                    "当前相机姿态：世界高度={:.1f} mm，相对插销半径={:.1f} mm，倾斜={:.2f}°".format(
                        (PIN_WORLD[2] + height) * 1000.0,
                        radius * 1000.0, tilt
                    )
                )
                print("按当前实测高度，对准后下降距离={:.1f} mm".format(descent_mm))
                aligned = (
                    abs(radius - 0.065) <= 0.003
                    and tilt <= 2.0
                    and descent_mm > 0.0
                )
                print(
                    "当前已达到预期姿态" if aligned else
                    "当前未达到预期姿态，将从实时姿态开始规划"
                )
            if controller.dock_mode.start():
                dock_plan = controller.dock_mode.plan
                dock_start_step, dock_finished = step, False
                descent_announced = False
                support_settle_announced = False
                if dock_plan.kind == "support":
                    print("先执行支撑调整轨迹")
                print_plan(dock_plan)
            else:
                print("不能进入对接模式：", controller.dock_mode.reason)
                request = getattr(
                    controller.dock_mode,
                    "consume_reposition_request",
                    lambda: None,
                )()
                controller.set_mode(controller.APPROACH)
                if request is not None:
                    controller.reset_to_stand(q_current)
                    started = getattr(
                        controller,
                        "start_dock_reposition",
                        lambda _: False,
                    )(request)
                    if not started:
                        print("无法继续自主可达位姿搜索，等待手柄调整")
                        getattr(
                            controller.dock_mode, "abort_trial", lambda _: None
                        )("自主可达位姿搜索没有找到安全路线")
                else:
                    getattr(
                        controller.dock_mode, "abort_trial", lambda _: None
                    )(controller.dock_mode.reason)

        terminal = getattr(
            controller.dock_mode, "poll_execution", lambda: None
        )()
        terminal_handled = terminal is not None
        if terminal_handled:
            success, reason = terminal
            dock_finished = True
            dock_plan = None
            pending_dock_entry = None
            tag_reacquire_attempted = False
            tag_reacquire_wait_reported = False
            support_settle_announced = False
            enabled = False
            getattr(controller, "cancel_dock_reposition", lambda: None)()
            if controller.mode == controller.DOCK:
                controller.set_mode(controller.APPROACH)
            q_target = q_current.copy()
            print(
                "对接成功：{}".format(reason) if success else
                "对接失败：{}".format(reason)
            )

        if dock_plan is not None:
            elapsed = (step - dock_start_step) * dt / dock_time_scale
            q_target = sample_plan(dock_plan, elapsed)
            if dock_plan.kind == "support":
                if elapsed >= dock_plan.times[-1]:
                    support_settled = bool(getattr(
                        controller,
                        "dock_support_plan_settled",
                        lambda _: True,
                    )(dock_plan.joints[-1]))
                    if not support_settled:
                        if not support_settle_announced:
                            print("支撑轨迹已结束，保持末端目标并等待真实关节和机身停止变化")
                            support_settle_announced = True
                        q_target = dock_plan.joints[-1].copy()
                    else:
                        support_settle_announced = False
                        leg_index = dock_plan.moving_leg
                        leg = body.LEG_NAMES[leg_index]
                        controller.dock_mode.joints.values = q_current.copy()
                        print("{}腿支撑调整完成，沿锁定路线续接下一局部轨迹".format(leg))
                        if getattr(
                            controller.dock_mode,
                            "continue_route",
                            lambda _: controller.dock_mode.start(),
                        )(q_current):
                            dock_plan = controller.dock_mode.plan
                            dock_start_step, dock_finished = step, False
                            support_settle_announced = False
                            print(
                                "继续调整下一条腿" if dock_plan.kind == "support"
                                else "开始完整对接轨迹"
                            )
                            print_plan(dock_plan)
                        else:
                            print("支撑调整失败：", controller.dock_mode.reason)
                            request = getattr(
                                controller.dock_mode,
                                "consume_reposition_request",
                                lambda: None,
                            )()
                            controller.set_mode(controller.APPROACH)
                            dock_plan = None
                            if request is not None:
                                controller.reset_to_stand(q_current)
                                started = getattr(
                                    controller,
                                    "start_dock_reposition",
                                    lambda _: False,
                                )(request)
                                if not started:
                                    print("无法继续自主可达位姿搜索，等待手柄调整")
                                    getattr(
                                        controller.dock_mode,
                                        "abort_trial",
                                        lambda _: None,
                                    )("自主可达位姿搜索没有找到安全路线")
                            else:
                                getattr(
                                    controller.dock_mode,
                                    "abort_trial",
                                    lambda _: None,
                                )(controller.dock_mode.reason)
            else:
                descent_started = bool(getattr(
                    controller.dock_mode,
                    "descent_has_started",
                    lambda: elapsed >= dock_plan.times[dock_plan.pre_dock_index],
                )())
                if not descent_announced and descent_started:
                    descent_announced = True
                    print("第一阶段预对准完成，开始第二阶段下降{:.1f} mm".format(descent_mm))
                if elapsed >= dock_plan.times[-1] and not dock_finished:
                    # 闭环对接必须由实际位置、姿态连续满足阈值后确认；
                    # 轨迹时间结束本身不代表插销已经进入卡紧机构。
                    q_target = dock_plan.joints[-1].copy()
        else:
            auto_command = (
                None if terminal_handled else
                getattr(
                    controller, "dock_reposition_command", lambda: None
                )()
            )
            settling_for_dock = pending_dock_entry is not None
            auto_control = auto_command is not None or settling_for_dock
            if settling_for_dock:
                command[:] = 0.0
            elif auto_command is not None:
                command[:] = auto_command
            else:
                axes = np.array((right, forward), dtype=float)
                axes /= max(1.0, np.linalg.norm(axes))
                command[:] = (
                    max_linear_speed * axes[0], max_linear_speed * axes[1],
                    max_vertical_speed * up, max_yaw_rate * yaw,
                ) if enabled and not controller.reset_active else 0.0
            if step % control_interval == 0 and (
                enabled or controller.reset_active or auto_control
            ):
                control_start = time.perf_counter()
                q_target = controller.update(q_current, command)
                control_cost += time.perf_counter() - control_start

        q_target = np.clip(q_target, lower, upper)
        position_targets.copy_(torch.as_tensor(
            control_to_external(q_target, dof_indices),
            dtype=torch.float32,
            device=position_targets.device,
        ))
        gym.set_dof_position_target_tensor(
            sim, gymtorch.unwrap_tensor(position_targets)
        )
        # 不写机器人根状态：手柄只驱动18个关节。
        gym.simulate(sim)
        gym.fetch_results(sim, True)
        gym.refresh_rigid_body_state_tensor(sim)

        body_state = rigid_body_tensor[base_index].detach().cpu().numpy()
        body_position = body_state[:3]
        body_rotation = body.quaternion_matrix(body_state[3:7])[:3, :3]
        sensor_position = (
            body_position + body_rotation @ CAMERA_POSITION_IN_LOCK
        )
        sensor_rotation = body_rotation @ mount_rotation
        sensor_pose = gymapi.Transform()
        sensor_pose.p = gymapi.Vec3(*sensor_position)
        sensor_pose.r = gymapi.Quat(
            *quaternion_from_rotation(sensor_rotation)
        )
        gym.set_camera_transform(sensor, camera_env, sensor_pose)

        if step % 120 == 60:
            foot_world = rigid_body_tensor[foot_indices].detach().cpu().numpy()
            print("机身世界坐标 [mm]：", np.round(body_position * 1000.0, 1).tolist())
            bottoms = (
                foot_world[:, 2] - FOOT_RADIUS
            )
            current_surface_z = back_surface.heights(foot_world[:, :2])
            print("六足世界XY [mm]：", np.round(foot_world[:, :2] * 1000.0, 1).tolist())
            print("六足底部高度 [mm]：", np.round(bottoms * 1000.0, 1).tolist())
            print("脚下实时背部高度 [mm]：", np.round(current_surface_z * 1000.0, 1).tolist())
        gym.step_graphics(sim)

        # 与CPU步态计算错帧，避免同一帧同时承担两项重任务。
        if step % camera_interval == 1:
            camera_start = time.perf_counter()
            gym.render_all_camera_sensors(sim)
            rgba = gym.get_camera_image(
                sim, camera_env, sensor, gymapi.IMAGE_COLOR
            )
            rgba = np.asarray(rgba, dtype=np.uint8).reshape(
                camera_height, camera_width, 4
            )
            bgr = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
            ros_camera.publish(bgr)

            # 独立显示机器人底部相机视角，并以10 Hz计算标签位姿。
            display = bgr.copy()
            if step % 6 == 1:
                pose, ids, corners, raw_ids = detect_pin_pose(
                    bgr, camera_matrix, distortion,
                    dictionary, detector_parameters,
                    sensor_position, sensor_rotation,
                    body_position, body_rotation,
                )
                last_ids = list(ids)
                if raw_ids is not None:
                    cv2.aruco.drawDetectedMarkers(display, corners, raw_ids)
                if pose is not None:
                    if local_pin_pose is None:
                        print(
                            "仿真相机标签位姿已就绪：ID={}，插销相对卡紧机构={} mm".format(
                                list(ids),
                                np.round(pose[:3, 3] * 1000.0, 2).tolist(),
                            )
                        )
                    local_pin_pose = pose
                    local_pin_ids = ids
                    local_pin_stamp = time.monotonic()
                    inject = getattr(
                        getattr(
                            getattr(controller, "dock_mode", None),
                            "perception",
                            None,
                        ),
                        "inject_complete",
                        None,
                    )
                    if inject is not None:
                        inject(pose, ids)
                    # DOCK中若仍能看到标签，后续支撑调整使用最新位姿。
                    if controller.mode == controller.DOCK:
                        controller.dock_mode.pin_pose = pose.copy()
                        controller.dock_mode.perception.used_ids = ids
            cv2.putText(
                display,
                "AprilTag IDs: {} | pose: {}".format(
                    last_ids or "none",
                    "ready" if time.monotonic() - local_pin_stamp <= 1.0
                    else "missing",
                ),
                (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                (0, 255, 255), 2, cv2.LINE_AA,
            )

            # RViz TF：base_footprint -> base_link -> isaac_camera -> tag。
            stamp = rospy.Time.now()
            tf_broadcaster.sendTransform([
                tf_message(
                    "base_footprint", "base_link",
                    body_position,
                    body_rotation, stamp,
                ),
                tf_message(
                    "base_link", "isaac_camera",
                    CAMERA_POSITION_IN_LOCK,
                    LOCK_FROM_CAMERA_ROTATION,
                    stamp,
                ),
            ])
            cv2.imshow(camera_window, display)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            camera_cost += time.perf_counter() - camera_start

        # Viewer按30 Hz绘制，物理仍为60 Hz；中间帧只轮询事件。
        if step % 2 == 0:
            gym.draw_viewer(viewer, sim, True)
        else:
            gym.poll_viewer_events(viewer)
        step += 1
        wait = realtime_start + step * dt - time.perf_counter()
        if wait > 0.0:
            time.sleep(wait)
        if step % 120 == 0:
            elapsed_wall = time.perf_counter() - wall_start
            print(
                "仿真实际频率：{:.1f} Hz；相机平均耗时：{:.1f} ms".format(
                    120.0 / elapsed_wall,
                    camera_cost * 1000.0 / (120 // camera_interval),
                )
            )
            print(
                "控制器平均耗时：{:.1f} ms".format(
                    control_cost * 1000.0 / (120 // control_interval)
                )
            )
            wall_start = time.perf_counter()
            camera_cost = 0.0
            control_cost = 0.0


def main():
    resources = SimulationResources()
    try:
        _run_simulation(resources)
    finally:
        resources.close()


class SimPerception:
    """向旧仿真循环提供矩阵，同时保留标准感知结果。"""

    def __init__(self, perception, fallback):
        self.core, self.fallback = perception, fallback
        self.used_ids = ()
        self.local_result = None
        self.local_result_time = 0.0

    def reset(self):
        self.core.reset()
        self.used_ids = ()

    def inject_complete(self, pose, decoded_ids, confidence=0.72):
        """把仿真相机实际解码结果送入与实机相同的完整ID路径。"""
        decoded_ids = tuple(sorted(int(tag_id) for tag_id in decoded_ids))
        if not decoded_ids:
            return
        self.local_result = PerceptionResult(
            True,
            np.asarray(pose, dtype=float).reshape(4, 4).copy(),
            rospy.Time.now(), decoded_ids, (), float(confidence),
            reason="Isaac Gym decoded complete tag id",
        )
        self.local_result_time = time.monotonic()

    def latest_result(self):
        result = self.core.latest()
        if (
            not result.valid
            and self.local_result is not None
            and time.monotonic() - self.local_result_time <= 1.0
        ):
            result = self.local_result
        if not result.valid:
            pose = self.fallback()
            if pose is not None:
                result = PerceptionResult(
                    True, pose, rospy.Time.now(), (), TAG_IDS, 0.55,
                    reason="Isaac Gym incomplete-tag pose",
                )
        self.used_ids = result.decoded_ids + result.inferred_ids
        return result

    def latest(self):
        result = self.latest_result()
        return result.lock_from_pin if result.valid else None


class SimDockMode(body.DockMode):
    """仅适配联合仿真接口，不改变正式对接主程序。"""

    STATISTICS_PATH = Path(os.environ.get(
        "DOCK_SIM_STATS_PATH",
        str(Path.home() / ".ros/dock_success_trials.csv"),
    ))
    _statistics_loaded = False
    _trial_statistics = {
        "complete": {"attempts": 0, "successes": 0},
        "incomplete": {"attempts": 0, "successes": 0},
    }

    def __init__(self, controller):
        global _dock_instance
        self._pin_pose = None
        self._terrain_ready = False
        self._observed = None
        self._support_world_feet = None
        self._terrain_attempts = np.zeros(6, dtype=int)
        self._terrain_previous_gap = np.full(6, np.inf)
        self._reposition_request = None
        self._normalize_support = False
        self._trial_active = False
        self._trial_kind = None
        self._trial_decoded_ids = ()
        self._trial_inferred_ids = ()
        self._trial_confidence = 0.0
        super().__init__(controller)
        self.configure_motion_planner(dock_time_scale)
        self.perception = SimPerception(self.perception, self.simulated_pin_pose)
        _dock_instance = self

    def simulated_pin_pose(self):
        """仿真中标签残缺时，用已知小蓝位姿补全插销位姿。"""
        if _body_pose[0] is None:
            return None
        world_from_lock = body.transform(*_body_pose)
        return np.linalg.inv(world_from_lock) @ body.transform(
            PIN_WORLD
        )

    @property
    def pin_pose(self):
        return self._pin_pose

    @pin_pose.setter
    def pin_pose(self, pose):
        self._pin_pose = pose

    def enter(self, foot_positions_base):
        self._terrain_ready = False
        self._observed = self._support_world_feet = None
        self._terrain_attempts[:] = 0
        self._terrain_previous_gap[:] = np.inf
        self._reposition_request = None
        self._normalize_support = bool(getattr(
            self.controller,
            "consume_dock_reposition_normalization",
            lambda: False,
        )())
        super().enter(foot_positions_base)
        if not self._normalize_support:
            # 上层已经在步态结束后锁定了真实足端并确认机身稳定。
            # STL几何高度差不是接触传感器，不能在这里再次把毫米级
            # 模型误差解释为悬腿并触发单腿落脚。
            self._terrain_ready = True
            print("使用已锁定的真实足端进入对接，不再按STL高度二次判断悬腿")

    def exit(self):
        self._observed = None
        super().exit()

    def reset_trial(self):
        """手动开始新试验时清除上一次未完成试验，不计入失败率。"""
        self.motion_planner.begin_session()
        self._trial_active = False
        self._trial_kind = None
        self._trial_decoded_ids = ()
        self._trial_inferred_ids = ()
        self._trial_confidence = 0.0

    def cancel_trial(self):
        """用户按B取消时清空计时，不把主动取消计入失败率。"""
        self.motion_planner.cancel_session()
        self._trial_active = False
        self._trial_kind = None

    @classmethod
    def _load_statistics(cls):
        if cls._statistics_loaded:
            return
        cls._statistics_loaded = True
        if not cls.STATISTICS_PATH.exists():
            return
        try:
            with cls.STATISTICS_PATH.open("r", newline="", encoding="utf-8") as stream:
                for row in csv.DictReader(stream):
                    kind = row.get("recognition")
                    if kind not in cls._trial_statistics:
                        continue
                    cls._trial_statistics[kind]["attempts"] += 1
                    cls._trial_statistics[kind]["successes"] += int(
                        row.get("success", "0") == "1"
                    )
        except (OSError, csv.Error, ValueError) as error:
            print("读取历史对接统计失败，本次仅使用内存统计：{}".format(error))

    def _append_trial_record(self, success, reason, duration):
        path = self.STATISTICS_PATH
        fields = (
            "time", "recognition", "success", "duration_s", "reason",
            "decoded_ids", "inferred_ids", "confidence", "segments",
            "local_corrections", "reposition_attempts",
        )
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            exists = path.exists() and path.stat().st_size > 0
            with path.open("a", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                if not exists:
                    writer.writeheader()
                writer.writerow({
                    "time": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "recognition": self._trial_kind,
                    "success": int(bool(success)),
                    "duration_s": "{:.3f}".format(duration),
                    "reason": reason,
                    "decoded_ids": " ".join(map(str, self._trial_decoded_ids)),
                    "inferred_ids": " ".join(map(str, self._trial_inferred_ids)),
                    "confidence": "{:.3f}".format(self._trial_confidence),
                    "segments": self.motion_planner.plan_step,
                    "local_corrections": self.motion_planner.local_corrections,
                    "reposition_attempts": getattr(
                        self.controller, "_dock_reposition_attempts", 0
                    ),
                })
        except OSError as error:
            print("写入对接统计失败：{}".format(error))

    def _begin_trial(self, observed):
        if self._trial_active:
            return
        self._load_statistics()
        self.motion_planner.ensure_session()
        complete = bool(observed.decoded_ids) and not observed.inferred_ids
        self._trial_kind = "complete" if complete else "incomplete"
        now = time.monotonic()
        self.motion_planner.set_phase("路线计算")
        self._trial_decoded_ids = tuple(observed.decoded_ids)
        self._trial_inferred_ids = tuple(observed.inferred_ids)
        self._trial_confidence = float(observed.confidence)
        self._trial_active = True
        label = "完整ID" if complete else "不完整ID"
        print("对接试验开始：{}，本次识别置信度={:.2f}".format(
            label, observed.confidence
        ))
        print("识别完成：按下X后用时{:.2f}s".format(
            self.motion_planner.elapsed
        ))

    def set_trial_phase(self, phase):
        """切换试验阶段，并输出上一阶段耗时。"""
        if not self._trial_active:
            return
        previous = self.motion_planner.set_phase(phase)
        if previous is not None:
            print("{}完成：耗时{:.2f}s".format(
                previous[0], previous[1]
            ))

    @classmethod
    def _statistics_text(cls):
        def rate(kind):
            item = cls._trial_statistics[kind]
            if not item["attempts"]:
                return "0/0（暂无样本）"
            return "{}/{}（{:.1f}%）".format(
                item["successes"], item["attempts"],
                100.0 * item["successes"] / item["attempts"],
            )

        complete = cls._trial_statistics["complete"]
        incomplete = cls._trial_statistics["incomplete"]
        total_attempts = complete["attempts"] + incomplete["attempts"]
        total_successes = complete["successes"] + incomplete["successes"]
        total = (
            "0/0（暂无样本）" if not total_attempts else
            "{}/{}（{:.1f}%）".format(
                total_successes, total_attempts,
                100.0 * total_successes / total_attempts,
            )
        )
        return "完整识别={}；不完整识别={}；全部状态={}".format(
            rate("complete"), rate("incomplete"), total
        )

    def finish_trial(self, success, reason):
        """只记录一次终态，并输出三类对接成功率。"""
        if not self._trial_active:
            return
        item = self._trial_statistics[self._trial_kind]
        item["attempts"] += 1
        item["successes"] += int(bool(success))
        duration = self.motion_planner.elapsed
        self._append_trial_record(success, reason, duration)
        label = "成功" if success else "失败"
        if self.motion_planner.phase_started > 0.0:
            print("{}结束：本阶段耗时{:.2f}s".format(
                self.motion_planner.phase,
                time.monotonic() - self.motion_planner.phase_started,
            ))
        print("对接试验{}：{}；总耗时={:.1f}s".format(
            label, reason, duration
        ))
        print("对接成功率：{}".format(self._statistics_text()))
        self._trial_active = False

    def abort_trial(self, reason):
        self.finish_trial(False, reason)

    def _request_reposition(self, observed, reason):
        """请求上层搜索可达位姿，先移动整机，停稳后再对接。"""
        return self._request_reposition_pose(
            observed.lock_from_pin, reason
        )

    def _request_reposition_pose(self, lock_from_pin, reason):
        """使用锁定目标在当前机身坐标系中的位姿请求整机重定位。"""
        self.state = "reposition"
        self.reason = reason
        self._reposition_request = {
            "lock_from_pin": np.asarray(
                lock_from_pin, dtype=float
            ).copy(),
            "reason": reason,
            "joints": (
                None if self.joints.values is None
                else self.joints.values.copy()
            ),
        }
        self.status_publisher.publish(reason)
        return False

    def consume_reposition_request(self):
        """只让联合仿真主循环消费一次自主移动请求。"""
        request, self._reposition_request = self._reposition_request, None
        return request

    def terrain_plan(self):
        """只处理越界恢复和自主移动后的标准支撑归位。"""
        joints = self.joints.values
        if joints is None or _body_pose[0] is None:
            return None
        world_from_lock = body.transform(*_body_pose)
        feet = body.GraspKinematic().forward_base(joints)
        world = feet @ world_from_lock[:3, :3].T + world_from_lock[:3, 3]
        surface = _back_surface.heights(world[:, :2])
        gaps = world[:, 2] - FOOT_RADIUS - surface
        valid = np.isfinite(gaps)
        if not valid.all():
            leg = int(np.where(~valid)[0][0])
            plan = self._recover_leg_plan(joints, world, leg)
            self.plan, self.state, self.reason = plan, "adjusting", "terrain recovery"
            self.trajectory_publisher.publish(body.trajectory_message(plan))
            print("越界恢复：{}腿自动移动到小蓝背部安全位置".format(
                body.LEG_NAMES[leg]
            ))
            return plan
        if self._normalize_support:
            # 自主重定位结束后，把六足逐条放回统一的标准支撑扇区。
            # 二维搜索正是用该构型验证完整对接轨迹，因此执行后预测与
            # 实际足端锚点一致，不受三角步态停车相位影响。
            nominal = body.GraspKinematic().forward_base(Q_STAND)
            target_world = nominal @ world_from_lock[:3, :3].T + world_from_lock[:3, 3]
            target_surface = np.array([
                np.nan if height is None else float(height)
                for height in (
                    _safe_surface_height(xy, margin=0.008)
                    for xy in target_world[:, :2]
                )
            ], dtype=float)
            if not np.isfinite(target_surface).all():
                raise RuntimeError("标准支撑构型存在不安全落脚点")
            target_world[:, 2] = (
                target_surface + FOOT_RADIUS - 0.0015
            )
            planar_error = np.linalg.norm(
                target_world[:, :2] - world[:, :2], axis=1
            )
            pending = np.where(
                planar_error
                > self.controller.DOCK_SUPPORT_NORMALIZE_TOLERANCE
            )[0]
            if len(pending):
                plan = None
                failures = []
                # 最大误差腿的直接轨迹可能越过关节边界。按误差从大到小
                # 逐腿尝试；直达失败时先落到该腿扇区中的可达中间点。
                for leg in pending[np.argsort(
                    planar_error[pending]
                )[::-1]]:
                    leg = int(leg)
                    target = (
                        np.linalg.inv(world_from_lock)
                        @ np.r_[target_world[leg], 1.0]
                    )[:3]
                    try:
                        plan = body.place_leg_plan(
                            joints, leg, target, lift=0.023
                        )
                    except body.IKError as error:
                        try:
                            plan = self._recover_leg_plan(
                                joints, world, leg,
                                minimum_move=0.003,
                                standard_stance=True,
                            )
                        except RuntimeError as recovery_error:
                            failures.append("{}；{}".format(
                                error, recovery_error
                            ))
                            continue
                        print(
                            "{}腿标准落脚直达不可达，先移动到安全中间落脚点".format(
                                body.LEG_NAMES[leg]
                            )
                        )
                    break
                if plan is None:
                    raise RuntimeError(
                        "六足标准化均无可达落脚轨迹：{}".format(
                            " | ".join(failures)
                        )
                    )
                self.plan, self.state, self.reason = (
                    plan, "adjusting", "normalize support"
                )
                self.trajectory_publisher.publish(
                    body.trajectory_message(plan)
                )
                print(
                    "自主移动后标准化支撑：移动{}腿 {:.1f} mm".format(
                        body.LEG_NAMES[leg], planar_error[leg] * 1000.0
                    )
                )
                return plan
            self._normalize_support = False
            print("自主移动后的六足标准支撑构型已经建立")
        self._terrain_ready = True
        print("支撑准备完成：保持真实足端，不再按STL高度差判断悬腿")
        return None

    def _recover_leg_plan(
        self, joints, world, leg, minimum_move=0.0,
        standard_stance=False,
    ):
        """用RTK、IMU和关节位置把越界腿收回可达的安全表面。"""
        position, rotation = _body_pose
        nominal_base = (
            body.GraspKinematic().forward_base(Q_STAND)[leg]
            if standard_stance else self.controller.foot_init_base[leg]
        )
        nominal = (
            position + nominal_base @ rotation.T
        )
        world_from_lock = body.transform(position, rotation)
        lock_from_world = np.linalg.inv(world_from_lock)
        for xy, height in _leg_surface_candidates(
            world[leg, :2], position[:2], nominal[:2]
        ):
            if np.linalg.norm(xy - world[leg, :2]) < minimum_move:
                continue
            target_world = np.array(
                (xy[0], xy[1], height + FOOT_RADIUS - 0.0015)
            )
            target = (lock_from_world @ np.r_[target_world, 1.0])[:3]
            try:
                return body.place_leg_plan(joints, leg, target, lift=0.015)
            except body.IKError:
                continue
        raise RuntimeError(
            "{}腿附近没有可达的安全落脚点".format(body.LEG_NAMES[leg])
        )

    def place_support_foot(self, foot, lock_from_pin, leg):
        """把新落脚点投影到小蓝STL的真实表面。"""
        world_from_lock = body.transform(
            PIN_WORLD
        ) @ np.linalg.inv(lock_from_pin)
        world = world_from_lock @ np.r_[foot, 1.0]
        height = _safe_surface_height(world[:2])
        if height is None:
            position, rotation = world_from_lock[:3, 3], world_from_lock[:3, :3]
            nominal = (
                position[:2]
                + self.controller.foot_init_base[leg, :2]
                @ rotation[:2, :2].T
            )
            candidate = next(_leg_surface_candidates(
                world[:2], position[:2], nominal
            ), None)
            if candidate is None:
                raise body.IKError(leg, 0.180)
            world[:2], height = candidate
        world[2] = height + FOOT_RADIUS - 0.0015
        return (np.linalg.inv(world_from_lock) @ world)[:3]

    def _precompute_plans(self, observed, max_adjustments=None):
        """机器人不动时，先确认调足后完整对接轨迹一定可达。"""
        joints = self.joints.values
        if joints is None:
            raise RuntimeError("缺少关节状态")
        pose = observed.lock_from_pin
        if body.measured_pre_dock_pin(pose)[2] >= 0.0:
            raise RuntimeError("卡紧机构中心已经低于插销")
        anchors = body.GraspKinematic().forward_base(joints)
        plans = []
        limit = 6 if max_adjustments is None else max(0, max_adjustments)
        for adjustment in range(limit + 1):
            try:
                dock_plan = body.plan_trajectory(pose, joints, anchors)
            except body.IKError as error:
                if adjustment == limit:
                    raise RuntimeError(
                        "剩余{}次调足仍不可达：{}".format(limit, error)
                    )
                try:
                    support = body.support_plan(
                        pose, joints, anchors, error,
                        place_foot=self.place_support_foot,
                    )
                except body.IKError as support_error:
                    # 外展轨迹自身不可达时，当前腿往往已经接近
                    # 关节极限。先把它收回标准站姿对应的安全扇区，
                    # 执行后按真实关节状态续接锁定路线。
                    position, rotation = _body_pose
                    world = anchors @ rotation.T + position
                    try:
                        support = self._recover_leg_plan(
                            joints,
                            world,
                            support_error.leg,
                            minimum_move=0.015,
                            standard_stance=True,
                        )
                    except RuntimeError as recovery_error:
                        raise RuntimeError(
                            "支撑外展不可达：{}；内收重置也失败：{}".format(
                                support_error, recovery_error
                            )
                        )
                    print(
                        "{}腿外展不可达，改为自动内收到标准安全支撑位姿".format(
                            body.LEG_NAMES[support_error.leg]
                        )
                    )
                plans.append(support)
                joints, anchors = support.joints[-1], support.anchors
            else:
                plans.append(dock_plan)
                return plans
        raise RuntimeError("无法生成完整对接轨迹")

    def _lock_route(self, observed, plans):
        self.motion_planner.lock_route(
            observed.lock_from_pin, plans, tuple(_body_pose)
        )

    def _current_locked_pose(self):
        fallback = self.simulated_pin_pose()
        return self.motion_planner.current_target(
            tuple(_body_pose), fallback
        )

    def _publish_active_plan(self):
        self.motion_planner.note_plan(self.plan)
        self.state = "adjusting" if self.plan.kind == "support" else "executing"
        self.reason = "trajectory published"
        if self.plan.kind == "support":
            self.adjustments += 1
            self._capture_support_feet()
            print(
                "支撑调整 {}/{}：移动{}腿".format(
                    self.adjustments, self.motion_planner.support_total,
                    body.LEG_NAMES[self.plan.moving_leg],
                )
            )
        else:
            self.foot_anchors_base = self.plan.anchors
            if not self._prepare_feedback_descent():
                return False
            print("支撑调整完成，开始实际位姿闭环对接")
        self.trajectory_publisher.publish(body.trajectory_message(self.plan))
        self.status_publisher.publish(
            "step {}/{} kind={} remaining={}".format(
                self.motion_planner.plan_step,
                self.motion_planner.plan_step + len(self.motion_planner.plans),
                self.plan.kind,
                len(self.motion_planner.plans),
            )
        )
        return True

    def _activate_local_correction(self, pose, error):
        """完整路线受真实误差影响时，只插入有限次单腿局部纠偏。"""
        if not self.motion_planner.allow_local_correction():
            reason = (
                "局部纠偏已达{}次上限，仍不可达：{}".format(
                    self.motion_planner.config.correction_limit, error
                )
            )
            return self._request_reposition_pose(pose, reason)
        joints = self.joints.values
        anchors = body.GraspKinematic().forward_base(joints)
        try:
            correction = body.support_plan(
                pose, joints, anchors, error,
                lift=0.015,
                place_foot=self.place_support_foot,
            )
        except body.IKError as support_error:
            reason = "局部纠偏轨迹不可达：{}".format(support_error)
            return self._request_reposition_pose(pose, reason)
        self.motion_planner.support_total += 1
        self.plan = correction
        print(
            "执行局部纠偏 {}/{}：只调整{}腿，不重新识别或搜索全局路线".format(
                self.motion_planner.local_corrections,
                self.motion_planner.config.correction_limit,
                body.LEG_NAMES[correction.moving_leg],
            )
        )
        return self._publish_active_plan()

    def _activate_next_plan(self, rebuild=False):
        """执行锁定路线的下一段；rebuild只按真实状态重建该局部段。"""
        template, target_world = self.motion_planner.peek_route()
        if template is None:
            return self._fail("对接计划队列为空")
        if not rebuild:
            plan = template
        elif template.kind == "support":
            try:
                plan = self.motion_planner.rebuild_support(
                    template, target_world,
                    self.joints.values, tuple(_body_pose),
                )
            except (RuntimeError, body.IKError) as error:
                pose = self._current_locked_pose()
                if pose is None:
                    return self._fail("局部支撑续接失败：{}".format(error))
                return self._request_reposition_pose(
                    pose, "局部支撑续接失败：{}".format(error)
                )
        else:
            pose = self._current_locked_pose()
            joints = self.joints.values
            if pose is None or joints is None:
                return self._fail("对接局部续接缺少实际位姿或关节状态")
            anchors = body.GraspKinematic().forward_base(joints)
            try:
                plan = body.plan_trajectory(pose, joints, anchors)
            except body.IKError as error:
                return self._activate_local_correction(pose, error)
        self.motion_planner.take_route()
        self.plan = plan
        return self._publish_active_plan()

    def _world_feet(self):
        if self.joints.values is None or _body_pose[0] is None:
            return None
        feet = body.GraspKinematic().forward_base(self.joints.values)
        return feet @ _body_pose[1].T + _body_pose[0]

    def _capture_support_feet(self):
        feet = self._world_feet()
        self._support_world_feet = None if feet is None else feet.copy()

    def _check_support_result(self):
        before, current = self._support_world_feet, self._world_feet()
        if before is None or current is None or self.plan.moving_leg < 0:
            return 0.0
        stance = [leg for leg in range(6) if leg != self.plan.moving_leg]
        slip = np.max(np.linalg.norm(
            current[stance, :2] - before[stance, :2], axis=1
        ))
        print("支撑实际校验：最大支撑脚滑移={:.1f}mm，按真实状态续接下一局部段".format(
            slip * 1000.0
        ))
        return float(slip)

    def continue_route(self, actual_joints=None):
        """支撑落稳后沿锁定高层路线继续，不重新识别或全局搜索。"""
        if actual_joints is not None:
            self.joints.values = np.asarray(
                actual_joints
            ).reshape(6, 3).copy()
        planner = self.motion_planner
        if not planner.route_locked:
            # terrain_plan属于正式路线前的支撑准备，完成后才首次规划。
            return self.start()
        slip = self._check_support_result()
        pose = self._current_locked_pose()
        if pose is None:
            return self._fail("路线续接失败：缺少锁定目标的当前相对位姿")
        decision = planner.support_slip_decision(slip)
        if decision == "abort":
            return self._request_reposition_pose(
                pose,
                "支撑脚滑移{:.1f}mm超过{:.1f}mm安全限值".format(
                    slip * 1000.0,
                    planner.config.support_slip_abort * 1000.0,
                ),
            )
        if decision == "limit":
            return self._request_reposition_pose(
                pose,
                "支撑脚连续滑移且局部纠偏已达{}次上限".format(
                    planner.config.correction_limit
                ),
            )
        if decision == "correct":
            print(
                "支撑滑移触发局部状态修正 {}/{}；高层路线保持不变".format(
                    planner.local_corrections,
                    planner.config.correction_limit,
                )
            )
        return self._activate_next_plan(rebuild=True)

    def _prepare_feedback_descent(self):
        """从真实状态一次生成水平移动与整平合成的连续S曲线。"""
        pose = self._current_locked_pose()
        joints = self.joints.values
        if pose is None or joints is None:
            self._feedback_failure("连续预对准失败：缺少实际位姿或关节状态")
            return False
        try:
            duration = self.motion_planner.prepare_dock_plan(
                self.plan, pose, joints
            )
        except body.IKError as error:
            self._feedback_failure("连续预对准IK不可达：{}".format(error))
            return False
        horizontal, _, tilt = self.measured_error()
        print(
            "开始连续S曲线预对准：水平距离{:.1f}mm，倾斜{:.2f}°，"
            "预计{:.2f}s；中途不停车、不按15mm分段".format(
                horizontal * 1000.0, tilt, duration
            )
        )
        return True

    def measured_error(self):
        if _body_pose[0] is None:
            return None
        difference = _body_pose[0] - PIN_WORLD
        horizontal = np.linalg.norm(difference[:2])
        vertical = abs(difference[2])
        tilt = np.rad2deg(np.arccos(
            np.clip(_body_pose[1][2, 2], -1.0, 1.0)
        ))
        return horizontal, vertical, tilt

    def _feedback_failure(self, reason):
        self.state, self.reason = "failed", reason
        self.status_publisher.publish(reason)

    def monitor_descent(self, plan, elapsed):
        """连续预对准和下降均保留IMU极限保护。"""
        errors = self.measured_error()
        if errors is None:
            return self._feedback_failure("安全监测失败：缺少机身实际位姿")
        reason = self.motion_planner.safety_check(
            plan, elapsed, self.joints.values, errors[2]
        )
        if reason:
            self._feedback_failure(reason)

    def _append_feedback_step(self, plan, elapsed):
        pose, joints = self._current_locked_pose(), self.joints.values
        if pose is None or joints is None:
            return self._feedback_failure("闭环对接失败：缺少实际位姿或关节状态")
        errors = self.measured_error()
        if errors is None:
            return self._feedback_failure("闭环对接失败：缺少实际误差")
        try:
            update = self.motion_planner.advance_dock_plan(
                plan, elapsed, pose, joints, errors
            )
        except body.IKError as error:
            return self._feedback_failure("连续轨迹IK不可达：{}".format(error))
        if update.message:
            print(update.message)
        if update.state in ("success", "failed"):
            self.state, self.reason = update.state, update.reason
            self.status_publisher.publish(update.reason)

    def poll_execution(self):
        """返回一次明确终态；支持段和下降段均受总流程时限约束。"""
        deadline = self.motion_planner.deadline_status(self.state)
        if deadline.warning:
            print(deadline.warning)
        if deadline.failure:
            self._feedback_failure(deadline.failure)
        if self.motion_planner.terminal_reported:
            return None
        if self.state == "success":
            self.motion_planner.terminal_reported = True
            self.finish_trial(True, self.reason)
            return True, self.reason
        if self.state == "failed":
            self.motion_planner.terminal_reported = True
            self.finish_trial(False, self.reason)
            return False, self.reason
        return None

    def descent_has_started(self):
        return self.motion_planner.descent_started

    def start(self):
        if self.motion_planner.route_locked:
            return self.continue_route()
        observed = self.perception.latest_result()
        if observed.valid:
            self._begin_trial(observed)
        if observed.valid and not self._terrain_ready:
            try:
                terrain = self.terrain_plan()
            except (RuntimeError, body.IKError) as error:
                reason = "支撑准备不可达：{}".format(error)
                print(reason)
                return self._request_reposition(observed, reason)
            if terrain is not None:
                return True
        if not observed.valid:
            return self._fail(observed.reason)
        print(
            "视觉补全完成：完整ID={}，推断ID={}，置信度={:.2f}".format(
                observed.decoded_ids, observed.inferred_ids,
                observed.confidence,
            )
        )
        if not observed.decoded_ids:
            print("标签残缺无法解码，Isaac Gym使用小蓝已知位姿补全")
        self.set_trial_phase("路线计算")
        planning_started = time.monotonic()
        try:
            plans = self._precompute_plans(
                observed, max_adjustments=6 - self.adjustments
            )
        except RuntimeError as error:
            reason = "对接不可达：{}".format(error)
            print(reason)
            return self._request_reposition(observed, reason)
        self._observed = observed
        self._pin_pose = observed.lock_from_pin.copy()
        remaining_support = sum(plan.kind == "support" for plan in plans)
        self.motion_planner.support_total = self.adjustments + remaining_support
        self._lock_route(observed, plans)
        planning_time = time.monotonic() - planning_started
        print(
            "高层路线已确定（用时{:.2f}s）：{}次支撑调整 + 闭环对接，"
            "开环预计{:.1f}s；执行期间仅按真实状态续接局部轨迹".format(
                planning_time,
                remaining_support,
                sum(plan.times[-1] for plan in plans)
                * dock_time_scale,
            )
        )
        return self._activate_next_plan()


_back_surface = BackSurface(
    XIAOLAN_ROOT / "meshes/base_link_newxiaolan.STL"
)

def _safe_surface_height(xy, margin=0.012):
    """要求足端中心及周围均位于STL上，返回中心正下方表面。"""
    diagonal = margin / np.sqrt(2.0)
    offsets = np.array((
        (0.0, 0.0), (margin, 0.0), (-margin, 0.0),
        (0.0, margin), (0.0, -margin),
        (diagonal, diagonal), (diagonal, -diagonal),
        (-diagonal, diagonal), (-diagonal, -diagonal),
    ))
    heights = _back_surface.heights(np.asarray(xy) + offsets)
    return float(heights[0]) if np.isfinite(heights).all() else None


def _surface_candidates(start, goals):
    """从越界位置向多个内部目标搜索带12 mm边界余量的落脚点。"""
    start = np.asarray(start, dtype=float)
    seen = set()
    for goal in goals:
        goal = np.asarray(goal, dtype=float)
        # 优先选择标准/内收目标，只有目标不可用时才向原位置回退。
        for phase in np.linspace(1.0, 0.1, 19):
            xy = (1.0 - phase) * start + phase * goal
            key = tuple(np.round(xy, 5))
            if key in seen:
                continue
            seen.add(key)
            height = _safe_surface_height(xy)
            if height is not None:
                yield xy, height


def _leg_surface_candidates(start, base_xy, nominal):
    """只在本腿原扇区和安全外展半径内寻找落脚点。"""
    base_xy = np.asarray(base_xy, dtype=float)
    nominal = np.asarray(nominal, dtype=float)
    direction = nominal - base_xy
    nominal_radius = np.linalg.norm(direction)
    if nominal_radius < 1e-9:
        return
    goals = (
        nominal,
        base_xy + 0.90 * direction,
        base_xy + 0.80 * direction,
    )
    minimum_radius = max(0.130, 0.75 * nominal_radius)
    maximum_radius = min(0.230, 1.10 * nominal_radius)
    cosine_limit = np.cos(np.deg2rad(35.0))
    for xy, height in _surface_candidates(start, goals):
        radial = xy - base_xy
        radius = np.linalg.norm(radial)
        same_sector = (
            np.dot(radial, direction)
            >= cosine_limit * radius * nominal_radius
        )
        if minimum_radius <= radius <= maximum_radius and same_sector:
            yield xy, height


_print = print
_tf_message = _base_tf_message
_set_contact_friction = _base_set_contact_friction
_sample_plan = body.sample_plan
_dock_instance = None
_body_pose = [None, None]
_next_camera_log = 0.0


class TerrainApproachMode(ApproachMode):
    """小蓝背部三足步态：真实落脚高度和支撑脚世界锚点。"""

    TILT_SLOW = np.deg2rad(5.0)
    TILT_STOP = np.deg2rad(8.0)

    def __init__(self, controller):
        super().__init__(controller)
        self.ground_z = controller.foot_init_base[:, 2].copy()
        self.anchors = np.full((6, 3), np.nan)
        self.recovery_world = np.full((6, 3), np.nan)
        self.swing_origin_world = np.full((6, 3), np.nan)
        self.recovery_active = False
        self.planned_position = None
        self.planned_rotation = None
        self._requested_command = np.zeros(4)
        self._phase_command_nominal = np.zeros(4)
        self._tilt_guard_level = 0
        self._tilt_stop_latched = False
        self._workspace_stop_latched = False

    def finish_reset(self):
        super().finish_reset()
        self.ground_z = self.controller.foot_init_base[:, 2].copy()
        self.anchors[:] = np.nan
        self.recovery_world[:] = np.nan
        self.swing_origin_world[:] = np.nan
        self.recovery_active = False
        self.planned_position = self.planned_rotation = None
        self._requested_command[:] = 0.0
        self._phase_command_nominal[:] = 0.0
        self._tilt_guard_level = 0
        self._tilt_stop_latched = False
        self._workspace_stop_latched = False

    @staticmethod
    def _level_rotation(rotation):
        """只保留IMU偏航，横滚和俯仰始终以水平姿态为目标。"""
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
        cosine, sine = np.cos(yaw), np.sin(yaw)
        return np.array((
            (cosine, -sine, 0.0),
            (sine, cosine, 0.0),
            (0.0, 0.0, 1.0),
        ))

    def _sync_plan(self):
        if _body_pose[0] is None:
            return False
        position, rotation = _body_pose
        if self.planned_position is None:
            self.planned_position = position.copy()
            self.planned_rotation = self._level_rotation(rotation)
        else:
            # 换相只用实际XY消除平移积分误差。目标高度和偏航继续沿用
            # 上一相的规划值，不能把接触弹跳造成的Z/姿态误差逐步积累。
            self.planned_position[:2] = position[:2]
        return True

    def _guard_tilt(self, command):
        """倾斜时连续减速；超过8度立即停止平移并完成当前落脚。"""
        if _body_pose[1] is None:
            return command
        tilt = np.arccos(np.clip(
            _body_pose[1][2, 2], -1.0, 1.0
        ))
        planar_input = np.linalg.norm(command[[0, 1, 3]])
        if tilt >= self.TILT_STOP:
            self._tilt_stop_latched = True
        elif (
            self._tilt_stop_latched
            and not self.gait_started
            and not self.transfer_active
            and planar_input < 1e-8
        ):
            self._tilt_stop_latched = False

        if self._tilt_stop_latched:
            scale, level = 0.0, 2
        elif tilt <= self.TILT_SLOW:
            scale, level = 1.0, 0
        else:
            scale = (
                (self.TILT_STOP - tilt)
                / (self.TILT_STOP - self.TILT_SLOW)
            )
            level = 1

        guarded = command.copy()
        guarded[[0, 1, 3]] *= scale
        if self.gait_started:
            self.phase_command[[0, 1, 3]] = (
                self._phase_command_nominal[[0, 1, 3]] * scale
            )
        if level != self._tilt_guard_level:
            if level == 1:
                print("姿态保护：机身倾角{:.1f}°，降低步态速度".format(
                    np.rad2deg(tilt)
                ))
            elif level == 2:
                print("姿态保护：机身倾角{:.1f}°，停止平移并落稳六足".format(
                    np.rad2deg(tilt)
                ))
            elif self._tilt_guard_level:
                print("姿态已恢复，手柄速度限制解除")
            self._tilt_guard_level = level
        return guarded

    def _guard_workspace(self, command):
        """足端到达mode6安全包络后完成当前落脚，并等待摇杆释放。"""
        planar_input = np.linalg.norm(command[[0, 1, 3]])
        if self.controller.consume_workspace_limit():
            if not self._workspace_stop_latched:
                print("足端工作空间保护：停止平移，完成当前摆腿后保持站立")
            self._workspace_stop_latched = True
        elif (
            self._workspace_stop_latched
            and not self.gait_started
            and not self.transfer_active
            and planar_input < 1e-8
        ):
            self._workspace_stop_latched = False
            print("足端已停在安全工作空间内，手柄平移限制解除")

        guarded = command.copy()
        if self._workspace_stop_latched:
            guarded[[0, 1, 3]] = 0.0
            if self.gait_started:
                self.phase_command[[0, 1, 3]] = 0.0
        return guarded

    def _advance_plan(self, command):
        if self.planned_position is None and not self._sync_plan():
            return
        command = np.asarray(command)
        # 摇杆平移只使用偏航方向，不能把横滚/俯仰混入水平速度。
        yaw = np.arctan2(
            self.planned_rotation[1, 0], self.planned_rotation[0, 0]
        )
        cosine, sine = np.cos(yaw), np.sin(yaw)
        yaw_rotation = np.array((
            (cosine, -sine, 0.0),
            (sine, cosine, 0.0),
            (0.0, 0.0, 1.0),
        ))
        velocity = yaw_rotation @ np.r_[command[:2], 0.0]
        self.planned_position[:2] += velocity[:2] * self.dt
        # Z高度已经由ApproachMode.body_height_offset限幅积分；这里若再次
        # 积分会让机身持续升高，最终把支撑腿推到关节极限。
        angle = command[3] * self.dt
        cosine, sine = np.cos(angle), np.sin(angle)
        yaw_step = np.array((
            (cosine, -sine, 0.0),
            (sine, cosine, 0.0),
            (0.0, 0.0, 1.0),
        ))
        self.planned_rotation = yaw_step @ self.planned_rotation

    def _capture_anchors(self, legs, desired=False):
        if self.planned_position is None and not self._sync_plan():
            return
        feet = (
            self.controller.foot_desired_base
            if desired else self.controller.actual_feet_base
        )
        position, rotation = (
            (self.planned_position, self.planned_rotation)
            if desired else _body_pose
        )
        self.anchors[legs] = feet[legs] @ rotation.T + position

    def _apply_anchors(self, legs):
        if self.planned_position is None or not len(legs):
            return
        valid = np.isfinite(self.anchors[legs]).all(axis=1)
        legs = legs[valid]
        if not len(legs):
            return
        candidate = self.controller.foot_desired_base.copy()
        candidate[legs] = (
            self.anchors[legs] - self.planned_position
        ) @ self.planned_rotation
        self.controller._commit_workspace_candidate(candidate)

    def _refresh_swing_ground(self):
        if (
            _body_pose[0] is None
            or not self.gait_started
            or self.transfer_active
        ):
            return
        legs = np.where(~self.gaits)[0]
        targets = self.swing_target_base[legs].copy()
        if self.planned_position is None and not self._sync_plan():
            return
        position, rotation = self.planned_position, self.planned_rotation
        world = targets @ rotation.T + position
        heights = _back_surface.heights(world[:, :2])
        valid = np.isfinite(heights)
        recovering = np.array([
            np.isfinite(self.recovery_world[leg]).all()
            for leg in legs
        ])
        if recovering.any():
            world[recovering] = self.recovery_world[legs[recovering]]
            heights[recovering] = (
                world[recovering, 2] - FOOT_RADIUS
            )
            valid[recovering] = True
        if not valid.all():
            recovered = []
            for local in np.where(~valid)[0]:
                leg = legs[local]
                nominal = (
                    position[:2]
                    + self.controller.foot_init_base[leg, :2]
                    @ rotation[:2, :2].T
                )
                candidate = next(_leg_surface_candidates(
                    world[local, :2], position[:2], nominal
                ), None)
                if candidate is None:
                    origin = self.swing_origin_world[leg]
                    if np.isfinite(origin).all():
                        height = _back_surface.heights(origin[None, :2])[0]
                        if np.isfinite(height):
                            candidate = origin[:2].copy(), float(height)
                if candidate is None:
                    continue
                xy, height = candidate
                self.recovery_world[leg] = (
                    xy[0], xy[1], height + FOOT_RADIUS
                )
                world[local] = self.recovery_world[leg]
                heights[local], valid[local] = height, True
                targets[local] = (world[local] - position) @ rotation
                self.swing_target_base[leg, :2] = targets[local, :2]
                recovered.append(body.LEG_NAMES[leg])
            if recovered:
                self.phase_command[[0, 1, 3]] = 0.0
                self.recovery_active = True
                print("越界恢复开始：{}腿返回各自安全扇区".format(
                    ",".join(recovered)
                ))
            if not valid.all():
                print("步态保护：未找到可达的STL内部安全落脚点")
        if not valid.any():
            return
        world[valid, 2] = heights[valid] + FOOT_RADIUS
        targets[valid] = (world[valid] - position) @ rotation
        valid_legs = legs[valid]
        self.swing_target_base[valid_legs, 2] = targets[valid, 2]
        self.ground_z[valid_legs] = (
            targets[valid, 2] + self.body_height_offset
        )

    def _begin_step(self, command):
        super()._begin_step(command)
        self._phase_command_nominal = self._requested_command.copy()
        self._sync_plan()
        stance = np.where(self.gaits)[0]
        swing = np.where(~self.gaits)[0]
        self._capture_anchors(stance)
        self.swing_origin_world[:] = np.nan
        position, rotation = _body_pose
        self.swing_origin_world[swing] = (
            self.controller.actual_feet_base[swing] @ rotation.T + position
        )
        # 摆动轨迹从真实足端世界位置起步，但在水平目标机身坐标系中
        # 插值，避免机身已有小倾角时制造额外的虚假抬腿高度。
        self.swing_start_base[swing] = (
            self.swing_origin_world[swing] - self.planned_position
        ) @ self.planned_rotation
        self.swing_start_velocity_xy[swing] = (
            self.foot_velocity_xy[swing]
        )
        self._refresh_swing_ground()

    def _update_gait(self, command):
        command = np.asarray(command).copy()
        if self.recovery_active:
            command[[0, 1, 3]] = 0.0
        self._requested_command = command.copy()
        command = self._guard_workspace(command)
        command = self._guard_tilt(command)
        if self.recovery_active:
            self.phase_command[[0, 1, 3]] = 0.0
        was_transfer = self.transfer_active
        old_stance = self.gaits.copy()
        if self.gait_started:
            # 一个三足步态周期内连续积分；只在_begin_step换相时同步RTK/IMU。
            self._advance_plan(command)
        self._refresh_swing_ground()
        original_z = self.controller.foot_init_base[:, 2].copy()
        try:
            self.controller.foot_init_base[:, 2] = self.ground_z
            super()._update_gait(command)
        finally:
            self.controller.foot_init_base[:, 2] = original_z
        if (
            self.recovery_active
            and not self.gait_started
            and not self.transfer_active
        ):
            recovered = [
                body.LEG_NAMES[leg] for leg in range(6)
                if np.isfinite(self.recovery_world[leg]).all()
            ]
            self.recovery_world[:] = np.nan
            self.swing_origin_world[:] = np.nan
            self.recovery_active = False
            print("越界恢复完成：{}腿已落稳，手柄控制自动恢复".format(
                ",".join(recovered)
            ))
        if self.transfer_active and not was_transfer:
            self._capture_anchors(np.where(~old_stance)[0], True)
        if self.transfer_active:
            self._apply_anchors(np.arange(6))
        elif self.gait_started:
            self._apply_anchors(np.where(self.gaits)[0])
        else:
            # transfer结束和gait_started清零发生在同一控制周期；退出前
            # 必须最后提交一次六足世界锚点，否则会把过渡相的ground_z
            # 留作静态目标，造成部分足端悬空和机身倾斜。
            if was_transfer:
                self._apply_anchors(np.arange(6))
            self.planned_position = self.planned_rotation = None


class TerrainGraspController(GraspController):
    DOCK = "dock"
    DOCK_REPOSITION_MAX_ATTEMPTS = 8
    DOCK_REPOSITION_STEP = 0.020
    DOCK_REPOSITION_SPEED = 0.025
    DOCK_REPOSITION_VERTICAL_SPEED = 0.010
    DOCK_REPOSITION_TOLERANCE = 0.004
    DOCK_REPOSITION_HEIGHT_TOLERANCE = 0.003
    DOCK_REPOSITION_TIMEOUT = 6.0
    DOCK_REPOSITION_SEARCH_STEP = 0.010
    DOCK_REPOSITION_SEARCH_RADIUS = 0.040
    DOCK_REPOSITION_SEARCH_TIMEOUT = 4.0
    DOCK_REPOSITION_MAX_CANDIDATES = 64
    DOCK_REPOSITION_CANDIDATE_ADJUSTMENTS = 6
    DOCK_REPOSITION_MIN_JOINT_MARGIN = np.deg2rad(0.5)
    DOCK_REPOSITION_DIVERGENCE = 0.012
    DOCK_REPOSITION_TRAVEL_MARGIN = 0.020
    DOCK_SETTLE_POSITION_STEP = 0.0005
    DOCK_SETTLE_ANGLE_STEP = np.deg2rad(0.25)
    DOCK_SETTLE_JOINT_STEP = np.deg2rad(0.30)
    DOCK_SETTLE_SAMPLES = 8
    DOCK_ENTRY_MAX_TILT = np.deg2rad(3.0)
    DOCK_ENTRY_MIN_JOINT_MARGIN = np.deg2rad(5.0)
    DOCK_SUPPORT_SETTLE_TIMEOUT = 2.0
    DOCK_SUPPORT_NORMALIZE_TOLERANCE = 0.008
    TERRAIN_WORKSPACE_XY_RADIUS = 0.055
    TERRAIN_WORKSPACE_Z_RADIUS = 0.030
    # 对接前自主小步调整需要跨过背板边缘；只在该阶段额外抬腿，
    # 停稳后立即恢复普通平地步态高度，再进入六足固定对接。
    DOCK_REPOSITION_EXTRA_STEP_HEIGHT = 0.015

    def __init__(self, dt):
        # super().__init__会调用_workspace_feasible，字段必须提前建立。
        self._terrain_workspace_center = None
        self._workspace_limit_pending = False
        super().__init__(dt)
        # mode6运行在真实背板表面，足端候选始终限制在稳定初始足端周围，
        # 避免长距离手柄移动后关节目标持续顶住机械限位。
        self.enable_workspace_check = True
        self.actual_feet_base = self.foot_init_base.copy()
        self.actual_joints = self.q_init.copy()
        self.approach_mode = TerrainApproachMode(self)
        self._dock_reposition_active = False
        self._dock_reposition_stopping = False
        self._dock_reposition_ready = False
        self._dock_reposition_target = None
        self._dock_reposition_height_target = None
        self._dock_reposition_origin = None
        self._dock_reposition_best_distance = np.inf
        self._dock_reposition_started = 0.0
        self._dock_reposition_attempts = 0
        self._dock_reposition_normal_step_height = None
        self._dock_reposition_normalization = False
        self._dock_nominal_body_clearance = None
        self._entry_settle_samples = 0
        self._entry_settle_position = None
        self._entry_settle_rotation = None
        self._entry_settle_joints = None
        self._entry_tilt_rejected = False
        self._support_settle_samples = 0
        self._support_settle_position = None
        self._support_settle_rotation = None
        self._support_settle_joints = None
        self._support_settle_target = None
        self._support_settle_started = None
        self._dock_entry_hold_active = False
        self.dock_mode = SimDockMode(self)

    def configure_terrain_workspace(self, stable_feet_base):
        """用仿真已求解的稳定接触站姿建立mode6专用安全包络。"""
        self._terrain_workspace_center = np.asarray(
            stable_feet_base, dtype=float
        ).reshape(6, 3).copy()

    def _workspace_feasible(self, foot_positions_base):
        center = getattr(self, "_terrain_workspace_center", None)
        if center is None:
            return super()._workspace_feasible(foot_positions_base)
        feet = np.asarray(
            foot_positions_base, dtype=float
        ).reshape(6, 3)
        delta = feet - center
        return (
            np.linalg.norm(delta[:, :2], axis=1)
            <= self.TERRAIN_WORKSPACE_XY_RADIUS + 1e-9
        ) & (
            np.abs(delta[:, 2])
            <= self.TERRAIN_WORKSPACE_Z_RADIUS + 1e-9
        )

    def _project_workspace(self, foot_positions_base):
        center = getattr(self, "_terrain_workspace_center", None)
        if center is None:
            return super()._project_workspace(foot_positions_base)
        feet = np.asarray(
            foot_positions_base, dtype=float
        ).reshape(6, 3).copy()
        delta = feet - center
        radius = np.linalg.norm(delta[:, :2], axis=1)
        outside = radius > self.TERRAIN_WORKSPACE_XY_RADIUS
        delta[outside, :2] *= (
            self.TERRAIN_WORKSPACE_XY_RADIUS / radius[outside]
        )[:, None]
        delta[:, 2] = np.clip(
            delta[:, 2],
            -self.TERRAIN_WORKSPACE_Z_RADIUS,
            self.TERRAIN_WORKSPACE_Z_RADIUS,
        )
        self._workspace_limit_pending = True
        return center + delta

    def consume_workspace_limit(self):
        limited, self._workspace_limit_pending = (
            self._workspace_limit_pending, False
        )
        return limited

    def set_mode(self, mode):
        """在现有APPROACH/CLIMB控制器外增加仿真DOCK适配。"""
        if mode == self.DOCK:
            if getattr(self, "mode", None) != self.DOCK:
                self.mode = mode
                self.dock_mode.enter(self.foot_desired_base)
            return
        if (
            getattr(self, "mode", None) == self.DOCK
            and hasattr(self, "dock_mode")
        ):
            self.dock_mode.exit()
        super().set_mode(mode)

    @staticmethod
    def _candidate_joint_margin(plan):
        """返回整条轨迹距离关节上下限的最小余量。"""
        lower = body.joint_lower[None, ...]
        upper = body.joint_upper[None, ...]
        return float(np.min(np.minimum(
            plan.joints - lower, upper - plan.joints
        )))

    def _predict_reposition_stance(self, request, displacement_local):
        """验证候选位置的地面、足端碰撞和标准站姿逆解。"""
        position, rotation = _body_pose
        displacement_local = np.asarray(
            displacement_local, dtype=float
        ).reshape(2)
        displacement_world = (
            rotation[:2, :2] @ displacement_local
        ).reshape(2)
        candidate_position = position.copy()
        candidate_position[:2] += displacement_world

        # 预测移动完成后六足回到当前地形站姿扇区，并逐足投影到真实STL。
        nominal = body.GraspKinematic().forward_base(Q_STAND)
        world_feet = nominal @ rotation.T + candidate_position
        surface = np.array([
            np.nan if height is None else float(height)
            for height in (
                _safe_surface_height(xy, margin=0.008)
                for xy in world_feet[:, :2]
            )
        ], dtype=float)
        if not np.isfinite(surface).all():
            raise RuntimeError("候选位置存在不安全落脚点")
        # 自主恢复必须同时回到启动时的真实稳定离地高度。Q_STAND只是
        # 关节参考角，不代表崎岖背部上的机身高度，不能直接用它反推Z。
        clearance = self._dock_nominal_body_clearance
        if clearance is None:
            clearance = float(self.base_height_at_stand)
        candidate_position[2] = float(np.mean(surface) + clearance)
        world_feet[:, 2] = surface + FOOT_RADIUS - 0.0015
        candidate_feet = (
            world_feet - candidate_position
        ) @ rotation
        if not self._foot_collision_free(candidate_feet).all():
            raise RuntimeError("候选标准站姿存在足端碰撞")

        seed = request.get("joints")
        if seed is None:
            seed = self.q_init
        candidate_joints, _ = body.solve_joints(
            body.GraspKinematic(), candidate_feet,
            np.asarray(seed).reshape(6, 3),
        )

        return {
            "local": displacement_local,
            "world": displacement_world,
            "target": candidate_position[:2],
            "height": float(candidate_position[2]),
            "joints": candidate_joints,
            "feet": candidate_feet,
        }

    def _predict_reposition_candidate(self, request, displacement_local):
        """验证候选站姿以及从该站姿开始的完整对接轨迹。"""
        position, rotation = _body_pose
        candidate = self._predict_reposition_stance(
            request, displacement_local
        )
        candidate_position = position.copy()
        candidate_position[:2] = candidate["target"]
        candidate_position[2] = candidate["height"]
        candidate_joints = candidate["joints"]
        candidate_feet = candidate["feet"]

        world_from_pin = (
            body.transform(position, rotation)
            @ np.asarray(request["lock_from_pin"])
        )
        candidate_lock_from_pin = (
            np.linalg.inv(body.transform(
                candidate_position, rotation
            )) @ world_from_pin
        )
        route, joints, anchors = [], candidate_joints, candidate_feet
        for adjustment in range(
            self.DOCK_REPOSITION_CANDIDATE_ADJUSTMENTS + 1
        ):
            try:
                plan = body.plan_trajectory(
                    candidate_lock_from_pin, joints, anchors
                )
            except body.IKError as error:
                if adjustment >= self.DOCK_REPOSITION_CANDIDATE_ADJUSTMENTS:
                    raise RuntimeError(
                        "候选经过{}次调足仍不可达：{}".format(
                            self.DOCK_REPOSITION_CANDIDATE_ADJUSTMENTS, error
                        )
                    )
                try:
                    support = body.support_plan(
                        candidate_lock_from_pin, joints, anchors, error,
                        lift=0.015,
                        place_foot=self.dock_mode.place_support_foot,
                    )
                except body.IKError as support_error:
                    raise RuntimeError(
                        "候选支撑调整不可达：{}".format(support_error)
                    )
                route.append(support)
                joints, anchors = support.joints[-1], support.anchors
            else:
                route.append(plan)
                break
        margins = [self._candidate_joint_margin(item) for item in route]
        margin = min(margins)
        if margin < self.DOCK_REPOSITION_MIN_JOINT_MARGIN:
            raise RuntimeError("候选对接轨迹关节余量不足")
        return {
            "local": candidate["local"],
            "world": candidate["world"],
            "target": candidate["target"],
            "height": candidate["height"],
            "margin": margin,
            "horizontal": float(np.linalg.norm(
                candidate_lock_from_pin[:2, 3]
            )),
            "support_count": sum(
                item.kind == "support" for item in route
            ),
            "route_duration": sum(float(item.times[-1]) for item in route),
        }

    def _search_dock_reposition(self, request):
        """二维枚举当前位置附近的可达站姿，选择短且关节余量大的目标。"""
        step = self.DOCK_REPOSITION_SEARCH_STEP
        radius = self.DOCK_REPOSITION_SEARCH_RADIUS
        values = np.arange(-radius, radius + 0.5 * step, step)
        candidates = [
            np.array((right, forward))
            for right in values for forward in values
            if (
                np.hypot(right, forward) >= 0.5 * step
                and np.hypot(right, forward) <= radius + 1e-9
            )
        ]
        # 先检查最接近插销的候选。旧逻辑把移动距离放在首位，会在
        # 机器人离插销较远时耗尽候选上限，只检查当前位置附近的小步。
        pin_xy = np.asarray(
            request["lock_from_pin"][:2, 3], dtype=float
        )
        candidates.sort(key=lambda offset: (
            np.linalg.norm(pin_xy - offset),
            np.linalg.norm(offset),
        ))

        feasible, closest_failure = [], None
        rejected_surface = rejected_geometry = rejected_margin = 0
        search_started = time.monotonic()
        evaluated = 0
        for offset in candidates:
            if (
                evaluated >= self.DOCK_REPOSITION_MAX_CANDIDATES
                or time.monotonic() - search_started
                >= self.DOCK_REPOSITION_SEARCH_TIMEOUT
            ):
                break
            evaluated += 1
            try:
                result = self._predict_reposition_candidate(request, offset)
            except body.IKError as error:
                failure = (float(error.residual), float(
                    np.linalg.norm(offset)
                ), offset.copy(), str(error))
                if closest_failure is None or failure[:2] < closest_failure[:2]:
                    closest_failure = failure
                continue
            except RuntimeError as error:
                message = str(error)
                if "落脚点" in message:
                    rejected_surface += 1
                elif "余量" in message:
                    rejected_margin += 1
                else:
                    rejected_geometry += 1
                continue
            travel = float(np.linalg.norm(offset))
            # 在移动距离相近时，奖励完整轨迹的关节余量和更小水平误差。
            result["score"] = (
                travel + 0.20 * result["horizontal"]
                + 0.008 * result["support_count"]
                + 0.0005 * result["route_duration"]
                - 0.015 * min(0.35, max(0.0, result["margin"]))
            )
            feasible.append(result)
            # 获得三个完整可达候选后即可评分选择，不再遍历整张网格。
            if len(feasible) >= 3:
                break

        if feasible:
            selected = min(feasible, key=lambda item: item["score"])
            print(
                "二维可达位姿搜索：验证{}个候选，{}个可完整对接；"
                "选择右/前={} mm，预测调足{}次，关节余量={:.1f}°".format(
                    evaluated, len(feasible),
                    np.round(selected["local"] * 1000.0, 1).tolist(),
                    selected["support_count"],
                    np.rad2deg(selected["margin"]),
                )
            )
            return selected

        # 没有一步完成对接的候选时，只允许执行经过地形、碰撞和站姿
        # IK验证且确实减小插销水平距离的中间站姿。停稳后根据真实足端
        # 再规划，不能再执行旧版“残差最小但方向可能相反”的候选。
        safe = []
        current_error = float(np.linalg.norm(pin_xy))
        for offset in candidates:
            try:
                stance = self._predict_reposition_stance(request, offset)
            except (RuntimeError, body.IKError):
                continue
            remaining = float(np.linalg.norm(pin_xy - offset))
            if remaining + 0.002 < current_error:
                stance["remaining"] = remaining
                safe.append(stance)
        if not safe:
            print(
                "二维搜索无可用候选：不安全落脚{}个，几何拒绝{}个，"
                "关节余量不足{}个；已检查{}/{}个完整轨迹候选，且没有"
                "能安全缩短对接距离的中间站姿，用时{:.2f}s".format(
                    rejected_surface, rejected_geometry, rejected_margin,
                    evaluated, len(candidates), time.monotonic() - search_started,
                )
            )
            return None
        selected = min(safe, key=lambda item: (
            item["remaining"], np.linalg.norm(item["local"])
        ))
        detail = ""
        residual = np.inf
        if closest_failure is not None:
            residual, _, _, reason = closest_failure
            detail = "；最近完整轨迹残差{:.1f} mm：{}".format(
                residual * 1000.0, reason
            )
        print(
            "二维搜索暂无一步直达解；选择已验证的安全中间站姿右/前={} mm，"
            "预计水平剩余{:.1f} mm{}".format(
                np.round(selected["local"] * 1000.0, 1).tolist(),
                selected["remaining"] * 1000.0, detail,
            )
        )
        selected["margin"] = -residual
        selected["horizontal"] = selected["remaining"]
        return selected

    def _restore_dock_reposition_step_height(self):
        normal = self._dock_reposition_normal_step_height
        if normal is None:
            return
        self.approach_mode.step_height = normal
        self._dock_reposition_normal_step_height = None

    def _reset_entry_settle_monitor(self):
        self._entry_settle_samples = 0
        self._entry_settle_position = None
        self._entry_settle_rotation = None
        self._entry_settle_joints = None

    def _entry_body_and_joints_stable(self):
        """入口只使用自己的采样状态判断机身和关节是否稳定。"""
        if _body_pose[0] is None:
            self._reset_entry_settle_monitor()
            return False
        position, rotation = _body_pose
        joints = self.actual_joints
        tilt = np.arccos(np.clip(
            rotation[2, 2], -1.0, 1.0
        ))
        if tilt > self.DOCK_ENTRY_MAX_TILT:
            self._entry_settle_samples = 0
            if not self._entry_tilt_rejected:
                print("禁止进入对接：机身倾角{:.1f}°，请先恢复稳定站姿".format(
                    np.rad2deg(tilt)
                ))
                self._entry_tilt_rejected = True
            return False
        if self._entry_tilt_rejected:
            print("机身姿态已恢复，继续确认对接前稳定状态")
            self._entry_tilt_rejected = False
        if self._entry_settle_position is None:
            stable = False
        else:
            position_step = np.linalg.norm(
                position - self._entry_settle_position
            )
            relative = rotation @ self._entry_settle_rotation.T
            angle_step = np.arccos(np.clip(
                (np.trace(relative) - 1.0) / 2.0,
                -1.0, 1.0,
            ))
            joint_step = np.max(np.abs(
                joints - self._entry_settle_joints
            ))
            stable = bool(
                position_step <= self.DOCK_SETTLE_POSITION_STEP
                and angle_step <= self.DOCK_SETTLE_ANGLE_STEP
                and joint_step <= self.DOCK_SETTLE_JOINT_STEP
            )
        self._entry_settle_position = position.copy()
        self._entry_settle_rotation = rotation.copy()
        self._entry_settle_joints = joints.copy()
        self._entry_settle_samples = (
            self._entry_settle_samples + 1 if stable else 0
        )
        return self._entry_settle_samples >= self.DOCK_SETTLE_SAMPLES

    def _reset_support_settle_monitor(self):
        self._support_settle_samples = 0
        self._support_settle_position = None
        self._support_settle_rotation = None
        self._support_settle_joints = None
        self._support_settle_target = None
        self._support_settle_started = None

    def reset_dock_reposition(self):
        """开始一次新的手动对接试验时清空自动搜索计数。"""
        getattr(self.dock_mode, "reset_trial", lambda: None)()
        self._restore_dock_reposition_step_height()
        self._dock_reposition_active = False
        self._dock_reposition_stopping = False
        self._dock_reposition_ready = False
        self._dock_reposition_target = None
        self._dock_reposition_height_target = None
        self._dock_reposition_origin = None
        self._dock_reposition_best_distance = np.inf
        self._dock_reposition_attempts = 0
        self._dock_reposition_normalization = False
        self._dock_entry_hold_active = False
        self._reset_entry_settle_monitor()
        self._reset_support_settle_monitor()

    def cancel_dock_reposition(self):
        getattr(self.dock_mode, "cancel_trial", lambda: None)()
        self._restore_dock_reposition_step_height()
        self._dock_reposition_active = False
        self._dock_reposition_stopping = False
        self._dock_reposition_ready = False
        self._dock_reposition_target = None
        self._dock_reposition_height_target = None
        self._dock_reposition_origin = None
        self._dock_reposition_best_distance = np.inf
        self._dock_reposition_normalization = False
        self._dock_entry_hold_active = False
        self._reset_entry_settle_monitor()
        self._reset_support_settle_monitor()

    def start_dock_reposition(self, request):
        """搜索并发起一次二维小步态，停稳后重新视觉计算。"""
        if request is None or _body_pose[0] is None:
            return False
        getattr(
            self.dock_mode, "set_trial_phase", lambda _: None
        )("自主重定位搜索与移动")
        self._dock_entry_hold_active = False
        self._reset_entry_settle_monitor()
        self._reset_support_settle_monitor()
        if self._dock_reposition_attempts >= self.DOCK_REPOSITION_MAX_ATTEMPTS:
            print(
                "自主可达位姿搜索已达{}次上限，停止并等待手柄".format(
                    self.DOCK_REPOSITION_MAX_ATTEMPTS
                )
            )
            return False

        selected = self._search_dock_reposition(request)
        if selected is None:
            print("二维可达位姿搜索失败：附近没有安全站姿，等待手柄调整")
            return False
        displacement = selected["world"]

        normal_height = self._dock_reposition_normal_step_height
        if normal_height is None:
            normal_height = float(self.approach_mode.step_height)
            self._dock_reposition_normal_step_height = normal_height
        raised_height = normal_height + self.DOCK_REPOSITION_EXTRA_STEP_HEIGHT
        self.approach_mode.step_height = raised_height

        self._dock_reposition_attempts += 1
        self._dock_reposition_target = selected["target"]
        self._dock_reposition_height_target = selected["height"]
        self._dock_reposition_origin = _body_pose[0][:2].copy()
        self._dock_reposition_best_distance = float(np.linalg.norm(
            self._dock_reposition_target - self._dock_reposition_origin
        ))
        self._dock_reposition_active = True
        self._dock_reposition_stopping = False
        self._dock_reposition_ready = False
        self._dock_reposition_started = time.monotonic()
        print(
            "自主小步移动 {}/{}：世界平面位移={} mm，抬腿高度 {:.0f}→{:.0f} mm，"
            "目标机身高度{:.1f} mm，停稳后自动重试对接".format(
                self._dock_reposition_attempts,
                self.DOCK_REPOSITION_MAX_ATTEMPTS,
                np.round(displacement * 1000.0, 1).tolist(),
                normal_height * 1000.0,
                raised_height * 1000.0,
                self._dock_reposition_height_target * 1000.0,
            )
        )
        return True

    def dock_reposition_command(self):
        """返回自主小步态指令；None表示本周期不接管手柄。"""
        if not self._dock_reposition_active:
            return None
        if self.reset_active:
            return np.zeros(4)
        if _body_pose[0] is None:
            self._dock_reposition_stopping = True

        if not self._dock_reposition_stopping:
            position, rotation = _body_pose
            remaining = self._dock_reposition_target - position[:2]
            distance = np.linalg.norm(remaining)
            height_error = self._dock_reposition_height_target - position[2]
            self._dock_reposition_best_distance = min(
                self._dock_reposition_best_distance, float(distance)
            )
            planned_travel = np.linalg.norm(
                self._dock_reposition_target - self._dock_reposition_origin
            )
            actual_travel = np.linalg.norm(
                position[:2] - self._dock_reposition_origin
            )
            diverged = bool(
                distance
                > self._dock_reposition_best_distance
                + self.DOCK_REPOSITION_DIVERGENCE
                or actual_travel
                > planned_travel + self.DOCK_REPOSITION_TRAVEL_MARGIN
            )
            timed_out = (
                time.monotonic() - self._dock_reposition_started
                >= self.DOCK_REPOSITION_TIMEOUT
            )
            position_reached = bool(
                distance <= self.DOCK_REPOSITION_TOLERANCE
                and abs(height_error) <= self.DOCK_REPOSITION_HEIGHT_TOLERANCE
            )
            if position_reached or timed_out or diverged:
                self._dock_reposition_stopping = True
                if diverged:
                    print(
                        "自主小步实际运动偏离目标，提前停步并按真实位姿重规划："
                        "当前距离{:.1f} mm，已移动{:.1f} mm".format(
                            distance * 1000.0, actual_travel * 1000.0
                        )
                    )
                elif timed_out:
                    print("自主小步移动超时，先完成当前摆腿并停稳")
            else:
                velocity_world = (
                    np.zeros(2) if distance <= self.DOCK_REPOSITION_TOLERANCE
                    else self.DOCK_REPOSITION_SPEED * remaining / distance
                )
                yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
                cosine, sine = np.cos(yaw), np.sin(yaw)
                world_from_heading = np.array((
                    (cosine, -sine), (sine, cosine)
                ))
                velocity_body = world_from_heading.T @ velocity_world
                return np.array((
                    velocity_body[0], velocity_body[1],
                    np.clip(
                        height_error,
                        -self.DOCK_REPOSITION_VERTICAL_SPEED,
                        self.DOCK_REPOSITION_VERTICAL_SPEED,
                    ),
                    0.0,
                ))

        if self.approach_mode.gait_started or self.approach_mode.transfer_active:
            return np.zeros(4)
        self._dock_reposition_active = False
        self._dock_reposition_stopping = False
        self._dock_reposition_ready = True
        # 整机小步后先逐腿恢复安全支撑，再按真实关节状态计算对接。
        # terrain_plan会在直达不可达时使用安全中间落脚点，不再抛出
        # 未处理的IKError。
        self._dock_reposition_normalization = True
        self._restore_dock_reposition_step_height()
        print("自主小步移动已停稳，重新识别并计算对接可达性")
        return None

    def consume_dock_reposition_ready(self):
        ready, self._dock_reposition_ready = self._dock_reposition_ready, False
        return ready

    def consume_dock_reposition_normalization(self):
        required = self._dock_reposition_normalization
        self._dock_reposition_normalization = False
        return required

    def dock_entry_ready(self):
        """步态结束后锁定一次真实足端，保持目标并等待机身稳定。"""
        if (
            self.reset_active
            or self.approach_mode.gait_started
            or self.approach_mode.transfer_active
        ):
            self._dock_entry_hold_active = False
            self._reset_entry_settle_monitor()
            return False

        # 规划足端是步态命令，不是接触传感器读数。步态停止后只同步一次
        # 当前真实关节/足端，随后保持该目标；不能因几毫米静态误差反复
        # reset_to_stand，否则会不断改变六腿目标并激发接触颤动。
        if not self._dock_entry_hold_active:
            joints = self.actual_joints.copy()
            feet = self.actual_feet_base.copy()
            workspace_ok = self._workspace_feasible(feet)
            joint_margin = np.minimum(
                joints - body.joint_lower,
                body.joint_upper - joints,
            )
            unsafe_legs = np.where(
                (~workspace_ok)
                | (np.min(joint_margin, axis=1)
                   < self.DOCK_ENTRY_MIN_JOINT_MARGIN)
            )[0]
            if len(unsafe_legs):
                # SimDockMode.terrain_plan已有逐腿抬起、内收、落稳逻辑。
                # 这里只标记复用，禁止同时拖动六足回到Q_STAND。
                self._dock_reposition_normalization = True
                print(
                    "对接入口安全检查：{}腿越界或关节余量不足，"
                    "先逐腿恢复标准安全支撑".format(
                        ",".join(body.LEG_NAMES[int(leg)] for leg in unsafe_legs)
                    )
                )
            self.q_init = joints.copy()
            self.q_des = joints.copy()
            self.foot_init_base = feet.copy()
            self.foot_desired_base = feet.copy()
            self.foot_init_hip = self.kinematic.base_to_hip(feet)
            self.foot_desired_hip = self.foot_init_hip.copy()
            self.foot_current_hip = self.kinematic.forward(joints)
            self.base_height_at_stand = (
                FOOT_RADIUS - np.mean(feet[:, 2])
            )
            self.approach_mode.finish_reset()
            self._dock_entry_hold_active = True
            self._reset_entry_settle_monitor()
            print("步态已结束：锁定当前真实足端，保持目标并等待机身稳定")
        return self._entry_body_and_joints_stable()

    def dock_support_plan_settled(self, target_joints):
        """支撑末端按真实状态停稳；不要求穿过碰撞面追上理想目标。"""
        target = np.asarray(target_joints).reshape(6, 3)
        if (
            self._support_settle_target is None
            or not np.allclose(
                target, self._support_settle_target, atol=1e-9, rtol=0.0
            )
        ):
            self._reset_support_settle_monitor()
            self._support_settle_target = target.copy()
            self._support_settle_started = time.monotonic()

        position, rotation = _body_pose
        joints = self.actual_joints
        if position is None:
            stable = False
        elif self._support_settle_position is None:
            stable = False
        else:
            position_step = np.linalg.norm(
                position - self._support_settle_position
            )
            relative = rotation @ self._support_settle_rotation.T
            angle_step = np.arccos(np.clip(
                (np.trace(relative) - 1.0) / 2.0,
                -1.0, 1.0,
            ))
            joint_step = np.max(np.abs(
                joints - self._support_settle_joints
            ))
            stable = bool(
                position_step <= self.DOCK_SETTLE_POSITION_STEP
                and angle_step <= self.DOCK_SETTLE_ANGLE_STEP
                and joint_step <= self.DOCK_SETTLE_JOINT_STEP
            )

        if position is not None:
            self._support_settle_position = position.copy()
            self._support_settle_rotation = rotation.copy()
        self._support_settle_joints = joints.copy()
        self._support_settle_samples = (
            self._support_settle_samples + 1 if stable else 0
        )
        elapsed = time.monotonic() - self._support_settle_started
        settled = self._support_settle_samples >= self.DOCK_SETTLE_SAMPLES
        timed_out = elapsed >= self.DOCK_SUPPORT_SETTLE_TIMEOUT
        if not settled and not timed_out:
            return False

        ideal_error = np.rad2deg(np.max(
            np.abs(joints - target)
        ))
        if timed_out and not settled:
            print(
                "支撑末端等待达到{:.1f}s上限，接受当前真实状态继续重规划；"
                "理想关节最大偏差={:.2f}°".format(
                    self.DOCK_SUPPORT_SETTLE_TIMEOUT, ideal_error
                )
            )
        else:
            print(
                "支撑末端真实状态已稳定，接受实际关节继续重规划；"
                "理想关节最大偏差={:.2f}°".format(ideal_error)
            )
        self._reset_support_settle_monitor()
        return True

    def observe_actual_state(self, q_current):
        """无论当前处于步态或DOCK轨迹，都刷新真实关节和足端状态。"""
        # 攀爬终态交接在控制循环开始前写入稳定站姿高度；只记录第一次值，
        # 避免手柄升降后的姿态覆盖自主恢复基准。
        if self._dock_nominal_body_clearance is None:
            self._dock_nominal_body_clearance = float(
                self.base_height_at_stand
            )
        self.actual_joints = np.asarray(
            q_current
        ).reshape(6, 3).copy()
        self.actual_feet_base = self.kinematic.forward_base(
            self.actual_joints
        )

    def reset_to_stand(self, q_current):
        """按当前位置地形重建站姿，不返回启动位置对应的旧关节角。"""
        joints = np.asarray(q_current).reshape(6, 3)
        feet = self.kinematic.forward_base(joints)
        target_joints = joints.copy()
        if _body_pose[0] is not None:
            position, rotation = _body_pose
            world = feet @ rotation.T + position
            heights = _back_surface.heights(world[:, :2])
            if np.isfinite(heights).all():
                world[:, 2] = heights + FOOT_RADIUS
                target_feet = (world - position) @ rotation
                try:
                    target_joints, _ = body.solve_joints(
                        self.kinematic, target_feet, joints
                    )
                    feet = target_feet
                except body.IKError:
                    pass
        self.q_init = target_joints.copy()
        self.q_des = target_joints.copy()
        self.foot_init_base = feet.copy()
        self.foot_desired_base = feet.copy()
        self.foot_init_hip = self.kinematic.base_to_hip(feet)
        self.foot_desired_hip = self.foot_init_hip.copy()
        self.foot_current_hip = self.kinematic.forward(joints)
        self.base_height_at_stand = (
            FOOT_RADIUS - np.mean(feet[:, 2])
        )
        super().reset_to_stand(joints)

    def update(self, q_current, command, navigation_state=None):
        self.observe_actual_state(q_current)
        return super().update(q_current, command, navigation_state)


GraspController = TerrainGraspController
_acquire_gym = gymapi.acquire_gym


class StableGym:
    """只为mode6提高PhysX求解精度，不读取虚拟足端接触力。"""

    def __init__(self, core):
        self.core = core

    def __getattr__(self, name):
        return getattr(self.core, name)

    def create_sim(self, compute, graphics, engine, params):
        params.physx.num_position_iterations = 8
        params.physx.num_velocity_iterations = 4
        return self.core.create_sim(compute, graphics, engine, params)


gymapi.acquire_gym = lambda: StableGym(_acquire_gym())


def feedback_sample_plan(plan, elapsed):
    target = _sample_plan(plan, elapsed)
    dock = _dock_instance
    if (
        dock is not None and dock.active
        and dock.motion_planner.feedback_enabled
        and plan.kind == "dock" and dock.state == "executing"
    ):
        dock.monitor_descent(plan, elapsed)
        if dock.state == "executing" and elapsed >= plan.times[-1]:
            dock._append_feedback_step(plan, elapsed)
    return target


def simulation_print(*values, **kwargs):
    periodic = (
        "机身世界坐标", "六足世界XY", "六足底部高度",
        "脚下实时背部高度", "仿真实际频率", "控制器平均耗时",
    )
    if (
        _dock_instance is not None
        and _dock_instance.active
        and values
        and isinstance(values[0], str)
        and values[0].startswith(periodic)
    ):
        return
    if values and values[0].startswith("手柄移动默认暂停"):
        values = ("A：启用三足步态  RT：升高  LT：下降  X：结束步态并锁定真实足端后对接",)
    elif values and values[0] == "识别标签ID：":
        values = ("位姿计算使用ID（含推断）：",) + values[1:]
    elif values and values[0] in ("继续调整下一条腿", "开始完整对接轨迹"):
        return
    elif values and values[0].startswith("第一阶段预对准完成"):
        values = ("连续水平与姿态预对准完成，开始单段S曲线连续下降",)
    _print(*values, **kwargs)


def print_plan_summary(plan):
    """终端只显示轨迹概要，不再刷出每个关节点。"""
    title = "支撑调整" if plan.kind == "support" else "完整对接"
    print(
        "{}轨迹：{}个点，轨迹时长{:.2f}s，仿真预计{:.1f}s".format(
            title, len(plan.joints), plan.times[-1],
            plan.times[-1] * dock_time_scale,
        )
    )


def set_contact_friction(gym, env, actor, friction):
    """脚垫正常接触；mode6为简化机身碰撞体补出中央插销通道。"""
    _set_contact_friction(gym, env, actor, friction)
    names = gym.get_actor_rigid_body_names(env, actor)
    if not any(name.endswith("_foot_link") for name in names):
        return
    ranges = gym.get_actor_rigid_body_shape_indices(env, actor)
    properties = gym.get_actor_rigid_shape_properties(env, actor)
    for name, index_range in zip(names, ranges):
        if not name.endswith("_foot_link"):
            for index in range(index_range.start, index_range.start + index_range.count):
                properties[index].rest_offset = (
                    -0.030 if name == "base_link" else -0.003
                )
    gym.set_actor_rigid_shape_properties(env, actor, properties)


def tf_message(parent, child, position, rotation, stamp):
    """按实时机身位姿输出Isaac Gym中的实际相机坐标。"""
    global _next_camera_log
    if child == "base_link":
        _body_pose[:] = position.copy(), rotation.copy()
        if _dock_instance is not None and _dock_instance.active:
            world_from_lock = body.transform(position, rotation)
            _dock_instance.pin_pose = np.linalg.inv(
                world_from_lock
            ) @ body.transform(PIN_WORLD)
    elif (
        child == "isaac_camera"
        and _body_pose[0] is not None
        and (_dock_instance is None or not _dock_instance.active)
    ):
        now = time.monotonic()
        if now >= _next_camera_log:
            xyz = _body_pose[0] + _body_pose[1] @ position
            print(
                "相机实际世界坐标 [mm]：{}，实际高度={:.1f} mm".format(
                    np.round(xyz * 1000.0, 1).tolist(),
                    xyz[2] * 1000.0,
                )
            )
            _next_camera_log = now + 2.0
    return _tf_message(parent, child, position, rotation, stamp)


print = simulation_print
print_plan = print_plan_summary
sample_plan = feedback_sample_plan


if __name__ == "__main__":
    main()
