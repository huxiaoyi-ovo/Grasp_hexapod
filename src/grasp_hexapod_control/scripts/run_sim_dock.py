#!/usr/bin/env python3
"""六足、小蓝、底部相机与正式DockMode的单文件Isaac Gym联合仿真。

只复用工程公共的dock_mode、control、kinematics和utils；不再依赖任何
run_sim_dock_mode*或isaacgym_apriltag_camera仿真脚本。启动状态由compact
攀爬终态重建，确保对接首帧连续承接攀爬末帧。
"""

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
# Ubuntu 20.04的OpenCV/aruco由apt安装在这里；放到Conda路径之后，
# 只补齐ROS系统包，不覆盖Isaac Gym环境中的NumPy和PyTorch。
SYSTEM_DIST_PACKAGES = Path("/usr/lib/python3/dist-packages")
if SYSTEM_DIST_PACKAGES.is_dir():
    sys.path.append(str(SYSTEM_DIST_PACKAGES))
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
from control import GraspController
from dock_mode import (
    LOCK_FROM_CAMERA,
    PIN_FROM_TAG,
    PerceptionResult,
    TAG_IDS,
    TAG_SIZE,
)
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
# 仅用于无人值守回归；默认关闭，不改变实体手柄Y键流程。
auto_dock_entry_test = os.environ.get(
    "DOCK_SIM_AUTO_ENTRY", "0"
).strip().lower() in ("1", "true", "yes")

# 当前联合仿真实际使用的底部相机与AprilTag几何。
ISAAC_FROM_OPTICAL_ROTATION = np.array(
    ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0))
)
LOCK_FROM_CAMERA_ROTATION = LOCK_FROM_CAMERA[:3, :3].copy()
CAMERA_POSITION_IN_LOCK = LOCK_FROM_CAMERA[:3, 3].copy()
PIN_WORLD = np.array((0.0, -0.028, 0.228))
TAG_SIZE_M = TAG_SIZE
PIN_FROM_OPENCV_TAG_ROTATION = np.diag((-1.0, -1.0, 1.0))
# 攀爬终态相机距标签面约38 mm，16:9画面的垂直视野无法容纳偏离光轴
# 35 mm的完整40 mm标签。对接请求后先把相机升到旧版已验证的260 mm，
# 对应标签面上方约69 mm；只有真实图像完整解码后才允许进入DockMode。
TAG_REACQUIRE_CAMERA_HEIGHT_M = 0.260
TAG_REACQUIRE_FRESH_S = 1.0
TAG_REACQUIRE_MIN_RAISE_M = 0.002


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
    target_joints, _ = body.solve_joints(
        controller.kinematic,
        target_feet_base,
        q_current,
    )

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


def configure_robot_contact(gym, env, actor, friction):
    """配置仿真接触几何；不参与DockMode控制判断。"""

    _base_set_contact_friction(gym, env, actor, friction)
    names = gym.get_actor_rigid_body_names(env, actor)
    ranges = gym.get_actor_rigid_body_shape_indices(env, actor)
    properties = gym.get_actor_rigid_shape_properties(env, actor)
    for name, index_range in zip(names, ranges):
        if name.endswith("_foot_link"):
            continue
        for index in range(
            index_range.start,
            index_range.start + index_range.count,
        ):
            properties[index].rest_offset = (
                -0.030 if name == "base_link" else -0.003
            )
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
            tag_position - pin_rotation @ PIN_FROM_TAG[tag_id][:3, 3]
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


class SimPerception:
    """把Isaac相机解码结果送入正式DockMode。"""

    def __init__(self):
        self.result = PerceptionResult(reason="waiting for simulated AprilTag")
        self.received_at = -np.inf

    def reset(self):
        # Y入口会调用reset；保留刚刚用于放行Y的同一帧相机结果。
        pass

    def inject(self, pose, decoded_ids):
        decoded_ids = tuple(sorted(int(value) for value in decoded_ids))
        if not decoded_ids:
            return
        self.result = PerceptionResult(
            True,
            np.asarray(pose, dtype=float).reshape(4, 4).copy(),
            rospy.Time.now(),
            decoded_ids,
            (),
            1.0,
            reason="Isaac Gym AprilTag pose",
        )
        self.received_at = time.monotonic()

    def latest(self, max_age=1.0):
        del max_age
        return self.result


class NullPublisher:
    def publish(self, _message):
        pass


def place_support_foot_on_surface(surface, foot, lock_from_pin, _leg):
    """仿真只把调足目标落到小蓝表面，不做额外搜索或安全包络。"""

    world_from_lock = body.transform(PIN_WORLD) @ np.linalg.inv(lock_from_pin)
    world = world_from_lock @ np.r_[np.asarray(foot, dtype=float), 1.0]
    height = surface.heights(world[None, :2])[0]
    if np.isfinite(height):
        world[2] = height + FOOT_RADIUS
    return (np.linalg.inv(world_from_lock) @ world)[:3]


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
    # PhysX仍可使用GPU；CPU版Torch只能包装CPU tensor pipeline。
    params.use_gpu_pipeline = torch.cuda.is_available()
    params.physx.use_gpu = True
    params.physx.solver_type = 1
    params.physx.num_position_iterations = 8
    params.physx.num_velocity_iterations = 4
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
    back_surface = BackSurface(
        XIAOLAN_ROOT / "meshes/base_link_newxiaolan.STL"
    )
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
    _base_set_contact_friction(gym, env, xiaolan, 5.0)

    controller = GraspController(dt=dt * control_interval)
    controller.approach_mode.step_height = 0.008
    controller.approach_mode.phase_duration = 0.45
    print("读取compact攀爬终态并重建对接初始关节分支...")
    climb_terminal = load_climb_terminal_state(controller)
    q_start, terminal_clearance_error = initialize_from_climb_terminal(
        controller, climb_terminal, back_surface
    )
    sim_perception = SimPerception()
    dock = body.DockMode(
        controller,
        perception=sim_perception,
        subscribe_joint_state=False,
        publish_trajectory=False,
        status_publisher=NullPublisher(),
    )
    dock.place_support_foot = lambda foot, pose, leg: (
        place_support_foot_on_surface(back_surface, foot, pose, leg)
    )
    controller.attach_dock_mode(dock)
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
    configure_robot_contact(gym, env, robot, 6.0)

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
        "/dock_camera/image_raw", "/dock_camera/camera_info", "isaac_camera",
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
    print("手柄移动默认暂停；A：启用原三角步态  B：回到站姿  Y：进入对接")
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
    foot_radius = np.mean(np.linalg.norm(
        controller.foot_init_base[:, :2], axis=1
    ))
    max_yaw_rate = max_linear_speed / foot_radius
    command = np.zeros(4)
    enabled = False
    pending_dock_entry = False
    tag_raise_started = False
    auto_entry_requested = False
    previous_a = previous_b = previous_y = False
    y_armed = False
    terminal_reported = False
    q_target = q_start.copy()
    step = 0
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
        body_state = rigid_body_tensor[base_index].detach().cpu().numpy()
        body_position = body_state[:3]
        body_rotation = body.quaternion_matrix(body_state[3:7])[:3, :3]
        world_from_base = body.transform(body_position, body_rotation)
        lock_from_pin = np.linalg.inv(world_from_base) @ body.transform(PIN_WORLD)

        joint_message = JointState()
        joint_message.header.stamp = rospy.Time.now()
        joint_message.name = list(body.joint_names)
        joint_message.position = q_current.reshape(-1).tolist()
        joint_publisher.publish(joint_message)

        _, right, forward, _, yaw = joystick.get_commands()
        up = joystick.get_height_command()
        a = joystick.get_button(0)
        b = joystick.get_button(1)
        y = joystick.get_button(3)
        if step > 30 and not y:
            y_armed = True
        press_a = a and not previous_a
        press_b = b and not previous_b
        press_y = y_armed and y and not previous_y
        scripted_y = bool(
            auto_dock_entry_test
            and not auto_entry_requested
            and controller.mode == controller.APPROACH
            and step > 60
        )
        previous_a, previous_b, previous_y = a, b, y

        if (press_y or scripted_y) and controller.mode != controller.DOCK:
            auto_entry_requested |= scripted_y
            y_armed = False
            pending_dock_entry = True
            tag_raise_started = False
            enabled = False
            print(
                "DOCK_SIM_AUTO_ENTRY：自动触发一次Y键对接回归"
                if scripted_y else "Y键对接请求已接收"
            )

        if press_a and controller.mode != controller.DOCK:
            enabled = not enabled
            print("手柄移动：", "启用" if enabled else "暂停")
            if enabled:
                controller.reset_to_stand(q_current)

        if press_b:
            pending_dock_entry = False
            tag_raise_started = False
            terminal_reported = False
            enabled = False
            if controller.dock_mode is not None and controller.dock_mode.active:
                controller.exit_dock(q_current)
            controller.set_mode(controller.APPROACH)
            controller.reset_to_stand(q_current)
            print("正在回到初始站姿")

        gait_finished = bool(
            not controller.reset_active
            and not controller.approach_mode.gait_started
            and not controller.approach_mode.transfer_active
        )
        complete_tag_ready = bool(
            local_pin_pose is not None
            and local_pin_ids
            and time.monotonic() - local_pin_stamp <= TAG_REACQUIRE_FRESH_S
        )
        if pending_dock_entry and gait_finished and not complete_tag_ready:
            if not tag_raise_started:
                raise_distance = start_tag_reacquisition_raise(
                    controller, q_current, body_position, body_rotation
                )
                tag_raise_started = True
                if raise_distance > 0.0:
                    print(
                        "保持当前足端并将相机升高{:.1f} mm以重捕获标签".format(
                            raise_distance * 1000.0
                        )
                    )
                else:
                    print("等待完整AprilTag")

        if (
            pending_dock_entry
            and gait_finished
            and complete_tag_ready
            and not press_b
        ):
            pending_dock_entry = False
            terminal_reported = False
            controller.enter_dock(q_current)
            print(
                "完整AprilTag已确认：ID={}；进入对接模式".format(
                    list(local_pin_ids)
                )
            )
            print("第一阶段恢复climb_compact终态并保持1.5秒")

        axes = np.array((right, forward), dtype=float)
        axes /= max(1.0, np.linalg.norm(axes))
        command[:] = (
            max_linear_speed * axes[0],
            max_linear_speed * axes[1],
            max_vertical_speed * up,
            max_yaw_rate * yaw,
        ) if enabled and controller.mode == controller.APPROACH else 0.0

        active_control = bool(
            controller.mode == controller.DOCK
            or controller.reset_active
            or pending_dock_entry
            or enabled
        )
        if step % control_interval == 0 and active_control:
            control_start = time.perf_counter()
            robot_state = None
            if controller.mode == controller.DOCK:
                robot_state = {
                    "joints": q_current,
                    "body_position": body_position,
                    "body_rotation": body_rotation,
                    "lock_from_pin": lock_from_pin,
                }
            q_target = controller.update(
                q_current,
                command,
                dock_robot_state=robot_state,
            )
            control_cost += time.perf_counter() - control_start

            dock = controller.dock_mode
            if (
                controller.mode == controller.DOCK
                and dock.state in dock.TERMINAL_STATES
                and not terminal_reported
            ):
                terminal_reported = True
                print(
                    "对接成功：{}".format(dock.reason)
                    if dock.state == dock.SUCCESS else
                    "对接失败：{}".format(dock.reason)
                )
                controller.exit_dock(q_current)
                controller.set_mode(controller.APPROACH)
                enabled = False

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
                    sim_perception.inject(pose, ids)
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
                _base_tf_message(
                    "base_footprint", "base_link",
                    body_position,
                    body_rotation, stamp,
                ),
                _base_tf_message(
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



if __name__ == "__main__":
    main()
