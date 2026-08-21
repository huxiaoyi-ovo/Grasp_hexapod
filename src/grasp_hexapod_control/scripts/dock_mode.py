"""实机视觉对接模式。

DockMode只负责三件事：接管攀爬结束关节姿态、读取AprilTag TF、把闭环
机身修正转换为六足base_link目标。关节反馈、DLS逆运动学和舵机下发
统一走run_real.py -> GraspController链路。
"""

from dataclasses import dataclass
from itertools import combinations
from typing import Mapping

import numpy as np
import rospy
import tf2_ros
import yaml
from tf.transformations import quaternion_matrix

from kinematics import JOINT_VELOCITY_LIMIT
from utils import package_config_path, pose_to_transform, transform_points


# AprilTag感知：apriltag_ros发布相机到标签的动态TF，DockMode只查询完整TF链。
TAG_IDS = (0, 1, 2, 3)
TAG_SIZE = 0.040
TAG_DIRECTIONS = {0: "+y", 1: "+x", 2: "-y", 3: "-x"}
MAX_POSITION_ERROR = 0.03
MAX_ANGLE_ERROR = np.deg2rad(15.0)


def rigid_transform(translation=(0.0, 0.0, 0.0), rotation=None):
    """构造4x4刚体变换。"""
    result = np.eye(4, dtype=np.float64)
    if rotation is not None:
        result[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    result[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return result


def invert_transform(transform):
    """解析求取刚体变换的逆。"""
    transform = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    rotation = transform[:3, :3]
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation.T
    result[:3, 3] = -rotation.T @ transform[:3, 3]
    return result


# run_sim_dock.py使用的公共几何构造名称。
transform = rigid_transform


def load_dock_system(path=None):
    """加载底部相机现有的DOCK标签与外参配置。"""
    config_path = package_config_path("dock_system.yaml") if path is None else path
    with open(config_path, encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    if not isinstance(config, Mapping):
        raise ValueError("dock_system.yaml must be a mapping")
    tag_ids = tuple(config.get("tag_ids", ()))
    if tag_ids != TAG_IDS:
        raise ValueError("dock_system.yaml tag_ids must be exactly 0..3")
    tag_size = float(config.get("tag_size_m", 0.0))
    if config.get("tag_family") != "tag36h11" or tag_size <= 0.0:
        raise ValueError("dock_system.yaml requires tag36h11 and positive size")
    descriptions = config.get("standalone_tags")
    if not isinstance(descriptions, list) or {
        int(item.get("id", -1)) for item in descriptions
    } != set(tag_ids) or any(
        not np.isclose(float(item.get("size", 0.0)), tag_size)
        for item in descriptions
    ):
        raise ValueError("dock_system.yaml standalone_tags must match tag_ids/size")
    tag_frames = {
        int(item["id"]): str(item.get("name", "")).strip()
        for item in descriptions
    }
    if any(not frame for frame in tag_frames.values()) or len(
        set(tag_frames.values())
    ) != len(tag_frames):
        raise ValueError("dock_system.yaml tag names must be nonempty and unique")
    lock = config.get("lock_from_camera", {})
    lock_translation = np.asarray(lock.get("translation_m"), dtype=float)
    lock_rotation = np.asarray(lock.get("rotation"), dtype=float)
    if lock_translation.shape != (3,) or lock_rotation.shape != (3, 3):
        raise ValueError("dock_system.yaml lock_from_camera shape is invalid")
    if not np.isfinite(lock_translation).all() or not np.isfinite(lock_rotation).all():
        raise ValueError("dock_system.yaml lock_from_camera must be finite")
    if not np.allclose(lock_rotation.T @ lock_rotation, np.eye(3), atol=1e-8):
        raise ValueError("dock_system.yaml lock rotation must be orthogonal")
    pin_source = config.get("pin_from_tag_m")
    if not isinstance(pin_source, Mapping) or {
        int(tag_id) for tag_id in pin_source
    } != set(tag_ids):
        raise ValueError("dock_system.yaml pin_from_tag_m ids must match tag_ids")
    pin_from_tag = {}
    for tag_id in tag_ids:
        translation_m = np.asarray(pin_source[str(tag_id)], dtype=float)
        if translation_m.shape != (3,) or not np.isfinite(translation_m).all():
            raise ValueError("dock_system.yaml pin_from_tag_m must be finite xyz")
        pin_from_tag[tag_id] = transform(translation_m)
    if not isinstance(config.get("real_calibrated"), bool):
        raise ValueError("dock_system.yaml real_calibrated must be bool")
    return {
        "path": str(config_path),
        "tag_ids": tag_ids,
        "tag_size_m": tag_size,
        "tag_frames": tag_frames,
        "lock_from_camera": transform(lock_translation, lock_rotation),
        "pin_from_tag": pin_from_tag,
        "real_calibrated": config["real_calibrated"],
    }


DOCK_SYSTEM = load_dock_system()
TAG_IDS = DOCK_SYSTEM["tag_ids"]
TAG_SIZE = DOCK_SYSTEM["tag_size_m"]
LOCK_FROM_CAMERA = DOCK_SYSTEM["lock_from_camera"]
PIN_FROM_TAG = DOCK_SYSTEM["pin_from_tag"]
REAL_CALIBRATED = DOCK_SYSTEM["real_calibrated"]
TAG_FROM_PIN = {tag_id: invert_transform(pose) for tag_id, pose in PIN_FROM_TAG.items()}


def pose_matrix(pose):
    if np.shape(pose) == (4, 4):
        return np.asarray(pose, dtype=np.float64).copy()
    result = pose_to_transform(pose)
    if result is None:
        raise ValueError("invalid pose quaternion")
    return result


def pin_pose_from_detection(tag_id, detection, lock_from_camera=LOCK_FROM_CAMERA, tag_from_pin=None):
    """用一个已完整解码的标签计算插销相对卡紧机构的位姿。"""
    tag_from_pin = TAG_FROM_PIN if tag_from_pin is None else tag_from_pin
    return lock_from_camera @ pose_matrix(detection.pose.pose.pose) @ tag_from_pin[tag_id]


def rotation_angle(rotation):
    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.arccos(cosine))


def pose_difference(left, right):
    position = np.linalg.norm(left[:3, 3] - right[:3, 3])
    rotation = left[:3, :3].T @ right[:3, :3]
    return position, rotation_angle(rotation)


def consistent_poses(poses, max_position=MAX_POSITION_ERROR, max_angle=MAX_ANGLE_ERROR):
    """多标签同时可见时，选择彼此一致的最大候选集合。"""
    if len(poses) < 2:
        return poses
    for size in range(len(poses), 1, -1):
        valid_groups = []
        for group in combinations(poses, size):
            errors = [pose_difference(a[1], b[1]) for a, b in combinations(group, 2)]
            if all(p <= max_position and a <= max_angle for p, a in errors):
                score = sum(p / max_position + a / max_angle for p, a in errors)
                valid_groups.append((score, group))
        if valid_groups:
            return list(min(valid_groups, key=lambda item: item[0])[1])
    return []


def fuse_poses(poses):
    """融合一致位姿，并返回最大位置和角度离散值。"""
    result = poses[0][1].copy()
    result[:3, 3] = np.mean([pose[:3, 3] for _, pose in poses], axis=0)
    left, _, right = np.linalg.svd(np.mean([pose[:3, :3] for _, pose in poses], axis=0))
    left[:, -1] *= np.linalg.det(left @ right)
    result[:3, :3] = left @ right
    errors = [pose_difference(result, pose) for _, pose in poses]
    return result, max(p for p, _ in errors), max(a for _, a in errors)


def pose_is_plausible(pose, lock_from_camera=LOCK_FROM_CAMERA):
    if pose is None or np.shape(pose) != (4, 4) or not np.isfinite(pose).all():
        return False
    rotation = pose[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3):
        return False
    if np.linalg.det(rotation) < 0.999:
        return False
    camera_from_pin = invert_transform(lock_from_camera) @ pose
    return 0.01 <= camera_from_pin[2, 3] <= 1.5 and np.linalg.norm(camera_from_pin[:2, 3]) <= 0.5


def confidence_score(decoded_count, position_spread, angle_spread):
    base = 0.6 + 0.12 * (decoded_count - 1)
    agreement = 1.0 - 0.25 * (
        min(position_spread / MAX_POSITION_ERROR, 1.0)
        + min(angle_spread / MAX_ANGLE_ERROR, 1.0)
    )
    return float(np.clip(base * agreement, 0.0, 1.0))


@dataclass(frozen=True)
class PerceptionResult:
    valid: bool = False
    lock_from_pin: object = None
    stamp: object = None
    decoded_ids: tuple = ()
    inferred_ids: tuple = ()
    confidence: float = 0.0
    position_spread: float = float("inf")
    angle_spread: float = float("inf")
    reason: str = "no perception result"


class DockPerception:
    """通过TF读取每个完整标签对应的锁紧机构到插销位姿。"""
    def __init__(self, max_age=0.35, lock_frame="dock_lock_center",
                 pin_frame_prefix="dock_pin_from_tag_", tf_buffer=None,
                 dock_system_path=None):
        dock_system = load_dock_system(dock_system_path)
        self.max_age = float(max_age)
        self.lock_from_camera = dock_system["lock_from_camera"]
        self.lock_frame = str(lock_frame)
        self.pin_frames = {
            tag_id: "{}{}".format(pin_frame_prefix, tag_id) for tag_id in TAG_IDS
        }
        self.tf_buffer = tf_buffer or tf2_ros.Buffer()
        self.tf_listener = (
            None if tf_buffer is not None
            else tf2_ros.TransformListener(self.tf_buffer)
        )
        self.stamp = None
        self.result = PerceptionResult()

    @staticmethod
    def _matrix(transform_stamped):
        transform_msg = transform_stamped.transform
        quaternion = transform_msg.rotation
        pose = quaternion_matrix((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        translation = transform_msg.translation
        pose[:3, 3] = (translation.x, translation.y, translation.z)
        return pose

    def _invalid(self, reason):
        self.result = PerceptionResult(stamp=self.stamp, reason=reason)
        return self.result

    def reset(self):
        self.result = PerceptionResult()

    def latest(self, max_age=None):
        max_age = self.max_age if max_age is None else float(max_age)
        now = rospy.Time.now()
        candidates = []
        stamps = {}
        for tag_id, pin_frame in self.pin_frames.items():
            try:
                transform_stamped = self.tf_buffer.lookup_transform(
                    self.lock_frame, pin_frame, rospy.Time(0), rospy.Duration(0.0)
                )
                stamp = transform_stamped.header.stamp
                self.stamp = stamp
                if abs((now - stamp).to_sec()) > max_age:
                    continue
                pose = self._matrix(transform_stamped)
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
                AttributeError,
                TypeError,
                ValueError,
            ):
                continue
            if np.isfinite(pose).all():
                candidates.append((tag_id, pose))
                stamps[tag_id] = stamp
        if not candidates:
            return self._invalid("no fresh complete dock TF")
        candidates.sort(
            key=lambda item: stamps[item[0]].to_sec(), reverse=True
        )
        poses = consistent_poses(candidates)
        poses_agree = bool(poses)
        # 一致性只作为观测质量诊断。只要存在完整且有限的标签位姿，
        # 就使用时间戳最新的候选，不按ID编号设置优先级。
        if not poses:
            poses = candidates[:1]
        pose, position_spread, angle_spread = fuse_poses(poses)
        decoded = tuple(sorted(tag_id for tag_id, _ in poses))
        self.stamp = max(
            (stamps[tag_id] for tag_id in decoded), key=lambda stamp: stamp.to_sec()
        )
        plausible = pose_is_plausible(pose, self.lock_from_camera)
        tag_text = ",".join(
            "ID{}({})".format(tag_id, TAG_DIRECTIONS[tag_id])
            for tag_id in decoded
        )
        quality_reason = "AprilTag {} pose".format(tag_text)
        if not poses_agree:
            quality_reason = "tag poses disagree; using {}".format(tag_text)
        elif not plausible:
            quality_reason += "; outside nominal camera workspace"
        self.result = PerceptionResult(
            True, pose, self.stamp, decoded, (),
            confidence_score(len(decoded), position_spread, angle_spread),
            position_spread, angle_spread, quality_reason,
        )
        return self.result


# 最小闭环状态机。
def limited_transform(target, max_translation, max_angle):
    """把视觉误差限制为一个控制周期可执行的小位姿增量。"""
    target = np.asarray(target, dtype=np.float64).reshape(4, 4)
    result = np.eye(4, dtype=np.float64)
    translation = target[:3, 3]
    distance = float(np.linalg.norm(translation))
    if distance > max_translation > 0.0:
        translation = translation * (max_translation / distance)
    result[:3, 3] = translation
    rotation = target[:3, :3]
    angle = rotation_angle(rotation)
    if angle <= max_angle or angle < 1e-9:
        result[:3, :3] = rotation
        return result
    axis = np.array((
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ))
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-9:
        values, vectors = np.linalg.eigh(rotation)
        axis = vectors[:, int(np.argmin(np.abs(values - 1.0)))]
    else:
        axis /= axis_norm
    skew = np.array((
        (0.0, -axis[2], axis[1]),
        (axis[2], 0.0, -axis[0]),
        (-axis[1], axis[0], 0.0),
    ))
    result[:3, :3] = (
        np.eye(3) + np.sin(max_angle) * skew
        + (1.0 - np.cos(max_angle)) * (skew @ skew)
    )
    return result


@dataclass(frozen=True)
class DockRobotState:
    joints: object = None
    lock_from_pin: object = None
    lock_confirmed: object = None


@dataclass(frozen=True)
class DockResult:
    foot_positions_base: object
    active: bool
    success: bool
    failed: bool
    request_lock: bool
    reason: str
    joint_positions: object = None
    state: str = ""


class DockMode:
    """用任一完整AprilTag到达预对接姿态，再锁存并向下对接。"""
    IDLE = "idle"
    CLIMB_TERMINAL_ENTRY = "climb_terminal_entry"
    WAITING_TAG = "waiting_tag"
    PREALIGN = "prealign"
    DESCENT = "descent"
    SIT_SETTLE = "sit_settle"
    LEG_LIFT = "leg_lift"
    ALIGNED = "aligned"
    SUCCESS = "success"
    FAILED = "failed"
    TERMINAL_STATES = (SUCCESS, FAILED)
    STATE_LABELS = {
        IDLE: "待机",
        CLIMB_TERMINAL_ENTRY: "恢复攀爬末端姿态",
        WAITING_TAG: "等待AprilTag",
        PREALIGN: "视觉预对准",
        DESCENT: "机械导向下降",
        SIT_SETTLE: "下坐稳定等待",
        LEG_LIFT: "腿部腾空",
        ALIGNED: "等待锁紧确认",
        SUCCESS: "对接成功",
        FAILED: "对接失败",
    }

    ENTRY_TRACKING_TOLERANCE = np.deg2rad(2.0)
    # 只用于从视觉调整切换到机械导向下降，不作为成功或失败限制。
    PREALIGN_POSITION_REFERENCE = 0.001
    PREALIGN_TILT_REFERENCE = MAX_ANGLE_ERROR
    LINEAR_SPEED_M_S = 0.150
    PREALIGN_ANGULAR_SPEED = np.deg2rad(10.0)
    LEG_LIFT_HEIGHT_M = 0.020
    LEG_LIFT_SPEED_M_S = 0.050
    SIT_SETTLE_DURATION_S = 0.5
    # None表示在进入下降时使用TF的垂直距离；也可在本文件中改为固定米数。
    DESCENT_DISTANCE_M = None

    def __init__(self, controller, perception=None, require_lock_confirmation=False,
                 linear_speed_m_s=LINEAR_SPEED_M_S, update_rate_hz=30.0,
                 perception_rate_hz=10.0,
                 leg_lift_speed_m_s=LEG_LIFT_SPEED_M_S,
                 sit_settle_duration_s=SIT_SETTLE_DURATION_S):
        self.controller = controller
        self.perception = perception or DockPerception()
        self.require_lock_confirmation = bool(require_lock_confirmation)
        self.linear_speed_m_s = float(linear_speed_m_s)
        if not np.isfinite(self.linear_speed_m_s) or self.linear_speed_m_s <= 0.0:
            raise ValueError("dock linear_speed_m_s must be finite and positive")
        self.leg_lift_speed_m_s = float(leg_lift_speed_m_s)
        if (
            not np.isfinite(self.leg_lift_speed_m_s)
            or self.leg_lift_speed_m_s <= 0.0
        ):
            raise ValueError("dock leg_lift_speed_m_s must be finite and positive")
        self.sit_settle_duration_s = float(sit_settle_duration_s)
        if (
            not np.isfinite(self.sit_settle_duration_s)
            or self.sit_settle_duration_s < 0.0
        ):
            raise ValueError("dock sit_settle_duration_s must be finite and nonnegative")
        self.update_rate_hz = float(update_rate_hz)
        if not np.isfinite(self.update_rate_hz) or self.update_rate_hz <= 0.0:
            raise ValueError("dock update_rate_hz must be finite and positive")
        self.perception_rate_hz = float(perception_rate_hz)
        if (
            not np.isfinite(self.perception_rate_hz)
            or self.perception_rate_hz <= 0.0
        ):
            raise ValueError("dock perception_rate_hz must be finite and positive")
        self.update_period = 1.0 / self.update_rate_hz
        self.perception_period = 1.0 / self.perception_rate_hz
        self.update_elapsed = 0.0
        self.update_dt = float(self.controller.dt)
        self.last_update_result = None
        self.perception_elapsed = 0.0
        self.perception_sampled = False
        self.last_perception_reason = "no perception result"
        self.using_last_complete_frame = False
        self.active = False
        self.state = self.IDLE
        self.reason = ""
        self.entry_start = None
        self.entry_target = None
        self.entry_elapsed = 0.0
        self.entry_duration = 0.0
        self.descent_total = 0.0
        self.descent_remaining = 0.0
        self.descent_duration = 0.0
        self.sit_settle_elapsed = 0.0
        self.sit_settle_feet = None
        self.leg_lift_start_feet = None
        self.leg_lift_progress = 0.0
        self.cached_pose = None
        self.cached_ids = ()

    @staticmethod
    def _field(source, name):
        if source is None:
            return None
        return source.get(name) if isinstance(source, Mapping) else getattr(source, name, None)

    def _set_state(self, state, reason):
        changed = state != self.state
        self.state = state
        self.reason = reason
        if changed:
            logger = rospy.logwarn if state == self.FAILED else rospy.loginfo
            logger(
                "DockMode阶段: %s (%s) - %s",
                self.STATE_LABELS.get(state, state),
                state,
                reason or "无",
            )

    def enter(self, current_joints, climb_terminal_joints=None):
        """先回到控制器保留的攀爬末关节姿态，再开放视觉伺服。"""
        current = np.asarray(current_joints, dtype=np.float64).reshape(6, 3)
        target = current if climb_terminal_joints is None else np.asarray(
            climb_terminal_joints, dtype=np.float64
        ).reshape(6, 3)
        if not np.isfinite(current).all() or not np.isfinite(target).all():
            raise ValueError("dock entry joints must be finite")
        self.active = True
        self.perception.reset()
        self.entry_start = current.copy()
        self.entry_target = target.copy()
        self.entry_elapsed = 0.0
        self.descent_total = 0.0
        self.descent_remaining = 0.0
        self.descent_duration = 0.0
        self.sit_settle_elapsed = 0.0
        self.sit_settle_feet = None
        self.leg_lift_start_feet = None
        self.leg_lift_progress = 0.0
        self.cached_pose = None
        self.cached_ids = ()
        self.update_elapsed = max(0.0, self.update_period - self.controller.dt)
        self.update_dt = self.update_period
        self.last_update_result = None
        self.perception_elapsed = max(
            0.0, self.perception_period - self.controller.dt
        )
        self.perception_sampled = False
        self.last_perception_reason = "no perception result"
        self.using_last_complete_frame = False
        ratio = np.abs(target - current) / JOINT_VELOCITY_LIMIT
        self.entry_duration = max(0.5, 1.875 * float(np.max(ratio)))
        self._set_state(self.CLIMB_TERMINAL_ENTRY, "正在进入攀爬结束关节姿态")

    def exit(self):
        self.active = False
        self.entry_start = None
        self.entry_target = None
        self.descent_total = 0.0
        self.descent_remaining = 0.0
        self.descent_duration = 0.0
        self.sit_settle_elapsed = 0.0
        self.sit_settle_feet = None
        self.leg_lift_start_feet = None
        self.leg_lift_progress = 0.0
        self.cached_pose = None
        self.cached_ids = ()
        self.update_elapsed = 0.0
        self.last_update_result = None
        self.perception_elapsed = 0.0
        self.perception_sampled = False
        self.last_perception_reason = "no perception result"
        self.using_last_complete_frame = False
        self._set_state(self.IDLE, "")

    def fail_execution(self, reason):
        if self.active:
            self._set_state(self.FAILED, str(reason))

    def _perception_pose(self, robot_state):
        raw = self._field(robot_state, "lock_from_pin")
        decoded_ids = tuple(self._field(robot_state, "decoded_ids") or ())
        if raw is None:
            self.perception_elapsed += self.update_dt
            refresh = (
                not self.perception_sampled
                or self.perception_elapsed + 1e-12 >= self.perception_period
            )
            if not refresh:
                if self.cached_pose is None:
                    return None, (), self.last_perception_reason
                reason = (
                    "使用最后完整AprilTag帧推算"
                    if self.using_last_complete_frame else ""
                )
                return self.cached_pose.copy(), self.cached_ids, reason

            self.perception_elapsed = 0.0
            observed = self.perception.latest()
            self.perception_sampled = True
            if not getattr(observed, "valid", False):
                self.last_perception_reason = observed.reason
                if self.cached_pose is None:
                    return None, (), observed.reason
                self.using_last_complete_frame = True
                return (
                    self.cached_pose.copy(), self.cached_ids,
                    "使用最后完整AprilTag帧推算",
                )
            raw = observed.lock_from_pin
            decoded_ids = tuple(observed.decoded_ids)
            self.last_perception_reason = ""
            self.using_last_complete_frame = False
        else:
            self.using_last_complete_frame = False
        try:
            pose = np.asarray(raw, dtype=np.float64).reshape(4, 4)
        except (TypeError, ValueError):
            return None, (), "invalid lock_from_pin shape"
        if not np.isfinite(pose).all():
            return None, (), "non-finite lock_from_pin"
        self.cached_pose = pose.copy()
        self.cached_ids = decoded_ids
        return pose, decoded_ids, ""

    def _actual_feet(self, joints):
        return self.controller.kinematic.forward_base(joints)

    def _result(self, feet=None, joints=None):
        return DockResult(
            feet, self.active, self.state == self.SUCCESS, self.state == self.FAILED,
            self.state == self.ALIGNED, self.reason, joints, self.state,
        )

    def _update_entry(self, current):
        self.entry_elapsed = min(self.entry_duration, self.entry_elapsed + self.update_dt)
        phase = self.entry_elapsed / self.entry_duration
        blend = self.controller._smooth_step(phase)
        command = (1.0 - blend) * self.entry_start + blend * self.entry_target
        if phase >= 1.0:
            error = float(np.max(np.abs(current - self.entry_target)))
            quality = "达标" if error <= self.ENTRY_TRACKING_TOLERANCE else "仅供参考"
            self._set_state(
                self.WAITING_TAG,
                "攀爬结束姿态指令已完成，关节误差{:.2f}deg（{}）；等待完整AprilTag".format(
                    np.rad2deg(error), quality
                ),
            )
        return self._result(joints=command)

    def _synced_feet(self, current):
        sync_actual_feet = getattr(self.controller, "_sync_actual_feet", None)
        return (
            sync_actual_feet(current)
            if callable(sync_actual_feet) else self._actual_feet(current)
        )

    def _visual_step(self, current, pose):
        actual_feet = self._synced_feet(current)
        body_correction = invert_transform(pose)
        body_correction[2, 3] = 0.0
        increment = limited_transform(
            body_correction,
            self.linear_speed_m_s * self.update_dt,
            self.PREALIGN_ANGULAR_SPEED * self.update_dt,
        )
        self.cached_pose = increment @ pose
        return transform_points(increment, actual_feet)

    def _descent_step(self, current):
        step = min(
            self.linear_speed_m_s * self.update_dt,
            self.descent_remaining,
        )
        self.descent_remaining -= step
        feet = transform_points(
            transform((0.0, 0.0, step)), self._synced_feet(current)
        )
        if self.descent_remaining <= 1e-9:
            self.sit_settle_elapsed = 0.0
            self.sit_settle_feet = feet.copy()
            self.leg_lift_start_feet = None
            self.leg_lift_progress = 0.0
            self._set_state(
                self.SIT_SETTLE,
                "下坐完成，稳定等待{:.1f}s".format(self.sit_settle_duration_s),
            )
        else:
            self._set_state(
                self.DESCENT,
                "机械导向下降中，剩余{:.1f}mm，预计{:.2f}s".format(
                    self.descent_remaining * 1000.0,
                    self.descent_remaining / self.linear_speed_m_s,
                ),
            )
        return self._result(feet=feet)

    def _sit_settle_step(self, current):
        self.sit_settle_elapsed = min(
            self.sit_settle_duration_s,
            self.sit_settle_elapsed + self.update_dt,
        )
        if self.sit_settle_elapsed >= self.sit_settle_duration_s:
            self._set_state(self.LEG_LIFT, "下坐稳定完成，开始将六腿同步抬起20mm")
            return self._leg_lift_step(current)
        self._set_state(
            self.SIT_SETTLE,
            "下坐稳定等待中：{:.2f}/{:.2f}s".format(
                self.sit_settle_elapsed, self.sit_settle_duration_s
            ),
        )
        return self._result(feet=self.sit_settle_feet.copy())

    def _leg_lift_step(self, current):
        if self.leg_lift_start_feet is None:
            self.leg_lift_start_feet = self._synced_feet(current).copy()

        if self.leg_lift_progress >= self.LEG_LIFT_HEIGHT_M:
            if self.require_lock_confirmation:
                self._set_state(self.ALIGNED, "六腿已抬起20mm，等待锁紧机构确认")
            else:
                self._set_state(self.SUCCESS, "六腿已抬起20mm，对接结束")
            return self._result(joints=current.copy())

        self.leg_lift_progress = min(
            self.LEG_LIFT_HEIGHT_M,
            self.leg_lift_progress + self.leg_lift_speed_m_s * self.update_dt,
        )
        feet = self.leg_lift_start_feet.copy()
        feet[:, 2] += self.leg_lift_progress
        self._set_state(
            self.LEG_LIFT,
            "六腿同步抬起中：{:.1f}/20.0mm".format(
                self.leg_lift_progress * 1000.0
            ),
        )
        return self._result(feet=feet)

    def update(self, robot_state=None):
        if not self.active:
            raise RuntimeError("enter() must be called before update()")
        if self.state in self.TERMINAL_STATES:
            return self._update_once(robot_state)

        self.update_elapsed += self.controller.dt
        if (
            self.last_update_result is not None
            and self.update_elapsed + 1e-12 < self.update_period
        ):
            return self.last_update_result

        self.update_dt = self.update_elapsed
        self.update_elapsed = 0.0
        self.last_update_result = self._update_once(robot_state)
        return self.last_update_result

    def _update_once(self, robot_state=None):
        # 终态优先返回，避免run_real进入HOLD、不再注入DockRobotState后
        # 把已经确认的SUCCESS覆盖成关节反馈缺失FAILED。
        if self.state in self.TERMINAL_STATES:
            terminal_joints = self._field(robot_state, "joints")
            if terminal_joints is None:
                return self._result()
            try:
                terminal_joints = np.asarray(
                    terminal_joints, dtype=np.float64
                ).reshape(6, 3)
            except (TypeError, ValueError):
                return self._result()
            return self._result(joints=terminal_joints.copy())

        current = self._field(robot_state, "joints")
        if current is None:
            self.fail_execution("对接执行缺少统一控制链路的关节反馈")
            return self._result()
        try:
            current = np.asarray(current, dtype=np.float64).reshape(6, 3)
        except (TypeError, ValueError):
            self.fail_execution("关节反馈形状无效")
            return self._result()
        if not np.isfinite(current).all():
            self.fail_execution("关节反馈包含非有限值")
            return self._result()

        if self.state == self.CLIMB_TERMINAL_ENTRY:
            return self._update_entry(current)

        # 下降开始后不再依赖AprilTag，避免标签离开视野时中断机械对接。
        if self.state == self.DESCENT:
            return self._descent_step(current)
        if self.state == self.SIT_SETTLE:
            return self._sit_settle_step(current)
        if self.state == self.LEG_LIFT:
            return self._leg_lift_step(current)

        confirmed = self._field(robot_state, "lock_confirmed")
        if confirmed is True:
            self._set_state(self.SUCCESS, "锁紧机构已确认，对接成功")
            return self._result(joints=current.copy())
        if self.state == self.ALIGNED:
            return self._result(joints=current.copy())

        pose, decoded_ids, perception_reason = self._perception_pose(robot_state)
        if pose is None:
            self._set_state(self.WAITING_TAG, "等待完整AprilTag：" + perception_reason)
            # 本次对接尚未得到过完整标签时保持当前位置。
            return self._result(joints=current.copy())

        horizontal = float(np.linalg.norm(pose[:2, 3]))
        tilt = float(np.arccos(np.clip(pose[2, 2], -1.0, 1.0)))
        ready = (
            horizontal <= self.PREALIGN_POSITION_REFERENCE
            and tilt <= self.PREALIGN_TILT_REFERENCE
        )
        tags = ",".join(
            "ID{}({})".format(tag_id, TAG_DIRECTIONS[tag_id])
            for tag_id in decoded_ids
        ) or "外部TF"
        if perception_reason:
            tags += "[最后完整帧推算]"
        if ready:
            # 水平和姿态已经进入导向锥可接管的预期区域。只锁存向下
            # 行程，后续不再用视觉修正横向位置或姿态。
            tf_distance = max(0.0, -float(pose[2, 3]))
            self.descent_total = (
                tf_distance
                if self.DESCENT_DISTANCE_M is None
                else max(0.0, float(self.DESCENT_DISTANCE_M))
            )
            self.descent_remaining = self.descent_total
            self.descent_duration = (
                self.descent_total / self.linear_speed_m_s
            )
            self._set_state(
                self.DESCENT,
                "{}到达下降参考：水平{:.1f}mm，倾斜{:.2f}deg".format(
                    tags, horizontal * 1000.0, np.rad2deg(tilt)
                ),
            )
            return self._descent_step(current)

        self._set_state(
            self.PREALIGN,
            "{}视觉调整：水平{:.1f}mm，倾斜{:.2f}deg".format(
                tags, horizontal * 1000.0, np.rad2deg(tilt)
            ),
        )
        # 保持当前高度，只调整水平位置并使机身尽量与标签平行。
        feet = self._visual_step(current, pose)
        return self._result(feet=feet)

    def descent_has_started(self):
        return self.state in (
            self.DESCENT, self.SIT_SETTLE, self.LEG_LIFT,
            self.ALIGNED, self.SUCCESS
        )


def self_check():
    """不依赖pytest的单文件四标签与下降状态机自检。"""
    from types import SimpleNamespace

    for tag_id in TAG_IDS:
        camera_from_tag = invert_transform(LOCK_FROM_CAMERA) @ PIN_FROM_TAG[tag_id]
        detection = SimpleNamespace(
            pose=SimpleNamespace(pose=SimpleNamespace(pose=camera_from_tag))
        )
        if not np.allclose(pin_pose_from_detection(tag_id, detection), np.eye(4)):
            raise AssertionError("ID{} TF self-check failed".format(tag_id))

    class Controller:
        dt = 0.1
        kinematic = SimpleNamespace(
            forward_base=lambda joints: np.zeros((6, 3))
        )

        @staticmethod
        def _smooth_step(phase):
            return phase

        @staticmethod
        def _sync_actual_feet(joints):
            return np.zeros((6, 3))

    class Perception:
        def __init__(self, pose):
            self.pose = pose

        def reset(self):
            pass

        def latest(self):
            return PerceptionResult(
                valid=True, lock_from_pin=self.pose, decoded_ids=(2,)
            )

    class DropoutPerception(Perception):
        def __init__(self, pose):
            super().__init__(pose)
            self.calls = 0

        def latest(self):
            self.calls += 1
            if self.calls == 1:
                return super().latest()
            return PerceptionResult(reason="simulated tag loss")

    dropout = DockMode(
        Controller(), DropoutPerception(transform((0.025, 0.0, -0.030)))
    )
    dropout.active, dropout.state = True, dropout.WAITING_TAG
    dropout_state = {"joints": np.zeros((6, 3))}
    dropout.update(dropout_state)
    cached_result = dropout.update(dropout_state)
    if "最后完整帧推算" not in cached_result.reason:
        raise AssertionError("last complete frame fallback self-check failed")
    for _ in range(20):
        if dropout.update(dropout_state).state == dropout.DESCENT:
            break
    if dropout.state != dropout.DESCENT:
        raise AssertionError("cached pose descent transition self-check failed")

    class FastController(Controller):
        dt = 1.0 / 30.0

        def __init__(self):
            self.feet = np.zeros((6, 3))

        def _sync_actual_feet(self, joints):
            return self.feet.copy()

    class CountingPerception(Perception):
        def __init__(self, pose):
            super().__init__(pose)
            self.calls = 0

        def latest(self):
            self.calls += 1
            return super().latest()

    counted = CountingPerception(transform((0.030, 0.0, -0.030)))
    fast_controller = FastController()
    rate_mode = DockMode(
        fast_controller, counted,
        update_rate_hz=30.0, perception_rate_hz=10.0,
    )
    rate_mode.enter(np.zeros((6, 3)))
    rate_mode.state = rate_mode.WAITING_TAG
    frame_steps = []
    for _ in range(4):
        result = rate_mode.update(dropout_state)
        frame_steps.append(float(np.max(np.linalg.norm(
            result.foot_positions_base - fast_controller.feet, axis=1
        ))))
        fast_controller.feet = result.foot_positions_base.copy()
    if counted.calls != 2:
        raise AssertionError("30Hz control/10Hz perception self-check failed")
    if not np.allclose(frame_steps, 0.005):
        raise AssertionError("150mm/s continuous target self-check failed")

    class MissingPerception(Perception):
        def __init__(self):
            super().__init__(None)
            self.calls = 0

        def latest(self):
            self.calls += 1
            return PerceptionResult(reason="no tag")

    missing = MissingPerception()
    waiting_mode = DockMode(
        FastController(), missing,
        update_rate_hz=30.0, perception_rate_hz=10.0,
    )
    waiting_mode.enter(np.zeros((6, 3)))
    waiting_mode.state = waiting_mode.WAITING_TAG
    for _ in range(4):
        waiting_mode.update(dropout_state)
    if missing.calls != 2:
        raise AssertionError("waiting TF must remain at 10Hz")

    pose = transform((0.0, 0.0, -0.030))
    mode = DockMode(Controller(), Perception(pose))
    mode.active, mode.state = True, mode.WAITING_TAG
    state = {"joints": np.zeros((6, 3))}
    if mode.update(state).state != mode.DESCENT:
        raise AssertionError("descent transition self-check failed")
    mode.perception.latest = lambda: (_ for _ in ()).throw(
        AssertionError("descent must not read AprilTag again")
    )
    highest_lift = 0.0
    lift_targets = []
    settle_frames = 0
    for _ in range(50):
        result = mode.update(state)
        if result.state == mode.SIT_SETTLE:
            settle_frames += 1
        if result.state == mode.LEG_LIFT and result.foot_positions_base is not None:
            lift_height = float(np.max(result.foot_positions_base[:, 2]))
            highest_lift = max(
                highest_lift,
                lift_height,
            )
            if result.reason.startswith("六腿同步抬起中"):
                lift_targets.append(lift_height)
        if mode.state in mode.TERMINAL_STATES:
            break
    if mode.state != mode.SUCCESS:
        raise AssertionError("dock completion self-check failed")
    if not np.isclose(settle_frames * mode.controller.dt, 0.5):
        raise AssertionError("0.5s sit settle self-check failed")
    if not np.isclose(highest_lift, mode.LEG_LIFT_HEIGHT_M):
        raise AssertionError("20mm leg lift self-check failed")
    if not np.allclose(np.diff([0.0] + lift_targets), 0.005):
        raise AssertionError("50mm/s leg lift self-check failed")
    return True


__all__ = (
    "DockMode", "DockResult", "DockRobotState", "DockPerception", "PerceptionResult",
    "TAG_IDS", "TAG_SIZE", "TAG_DIRECTIONS", "LOCK_FROM_CAMERA", "PIN_FROM_TAG", "REAL_CALIBRATED",
    "load_dock_system", "self_check",
)
