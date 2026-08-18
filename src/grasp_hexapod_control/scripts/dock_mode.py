"""单文件视觉对接模式。

包含完整/不完整ID感知、固定足端与自动调足轨迹、闭环运动规划，
以及仿真和实机共用的DockMode状态机。仅复用项目公共运动学和工具模块。
"""

from dataclasses import dataclass
from itertools import combinations
import sys
import time
from typing import Mapping

import cv2
import numpy as np
import rospy
import yaml
from apriltag_ros.msg import AprilTagDetectionArray
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import (
    quaternion_from_matrix,
    quaternion_matrix,
    quaternion_slerp,
)

from kinematics import (
    GraspKinematic,
    JOINT_LOWER,
    JOINT_UPPER,
    JOINT_VELOCITY_LIMIT,
    LEG_NAMES,
)
from utils import (
    CONTROL_DOF_NAMES,
    package_config_path,
    pose_to_transform,
    transform_points,
    yaw_from_transform,
)


def rigid_transform(translation=(0.0, 0.0, 0.0), rotation=None):
    """构造4×4刚体变换；平移单位为m，旋转为3×3正交矩阵。"""
    result = np.eye(4, dtype=np.float64)
    if rotation is not None:
        result[:3, :3] = np.asarray(
            rotation, dtype=np.float64
        ).reshape(3, 3)
    result[:3, 3] = np.asarray(
        translation, dtype=np.float64
    ).reshape(3)
    return result


def invert_transform(transform):
    """解析求取刚体变换的逆，避免通用矩阵求逆引入数值噪声。"""
    transform = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    rotation = transform[:3, :3]
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation.T
    result[:3, 3] = -rotation.T @ transform[:3, 3]
    return result


# =============================================================================
# 感知：完整ID融合与不完整ID标签板补全
# =============================================================================

TAG_IDS = (0, 1, 2, 3)
TAG_SIZE = 0.040
MIN_CONFIDENCE = 0.5
MAX_POSITION_ERROR = 0.03
MAX_ANGLE_ERROR = np.deg2rad(15.0)


# 保留旧仿真脚本使用的名称；正式实现已内置在本文件。
transform = rigid_transform


def load_dock_system(path=None):
    """严格加载底部USB相机的唯一DOCK标签与几何配置。"""

    config_path = package_config_path("dock_system.yaml") if path is None else path
    with open(config_path, encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    if not isinstance(config, Mapping):
        raise ValueError("dock_system.yaml must be a mapping")
    tag_ids = tuple(config.get("tag_ids", ()))
    if tag_ids != (0, 1, 2, 3):
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
TAG_FROM_PIN = {
    tag_id: np.linalg.inv(pose) for tag_id, pose in PIN_FROM_TAG.items()
}
PIN_FROM_OPENCV_TAG_ROTATION = np.diag((-1.0, -1.0, 1.0))
half_tag = TAG_SIZE / 2.0
TAG_CORNERS = np.array([
    (-half_tag, half_tag, 0.0),
    (half_tag, half_tag, 0.0),
    (half_tag, -half_tag, 0.0),
    (-half_tag, -half_tag, 0.0),
], dtype=np.float32)


def make_board(dictionary, pin_from_tag=PIN_FROM_TAG, tag_size=TAG_SIZE):
    half_tag = float(tag_size) / 2.0
    tag_corners = np.array([
        (-half_tag, half_tag, 0.0),
        (half_tag, half_tag, 0.0),
        (half_tag, -half_tag, 0.0),
        (-half_tag, -half_tag, 0.0),
    ], dtype=np.float32)
    corners = [
        (
            tag_corners @ PIN_FROM_OPENCV_TAG_ROTATION.T
            + pin_from_tag[tag_id][:3, 3]
        ).astype(np.float32)
        for tag_id in TAG_IDS
    ]
    return cv2.aruco.Board_create(
        corners, dictionary, np.array(TAG_IDS, dtype=np.int32)
    )


def pose_matrix(pose):
    result = pose_to_transform(pose)
    if result is None:
        raise ValueError("invalid pose quaternion")
    return result


def pin_pose_from_detection(
    tag_id,
    detection,
    lock_from_camera=LOCK_FROM_CAMERA,
    tag_from_pin=None,
):
    """用一个已解码标签独立计算插销相对卡紧机构的位姿。"""
    tag_from_pin = TAG_FROM_PIN if tag_from_pin is None else tag_from_pin
    return (
        lock_from_camera
        @ pose_matrix(detection.pose.pose.pose)
        @ tag_from_pin[tag_id]
    )


def pose_difference(left, right):
    """返回两个位姿的位置差和旋转角差。"""
    position = np.linalg.norm(left[:3, 3] - right[:3, 3])
    rotation = left[:3, :3].T @ right[:3, :3]
    angle = np.arccos(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    return position, angle


def consistent_poses(
    poses, max_position=MAX_POSITION_ERROR, max_angle=MAX_ANGLE_ERROR
):
    """选择彼此一致的最大候选集合。"""
    if len(poses) < 2:
        return poses
    for size in range(len(poses), 1, -1):
        valid_groups = []
        for group in combinations(poses, size):
            errors = [
                pose_difference(left[1], right[1])
                for left, right in combinations(group, 2)
            ]
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
    left, _, right = np.linalg.svd(
        np.mean([pose[:3, :3] for _, pose in poses], axis=0)
    )
    left[:, -1] *= np.linalg.det(left @ right)
    result[:3, :3] = left @ right
    errors = [pose_difference(result, pose) for _, pose in poses]
    return result, max(p for p, _ in errors), max(a for _, a in errors)


def pose_is_plausible(pose, lock_from_camera=LOCK_FROM_CAMERA):
    """检查刚体矩阵和插销是否位于相机前方的合理范围。"""
    if pose is None or np.shape(pose) != (4, 4) or not np.isfinite(pose).all():
        return False
    rotation = pose[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3):
        return False
    if np.linalg.det(rotation) < 0.999:
        return False
    camera_from_pin = np.linalg.inv(lock_from_camera) @ pose
    return (
        0.01 <= camera_from_pin[2, 3] <= 1.5
        and np.linalg.norm(camera_from_pin[:2, 3]) <= 0.5
    )


def confidence_score(decoded_count, inferred_count, position_spread, angle_spread):
    base = 0.6 + 0.12 * (decoded_count - 1) + 0.04 * inferred_count
    agreement = 1.0 - 0.25 * (
        min(position_spread / MAX_POSITION_ERROR, 1.0)
        + min(angle_spread / MAX_ANGLE_ERROR, 1.0)
    )
    return float(np.clip(base * agreement, 0.0, 1.0))


@dataclass(frozen=True)
class PerceptionResult:
    """一次视觉感知结果；位置单位为 m，角度单位为 rad。"""

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
    """接收视觉数据并保存最新的标准化感知结果。"""

    def __init__(
        self,
        detections_topic="/dock/tag_detections",
        image_topic="/dock_camera/image_raw",
        camera_info_topic="/dock_camera/camera_info",
        allow_inference=False,
        min_stable_frames=10,
        max_stable_position=0.003,
        max_stable_angle=np.deg2rad(2.0),
        max_age=0.2,
        lock_from_camera=LOCK_FROM_CAMERA,
        tag_from_pin=None,
        dock_system_path=None,
    ):
        dock_system = load_dock_system(dock_system_path)
        self.allow_inference = bool(allow_inference)
        self.min_stable_frames = max(1, int(min_stable_frames))
        self.max_stable_position = float(max_stable_position)
        self.max_stable_angle = float(max_stable_angle)
        self.max_age = float(max_age)
        self.lock_from_camera = np.asarray(
            dock_system["lock_from_camera"] if lock_from_camera is LOCK_FROM_CAMERA
            else lock_from_camera, dtype=float
        ).reshape(4, 4).copy()
        source = (
            {tag_id: np.linalg.inv(pose) for tag_id, pose in dock_system["pin_from_tag"].items()}
            if tag_from_pin is None else tag_from_pin
        )
        self.tag_from_pin = {
            int(tag_id): np.asarray(pose, dtype=float).reshape(4, 4).copy()
            for tag_id, pose in source.items()
        }
        self.stamp = None
        self.detections = {}
        self.image_stamp = None
        self.gray_image = None
        self.camera_info = None
        self.decoded_signature = ()
        self.stable_frames = 0
        self.stability_pose = None
        self.stability_stamp = None
        self.previous_pose = None
        self.previous_stamp = None
        self.result = PerceptionResult()
        self.bridge = CvBridge() if self.allow_inference else None
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36h11
        )
        self.parameters = cv2.aruco.DetectorParameters_create()
        pin_from_tag = {
            tag_id: np.linalg.inv(pose)
            for tag_id, pose in self.tag_from_pin.items()
        }
        self.board = make_board(
            self.dictionary, pin_from_tag, dock_system["tag_size_m"]
        )
        self.subscriber = rospy.Subscriber(
            detections_topic,
            AprilTagDetectionArray,
            self._callback,
            queue_size=1,
        )
        self.image_subscriber = None
        self.camera_info_subscriber = None
        if self.allow_inference:
            self.image_subscriber = rospy.Subscriber(
                image_topic, Image, self._image_callback, queue_size=1
            )
            self.camera_info_subscriber = rospy.Subscriber(
                camera_info_topic,
                CameraInfo,
                self._camera_info_callback,
                queue_size=1,
            )

    def _callback(self, message):
        self.stamp = message.header.stamp
        self.detections = {
            int(tag_id): detection
            for detection in message.detections
            for tag_id in detection.id
            if int(tag_id) in TAG_IDS
        }

    def _image_callback(self, message):
        try:
            self.gray_image = self.bridge.imgmsg_to_cv2(message, "mono8")
            self.image_stamp = message.header.stamp
        except CvBridgeError:
            self.gray_image = None
            self.image_stamp = None

    def _camera_info_callback(self, message):
        self.camera_info = message

    def _invalid(self, reason):
        stamp = self.stamp if self.stamp is not None else self.image_stamp
        self.result = PerceptionResult(stamp=stamp, reason=reason)
        return self.result

    def reset(self):
        """开始一次新对接时清除跨帧连续性记录。"""
        self.decoded_signature = ()
        self.stable_frames = 0
        self.stability_pose = None
        self.stability_stamp = None
        self.previous_pose = None
        self.previous_stamp = None
        self.result = PerceptionResult()

    def _candidate_poses(self):
        poses = []
        for tag_id in sorted(self.detections):
            try:
                pose = pin_pose_from_detection(
                    tag_id,
                    self.detections[tag_id],
                    self.lock_from_camera,
                    self.tag_from_pin,
                )
            except (AttributeError, KeyError, TypeError, ValueError):
                continue
            if np.isfinite(pose).all():
                poses.append((tag_id, pose))
        return poses

    def _recovered_board_pose(self):
        """用至少一个完整ID估计标签板，并推断其余ID。"""
        image, camera_info = self.gray_image, self.camera_info
        if image is None or camera_info is None:
            return None, (), ()
        try:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                image, self.dictionary, parameters=self.parameters
            )
        except cv2.error:
            return None, (), ()
        if ids is None:
            return None, (), ()
        keep = [i for i, tag_id in enumerate(ids.reshape(-1)) if tag_id in TAG_IDS]
        if not keep:
            return None, (), ()
        corners = [corners[i] for i in keep]
        ids = np.array([[int(ids[i, 0])] for i in keep], dtype=np.int32)
        decoded_ids = tuple(sorted(int(tag_id) for tag_id in ids.reshape(-1)))
        try:
            camera_matrix = np.asarray(camera_info.K).reshape(3, 3)
            distortion = np.asarray(camera_info.D)
            if (
                not np.isfinite(camera_matrix).all()
                or not np.isfinite(distortion).all()
                or min(camera_matrix[0, 0], camera_matrix[1, 1]) <= 0.0
            ):
                return None, decoded_ids, ()
            corners, ids, _, _ = cv2.aruco.refineDetectedMarkers(
                image,
                self.board,
                corners,
                ids,
                rejected,
                camera_matrix,
                distortion,
                10.0,
                -1.0,
            )
        except (cv2.error, TypeError, ValueError):
            return None, decoded_ids, ()
        inferred_ids = tuple(sorted(set(TAG_IDS) - set(decoded_ids)))
        try:
            used, rotation, translation = cv2.aruco.estimatePoseBoard(
                corners, ids, self.board, camera_matrix, distortion, None, None
            )
        except cv2.error:
            return None, decoded_ids, ()
        if not used:
            return None, decoded_ids, ()
        if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
            return None, decoded_ids, ()
        camera_from_pin = transform(
            translation.reshape(3), cv2.Rodrigues(rotation)[0]
        )
        return (
            self.lock_from_camera @ camera_from_pin,
            decoded_ids,
            inferred_ids,
        )

    def _image_is_synced(self, max_difference=0.1):
        if self.image_stamp is None or self.stamp is None:
            return False
        return abs((self.image_stamp - self.stamp).to_sec()) <= max_difference

    def _pose_is_continuous(self, pose, stamp, max_gap=0.5):
        if self.previous_pose is None or self.previous_stamp is None:
            return True
        gap = (stamp - self.previous_stamp).to_sec()
        if gap < 0.0:
            return False
        if gap > max_gap:
            return True
        position, angle = pose_difference(self.previous_pose, pose)
        return position <= 0.05 and angle <= np.deg2rad(20.0)

    def _pose_is_stable(self, pose, decoded, stamp):
        """每个新时间戳只计一次，并同时要求ID集合与位姿稳定。"""
        signature = tuple(sorted(decoded))
        stamp_key = None if stamp is None else stamp.to_nsec()
        if stamp_key == self.stability_stamp:
            return self.stable_frames >= self.min_stable_frames
        stable = False
        if signature and signature == self.decoded_signature:
            if self.stability_pose is not None:
                position, angle = pose_difference(self.stability_pose, pose)
                stable = (
                    position <= self.max_stable_position
                    and angle <= self.max_stable_angle
                )
        self.stable_frames = self.stable_frames + 1 if stable else 1
        self.decoded_signature = signature
        self.stability_pose = pose.copy()
        self.stability_stamp = stamp_key
        return self.stable_frames >= self.min_stable_frames

    def _accept(self, pose, stamp, decoded, inferred, spread=(0.0, 0.0), reason=""):
        position_spread, angle_spread = spread
        if not decoded:
            return self._invalid("no complete tag id")
        if inferred and not self.allow_inference:
            return self._invalid("inferred tag ids are disabled")
        if not pose_is_plausible(pose, self.lock_from_camera):
            return self._invalid("pin pose is outside camera workspace")
        if not self._pose_is_continuous(pose, stamp):
            return self._invalid("pin pose jumped")
        confidence = confidence_score(
            len(decoded), len(inferred), position_spread, angle_spread
        )
        if confidence < MIN_CONFIDENCE:
            return self._invalid("perception confidence is too low")
        self.result = PerceptionResult(
            True, pose, stamp, tuple(sorted(decoded)), tuple(sorted(inferred)),
            confidence, position_spread, angle_spread, reason,
        )
        self.previous_pose, self.previous_stamp = pose.copy(), stamp
        return self.result

    def latest(self, max_age=None):
        max_age = self.max_age if max_age is None else float(max_age)
        now = rospy.Time.now()
        detection_fresh = self.stamp is not None and 0.0 <= (
            now - self.stamp
        ).to_sec() <= max_age
        image_fresh = self.image_stamp is not None and 0.0 <= (
            now - self.image_stamp
        ).to_sec() <= max_age
        recovered, raw_ids, inferred_ids = (
            self._recovered_board_pose()
            if self.allow_inference and image_fresh
            else (None, (), ())
        )
        candidates = self._candidate_poses() if detection_fresh else []
        if not candidates:
            if recovered is None:
                return self._invalid("no tag detections")
            if not self._pose_is_stable(
                recovered, raw_ids, self.image_stamp
            ):
                return self._invalid(
                    "tag pose is not stable yet ({}/{})".format(
                        self.stable_frames, self.min_stable_frames
                    )
                )
            return self._accept(
                recovered, self.image_stamp, raw_ids, inferred_ids,
                reason="image board pose; inferred ids={}".format(inferred_ids),
            )

        poses = consistent_poses(candidates)
        if not poses:
            return self._invalid("tag poses disagree")
        pose, position_spread, angle_spread = fuse_poses(poses)
        kept_ids = {tag_id for tag_id, _ in poses}
        rejected = tuple(tag_id for tag_id, _ in candidates if tag_id not in kept_ids)
        if not self._image_is_synced():
            recovered, raw_ids, inferred_ids = None, (), ()
        recovery_rejected = False
        if recovered is not None:
            position_error, angle_error = pose_difference(pose, recovered)
            if position_error <= MAX_POSITION_ERROR and angle_error <= MAX_ANGLE_ERROR:
                inferred_ids = tuple(sorted(set(inferred_ids) - kept_ids))
                pose = recovered
                kept_ids.update(raw_ids)
                position_spread = max(position_spread, position_error)
                angle_spread = max(angle_spread, angle_error)
            else:
                inferred_ids = ()
                recovery_rejected = True
        if not self._pose_is_stable(pose, kept_ids, self.stamp):
            return self._invalid(
                "tag pose is not stable yet ({}/{})".format(
                    self.stable_frames, self.min_stable_frames
                )
            )
        details = []
        if rejected:
            details.append("rejected ids={}".format(rejected))
        if inferred_ids:
            details.append("inferred ids={}".format(inferred_ids))
        if recovery_rejected:
            details.append("recovered pose rejected")
        return self._accept(
            pose, self.stamp, kept_ids, inferred_ids,
            (position_spread, angle_spread),
            ", ".join(details) if details else "AprilTag pose",
        )

# =============================================================================
# 机身与单腿轨迹：固定足端IK、自动调足和轨迹采样
# =============================================================================

joint_names = CONTROL_DOF_NAMES
docked_pin_in_lock = np.zeros(3)
camera_position_in_lock = np.array((0.0, -0.065, -0.0325))

joint_lower = JOINT_LOWER
joint_upper = JOINT_UPPER


# 真实关节角读取：从/joint_states按固定顺序读取六条腿的18个关节角。
class JointReader:
    def __init__(self):
        self.values = None
        self.subscriber = rospy.Subscriber(
            "/joint_states", JointState, self._callback, queue_size=1
        )

    def _callback(self, message):
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in joint_names):
            self.values = np.array(
                [positions[name] for name in joint_names]
            ).reshape(6, 3)


def move_fixed_feet(feet, body_from_entry):
    return transform_points(body_from_entry, feet)


# 实测预对接高度：整平时保持按X瞬间的相机高度，不再使用固定300 mm。
def measured_pre_dock_pin(lock_from_pin):
    pin_from_lock = invert_transform(lock_from_pin)
    camera_in_pin = (
        pin_from_lock @ np.r_[camera_position_in_lock, 1.0]
    )[:3]
    return np.array((
        0.0, 0.0, camera_position_in_lock[2] - camera_in_pin[2]
    ))


# 目标机身变换：计算卡紧机构对准插销时，机身相对进入DOCK时的目标位姿。
def target_body_transform(lock_from_pin, desired_pin=None):
    """保持实测高度和当前偏航，仅对准水平位置与倾斜。"""
    if desired_pin is None:
        desired_pin = measured_pre_dock_pin(lock_from_pin)
    yaw = yaw_from_transform(lock_from_pin)
    cosine, sine = np.cos(yaw), np.sin(yaw)
    yaw_rotation = np.array(
        ((cosine, -sine, 0.0), (sine, cosine, 0.0), (0.0, 0.0, 1.0))
    )
    desired = transform(desired_pin, yaw_rotation)
    return desired @ invert_transform(lock_from_pin)


# 分段机身轨迹：按最多1 mm和0.5°将完整机身运动拆分为平滑轨迹点。
def interpolate_body(target, translation_step=0.001, rotation_step=np.deg2rad(0.5)):
    angle = np.arccos(np.clip((np.trace(target[:3, :3]) - 1.0) / 2.0, -1.0, 1.0))
    count = max(
        1,
        int(np.ceil(np.linalg.norm(target[:3, 3]) / translation_step)),
        int(np.ceil(angle / rotation_step)),
    )
    quaternion = quaternion_from_matrix(target)
    result = []
    for index in range(count + 1):
        phase = index / count
        phase = 10 * phase**3 - 15 * phase**4 + 6 * phase**5
        rotation = quaternion_matrix(
            quaternion_slerp((0.0, 0.0, 0.0, 1.0), quaternion, phase)
        )[:3, :3]
        result.append(transform(phase * target[:3, 3], rotation))
    return result


def interpolate_between(start, target):
    """在两个机身位姿间插值，下降阶段不再依赖视觉。"""
    return [
        pose @ start
        for pose in interpolate_body(target @ invert_transform(start))
    ]


class IKError(ValueError):
    """记录不可达轨迹点中误差最大的腿。"""

    def __init__(self, leg, residual, stage="", point=0):
        self.leg, self.residual = int(leg), float(residual)
        prefix = "{}阶段第{}点".format(stage, point) if stage else ""
        super().__init__(
            "{}{}腿IK不可达: {:.1f} mm".format(
                prefix, LEG_NAMES[self.leg], self.residual * 1000.0
            )
        )


# 单点逆运动学：六个足端固定，根据一个机身轨迹点反算对应的18个关节角。
def solve_joints(kinematic, target_feet, initial, tolerance=0.0005):
    target_hip = kinematic.base_to_hip(target_feet)
    joints = initial.copy()
    for _ in range(120):
        error = target_hip - kinematic.forward(joints)
        residual = float(np.max(np.linalg.norm(error, axis=1)))
        if residual <= tolerance:
            return joints, residual
        correction = (
            kinematic.damped_inverse_jacobian(joints, 0.003)
            @ error[..., None]
        ).squeeze(-1)
        joints = np.clip(
            joints + np.clip(correction, -0.05, 0.05),
            JOINT_LOWER, JOINT_UPPER,
        )
    residuals = np.linalg.norm(target_hip - kinematic.forward(joints), axis=1)
    residual = float(np.max(residuals))
    if residual <= tolerance:
        return joints, residual
    leg = int(np.argmax(residuals))
    raise IKError(leg, residuals[leg])


@dataclass
class Plan:
    body: list
    joints: np.ndarray
    times: np.ndarray
    max_error: float
    pre_dock_index: int
    kind: str = "dock"
    moving_leg: int = -1
    anchors: np.ndarray = None


def sample_plan(plan, elapsed):
    """按轨迹时间线性插值当前18关节目标。"""

    elapsed = max(0.0, float(elapsed))
    if elapsed >= float(plan.times[-1]):
        return np.asarray(plan.joints[-1], dtype=float).copy()
    right = max(1, int(np.searchsorted(
        plan.times, elapsed, side="right"
    )))
    left = right - 1
    interval = float(plan.times[right] - plan.times[left])
    phase = 1.0 if interval <= 0.0 else (
        elapsed - float(plan.times[left])
    ) / interval
    return (
        (1.0 - phase) * plan.joints[left]
        + phase * plan.joints[right]
    )


def remaining_plan(plan, elapsed, current_joints=None):
    """截取尚未执行的轨迹并把时间轴重新置零，供ROS重新发布。"""

    elapsed = max(0.0, float(elapsed))
    current = (
        sample_plan(plan, elapsed)
        if current_joints is None
        else np.asarray(current_joints, dtype=float).reshape(6, 3)
    )
    first = int(np.searchsorted(plan.times, elapsed, side="right"))
    future_joints = np.asarray(plan.joints[first:], dtype=float)
    future_times = np.asarray(plan.times[first:], dtype=float) - elapsed
    joints = np.r_[current[None, ...], future_joints]
    times = np.r_[0.0, future_times]
    if len(times) == 1:
        joints = np.r_[joints, current[None, ...]]
        times = np.array((0.0, 0.1), dtype=float)
    future_body = list(plan.body[first:first + len(future_joints)])
    if future_body:
        body_path = [future_body[0]] + future_body
    else:
        last_body = plan.body[-1] if plan.body else np.eye(4)
        body_path = [last_body] * len(joints)
    return Plan(
        body_path,
        joints,
        times,
        plan.max_error,
        max(0, int(plan.pre_dock_index) - first),
        plan.kind,
        plan.moving_leg,
        plan.anchors,
    )


def solve_feet_path(current_joints, feet_path, stage_for_point=None):
    """把一串base_link足端目标统一转换为关节路径。"""

    kinematic = GraspKinematic()
    joints = [np.asarray(current_joints, dtype=float).reshape(6, 3).copy()]
    errors = [0.0]
    for index, feet in enumerate(feet_path, 1):
        try:
            solved, residual = solve_joints(
                kinematic, feet, joints[-1]
            )
        except IKError as error:
            stage = "" if stage_for_point is None else stage_for_point(index)
            raise IKError(error.leg, error.residual, stage, index)
        joints.append(solved)
        errors.append(residual)
    return np.asarray(joints), max(errors)


def solve_body_path(current_joints, anchors, body_path, stage_for_point=None):
    """固定环境足端，把机身路径统一转换为关节路径。"""

    feet_path = [
        move_fixed_feet(anchors, body_pose)
        for body_pose in body_path[1:]
    ]
    return solve_feet_path(
        current_joints, feet_path, stage_for_point=stage_for_point
    )


def _leg_feet_path(start_feet, leg, waypoints, step=0.002):
    """沿若干三维航点生成单腿足端路径。"""

    feet = np.asarray(start_feet, dtype=float).reshape(6, 3).copy()
    path = []
    for waypoint in waypoints:
        waypoint = np.asarray(waypoint, dtype=float).reshape(3)
        origin = feet[leg].copy()
        count = max(1, int(np.ceil(np.linalg.norm(waypoint - origin) / step)))
        for phase in np.linspace(0.0, 1.0, count + 1)[1:]:
            candidate = feet.copy()
            candidate[leg] = (1.0 - phase) * origin + phase * waypoint
            path.append(candidate)
        feet = path[-1]
    return path, feet


def _leg_motion_plan(
    current_joints, leg, start_feet, waypoints,
    max_joint_speed=float(np.min(JOINT_VELOCITY_LIMIT)),
):
    """生成单腿轨迹并统一添加末端保持点。"""

    feet_path, final_feet = _leg_feet_path(start_feet, leg, waypoints)
    joints, max_error = solve_feet_path(current_joints, feet_path)
    joints = np.r_[joints, joints[-1:]]
    times = [0.0]
    for previous, current in zip(joints[:-2], joints[1:-1]):
        travel = float(np.max(np.abs(current - previous)))
        times.append(times[-1] + max(0.04, travel / max_joint_speed))
    times.append(times[-1] + 0.5)
    return Plan(
        [np.eye(4)] * len(joints), joints, np.asarray(times), max_error, 0,
        "support", int(leg), final_feet,
    )


# 完整轨迹规划：逐点执行逆运动学，任意一点不可达则拒绝整条轨迹。
def plan_trajectory(lock_from_pin, current_joints, foot_anchors=None):
    """单次视觉结果生成完整固定足端关节轨迹。"""
    kinematic = GraspKinematic()
    current = np.asarray(current_joints).reshape(6, 3)
    anchors = (
        kinematic.forward_base(current)
        if foot_anchors is None else np.asarray(foot_anchors).reshape(6, 3)
    )
    pre_dock = target_body_transform(lock_from_pin)
    docked = target_body_transform(lock_from_pin, docked_pin_in_lock)
    body_path = interpolate_body(pre_dock)
    pre_dock_index = len(body_path) - 1
    body_path += interpolate_between(pre_dock, docked)[1:]
    joints, max_error = solve_body_path(
        current,
        anchors,
        body_path,
        stage_for_point=lambda index: (
            "预对准" if index <= pre_dock_index else "下降"
        ),
    )
    times = [0.0]
    for previous, current in zip(joints[:-1], joints[1:]):
        times.append(times[-1] + max(0.02, np.max(np.abs(current - previous))))
    body_path.append(body_path[-1])
    joints = np.r_[joints, joints[-1:]]
    times.append(times[-1] + 1.0)
    return Plan(
        body_path, joints, np.asarray(times), max_error,
        pre_dock_index, anchors=anchors,
    )


def support_plan(
    lock_from_pin, current_joints, anchors, error,
    lift=0.008, place_foot=None,
):
    """把不可达腿抬起并逐步外移，扩大下降阶段的可达范围。"""
    leg, target = error.leg, np.asarray(anchors).copy()
    final_body = target_body_transform(lock_from_pin, docked_pin_in_lock)
    final_foot = move_fixed_feet(target, final_body)[leg]
    radius = np.linalg.norm(final_foot[:2])
    if radius < 1e-9:
        raise IKError(leg, 0.180)
    target_radius = min(0.210, max(0.180, radius + 0.015))
    final_foot[:2] *= target_radius / radius
    target[leg, :2] = (
        (final_foot - final_body[:3, 3]) @ final_body[:3, :3]
    )[:2]
    if place_foot is not None:
        target[leg] = place_foot(target[leg], lock_from_pin, leg)
    start = np.asarray(anchors).reshape(6, 3)
    waypoints = (
        start[leg] + (0.0, 0.0, lift),
        target[leg] + (0.0, 0.0, lift),
        target[leg],
    )
    return _leg_motion_plan(current_joints, leg, start, waypoints)


def place_leg_plan(current_joints, leg, target, lift=0.008):
    """单独抬放一条腿，使摆动腿重新落到实际支撑面。"""
    kinematic = GraspKinematic()
    start = kinematic.forward_base(current_joints)
    top = max(start[leg, 2], target[2]) + lift
    waypoints = (
        np.r_[start[leg, :2], top],
        np.r_[target[:2], top],
        np.asarray(target),
    )
    return _leg_motion_plan(current_joints, leg, start, waypoints)


# ROS轨迹消息：把所有轨迹点转换为包含18个关节角和时间戳的JointTrajectory。
def trajectory_message(plan):
    message = JointTrajectory()
    message.header.stamp = (
        rospy.Time.now() if rospy.core.is_initialized() else rospy.Time(0)
    )
    message.joint_names = list(joint_names)
    for joints, time_from_start in zip(plan.joints, plan.times):
        point = JointTrajectoryPoint()
        point.positions = joints.reshape(-1).tolist()
        point.time_from_start = rospy.Duration.from_sec(float(time_from_start))
        message.points.append(point)
    return message

# 同文件内保留旧仿真使用的 body.xxx 调用方式，不产生外部模块依赖。
body = sys.modules[__name__]

# =============================================================================
# 运动规划：目标锁定、连续预对准、下降、纠偏与安全时限
# =============================================================================

@dataclass(frozen=True)
class DockPlannerConfig:
    trial_timeout: float = 180.0
    warning_time: float = 150.0
    prealign_speed: float = 0.035
    prealign_angle_speed: float = np.deg2rad(20.0)
    descent_speed: float = 0.012
    descent_angle_speed: float = np.deg2rad(10.0)
    max_joint_speed: float = float(np.min(JOINT_VELOCITY_LIMIT))
    prealign_hold: float = 0.25
    prealign_position_tolerance: float = 0.006
    prealign_tilt_tolerance: float = 2.5
    final_position_tolerance: float = 0.005
    final_tilt_tolerance: float = 2.0
    emergency_tilt: float = 12.0
    correction_limit: int = 2
    support_slip_correction: float = 0.010
    support_slip_abort: float = 0.025
    descent_translation_step: float = 0.00075


@dataclass(frozen=True)
class DeadlineStatus:
    warning: str = ""
    failure: str = ""


@dataclass(frozen=True)
class MotionUpdate:
    state: str = "executing"
    reason: str = ""
    message: str = ""


class DockMotionPlanner:
    """不含设备接口的对接路线与连续运动状态机。"""

    def __init__(self, config=None, time_scale=1.0, clock=None):
        self.config = config or DockPlannerConfig()
        self.time_scale = float(time_scale)
        if self.time_scale <= 0.0:
            raise ValueError("time_scale must be positive")
        self.clock = clock or time.monotonic
        self.session_started = 0.0
        self.warning_reported = False
        self.phase = "等待识别"
        self.phase_started = 0.0
        self.reset_route()

    def reset_route(self):
        """清除一次进入DOCK产生的路线，不影响跨重定位总计时。"""
        self.route_locked = False
        self.plans = []
        self.route_targets_world = []
        self.locked_world_from_pin = None
        self.plan_step = 0
        self.support_total = 0
        self.local_corrections = 0
        self.prealign_corrections = 0
        self.descent_corrections = 0
        self.feedback_enabled = False
        self.descent_started = False
        self.stable_samples = 0
        self.unsafe_tilt_samples = 0
        self.next_safety_elapsed = 0.0
        self.terminal_reported = False

    def begin_session(self):
        """按下X时开始唯一的180秒总计时。"""
        self.session_started = self.clock()
        self.warning_reported = False
        self.phase = "等待识别"
        self.phase_started = self.session_started
        self.reset_route()

    def cancel_session(self):
        self.session_started = 0.0
        self.phase_started = 0.0
        self.reset_route()

    def ensure_session(self):
        if self.session_started <= 0.0:
            self.begin_session()

    @property
    def elapsed(self):
        return 0.0 if self.session_started <= 0.0 else (
            self.clock() - self.session_started
        )

    def set_phase(self, phase):
        """返回上一阶段名称和耗时，便于适配层记录日志。"""
        now = self.clock()
        if phase == self.phase:
            return None
        previous = (
            None if self.phase_started <= 0.0
            else (self.phase, now - self.phase_started)
        )
        self.phase = phase
        self.phase_started = now
        return previous

    def deadline_status(self, state):
        if self.session_started <= 0.0:
            return DeadlineStatus()
        elapsed = self.elapsed
        warning = ""
        if elapsed >= self.config.warning_time and not self.warning_reported:
            self.warning_reported = True
            warning = (
                "对接总时限预警：已用{:.1f}s，当前阶段={}，剩余{:.1f}s"
                .format(
                    elapsed, self.phase,
                    max(0.0, self.config.trial_timeout - elapsed),
                )
            )
        failure = ""
        if elapsed >= self.config.trial_timeout and state not in (
            "success", "failed"
        ):
            failure = (
                "对接总流程达到{:.0f}s上限：当前阶段={}，"
                "已执行{}段，局部纠偏{}次".format(
                    self.config.trial_timeout,
                    self.phase,
                    self.plan_step,
                    self.local_corrections,
                )
            )
        return DeadlineStatus(warning, failure)

    def lock_route(self, lock_from_pin, plans, body_pose):
        """将视觉目标和各支撑落点锁定在世界坐标系中。"""
        position, rotation = body_pose
        if position is None or rotation is None:
            raise RuntimeError("锁定路线时缺少机身世界位姿")
        world_from_lock = rigid_transform(position, rotation)
        self.locked_world_from_pin = world_from_lock @ lock_from_pin
        self.plans = list(plans)
        self.route_targets_world = []
        for plan in plans:
            target = None
            if plan.kind == "support" and plan.anchors is not None:
                leg = plan.moving_leg
                target = plan.anchors[leg] @ rotation.T + position
            self.route_targets_world.append(target)
        self.route_locked = True
        self.local_corrections = 0
        self.terminal_reported = False

    def current_target(self, body_pose, fallback=None):
        if (
            self.locked_world_from_pin is None
            or body_pose[0] is None or body_pose[1] is None
        ):
            return None if fallback is None else np.asarray(fallback).copy()
        return (
            invert_transform(rigid_transform(*body_pose))
            @ self.locked_world_from_pin
        )

    def peek_route(self):
        if not self.plans:
            return None, None
        return self.plans[0], self.route_targets_world[0]

    def take_route(self):
        if not self.plans:
            return None, None
        return self.plans.pop(0), self.route_targets_world.pop(0)

    def note_plan(self, plan):
        self.plan_step += 1
        if plan.kind == "support":
            self.set_phase("支撑调整")

    def support_slip_decision(self, slip):
        if slip >= self.config.support_slip_abort:
            return "abort"
        if slip < self.config.support_slip_correction:
            return "continue"
        if self.local_corrections >= self.config.correction_limit:
            return "limit"
        self.local_corrections += 1
        return "correct"

    def allow_local_correction(self):
        if self.local_corrections >= self.config.correction_limit:
            return False
        self.local_corrections += 1
        return True

    def rebuild_support(self, template, target_world, joints, body_pose):
        if joints is None or body_pose[0] is None or body_pose[1] is None:
            raise RuntimeError("局部续接缺少真实关节或机身位姿")
        if target_world is None:
            raise RuntimeError("局部续接缺少锁定的落脚目标")
        target = (
            invert_transform(rigid_transform(*body_pose))
            @ np.r_[target_world, 1.0]
        )[:3]
        return body.place_leg_plan(
            joints, template.moving_leg, target, lift=0.015
        )

    def continuous_motion(
        self, lock_from_pin, joints, desired_pin, max_speed,
        max_angle_speed, minimum_time, translation_step,
    ):
        """生成中途不停顿的五次S曲线关节路径。"""
        target = body.target_body_transform(lock_from_pin, desired_pin)
        path = body.interpolate_body(
            target,
            translation_step=translation_step,
            rotation_step=np.deg2rad(0.25),
        )
        anchors = body.GraspKinematic().forward_base(joints)
        segment, _ = body.solve_body_path(
            joints,
            anchors,
            path,
            stage_for_point=lambda _index: "连续轨迹",
        )
        distance = float(np.linalg.norm(target[:3, 3]))
        angle = float(np.arccos(np.clip(
            (np.trace(target[:3, :3]) - 1.0) / 2.0, -1.0, 1.0
        )))
        if distance <= 0.010:
            max_speed = min(max_speed, 0.010)
        elif distance <= 0.030:
            max_speed = min(max_speed, 0.020)
        joint_travel = float(np.max(np.sum(
            np.abs(np.diff(segment, axis=0)), axis=0
        )))
        duration = max(
            minimum_time,
            1.875 * distance / max(max_speed, 1e-6),
            1.875 * angle / max(max_angle_speed, 1e-6),
            1.875 * joint_travel / self.config.max_joint_speed,
        )
        times = np.linspace(
            0.0, duration / self.time_scale, len(segment)
        )
        return path, segment, times, duration

    def prepare_route_step(
        self, lock_from_pin, joints, allow_support=True, place_foot=None,
    ):
        """从实际关节状态选择一次调足或连续对接轨迹。"""

        anchors = body.GraspKinematic().forward_base(joints)
        try:
            plan = body.plan_trajectory(lock_from_pin, joints, anchors)
        except body.IKError as error:
            if not allow_support:
                raise
            plan = body.support_plan(
                lock_from_pin, joints, anchors, error,
                place_foot=place_foot,
            )
            return plan, anchors, None
        duration = self.prepare_dock_plan(plan, lock_from_pin, joints)
        return plan, anchors, duration

    def prepare_dock_plan(self, plan, lock_from_pin, joints):
        desired = body.measured_pre_dock_pin(lock_from_pin)
        path, segment, times, duration = self.continuous_motion(
            lock_from_pin, joints, desired,
            self.config.prealign_speed,
            self.config.prealign_angle_speed,
            0.8, 0.001,
        )
        plan.body = path
        plan.joints = segment
        plan.times = times
        plan.pre_dock_index = len(segment) - 1
        plan.anchors = body.GraspKinematic().forward_base(joints)
        self.feedback_enabled = True
        self.descent_started = False
        self.prealign_corrections = 0
        self.descent_corrections = 0
        self.stable_samples = 0
        self.unsafe_tilt_samples = 0
        self.next_safety_elapsed = 0.0
        self.set_phase("连续预对准")
        return duration

    def _append_motion(
        self, plan, elapsed, lock_from_pin, joints, desired_pin,
        max_speed, max_angle_speed, minimum_time,
        translation_step, hold_time,
    ):
        path, segment, times, duration = self.continuous_motion(
            lock_from_pin, joints, desired_pin,
            max_speed, max_angle_speed, minimum_time, translation_step,
        )
        start = max(float(plan.times[-1]), float(elapsed))
        start += hold_time / self.time_scale
        plan.body += path
        plan.joints = np.r_[plan.joints, joints[None, ...], segment[1:]]
        plan.times = np.r_[plan.times, start, start + times[1:]]
        return len(segment), duration

    def advance_dock_plan(
        self, plan, elapsed, lock_from_pin, joints, errors,
    ):
        """在连续段末端决定下降、有限尾段修正或最终成功。"""
        horizontal, vertical, tilt = errors
        c = self.config
        if tilt >= c.emergency_tilt:
            return MotionUpdate(
                "failed",
                "闭环对接紧急停止：IMU倾斜{:.2f}°超过{:.1f}°".format(
                    tilt, c.emergency_tilt
                ),
            )
        if not self.descent_started:
            if horizontal > c.prealign_position_tolerance or (
                tilt > c.prealign_tilt_tolerance
            ):
                if self.prealign_corrections >= c.correction_limit:
                    return MotionUpdate(
                        "failed",
                        "连续预对准尾段修正已达{}次：水平{:.1f}mm，"
                        "倾斜{:.2f}°".format(
                            c.correction_limit, horizontal * 1000.0, tilt
                        ),
                    )
                self.prealign_corrections += 1
                desired = body.measured_pre_dock_pin(lock_from_pin)
                count, duration = self._append_motion(
                    plan, elapsed, lock_from_pin, joints, desired,
                    c.prealign_speed, c.prealign_angle_speed,
                    0.5, 0.001, 0.05,
                )
                return MotionUpdate(
                    message=(
                        "连续预对准尾段修正 {}/{}：{}点，预计{:.2f}s"
                        .format(
                            self.prealign_corrections,
                            c.correction_limit, count, duration,
                        )
                    )
                )
            self.descent_started = True
            self.set_phase("连续下降")
            count, duration = self._append_motion(
                plan, elapsed, lock_from_pin, joints, np.zeros(3),
                c.descent_speed, c.descent_angle_speed,
                1.0, c.descent_translation_step, c.prealign_hold,
            )
            return MotionUpdate(message=(
                "连续预对准完成：水平{:.1f}mm，倾斜{:.2f}°；"
                "保持{:.2f}s后连续下降，{}点，预计{:.2f}s".format(
                    horizontal * 1000.0, tilt, c.prealign_hold,
                    count, duration,
                )
            ))

        success = (
            horizontal <= c.final_position_tolerance
            and vertical <= c.final_position_tolerance
            and tilt <= c.final_tilt_tolerance
        )
        if success:
            self.stable_samples += 1
            if self.stable_samples >= 2:
                return MotionUpdate(
                    "success",
                    "最终误差：水平{:.1f}mm，垂直{:.1f}mm，"
                    "倾斜{:.2f}°".format(
                        horizontal * 1000.0,
                        vertical * 1000.0, tilt,
                    ),
                )
            start = max(float(plan.times[-1]), float(elapsed))
            plan.joints = np.r_[plan.joints, joints[None, ...]]
            plan.times = np.r_[
                plan.times, start + 0.25 / self.time_scale
            ]
            return MotionUpdate()
        self.stable_samples = 0
        if self.descent_corrections >= c.correction_limit:
            return MotionUpdate(
                "failed",
                "连续下降尾段修正已达{}次：水平{:.1f}mm，"
                "垂直{:.1f}mm，倾斜{:.2f}°".format(
                    c.correction_limit, horizontal * 1000.0,
                    vertical * 1000.0, tilt,
                ),
            )
        self.descent_corrections += 1
        count, duration = self._append_motion(
            plan, elapsed, lock_from_pin, joints, np.zeros(3),
            0.008, np.deg2rad(6.0), 0.6,
            c.descent_translation_step, 0.05,
        )
        return MotionUpdate(message=(
            "下降尾段修正 {}/{}：{}点，预计{:.2f}s；"
            "当前水平{:.1f}mm，垂直{:.1f}mm，倾斜{:.2f}°".format(
                self.descent_corrections, c.correction_limit,
                count, duration, horizontal * 1000.0,
                vertical * 1000.0, tilt,
            )
        ))

    @staticmethod
    def pose_errors(lock_from_pin):
        """返回插销相对锁紧机构的水平、垂直和倾斜误差。"""

        pose = np.asarray(lock_from_pin, dtype=float).reshape(4, 4)
        return (
            float(np.linalg.norm(pose[:2, 3])),
            float(abs(pose[2, 3])),
            float(np.rad2deg(np.arccos(np.clip(
                pose[2, 2], -1.0, 1.0
            )))),
        )

    def update_dock_plan(self, plan, elapsed, lock_from_pin, joints):
        """统一执行连续轨迹的安全检查和段末闭环推进。"""

        errors = self.pose_errors(lock_from_pin)
        reason = self.safety_check(plan, elapsed, joints, errors[2])
        if reason:
            return MotionUpdate("failed", reason)
        if elapsed < float(plan.times[-1]):
            return MotionUpdate()
        return self.advance_dock_plan(
            plan, elapsed, lock_from_pin, joints, errors
        )

    def safety_check(self, plan, elapsed, joints, tilt):
        if elapsed < self.next_safety_elapsed:
            return ""
        self.next_safety_elapsed = elapsed + 0.1
        self.unsafe_tilt_samples = (
            self.unsafe_tilt_samples + 1
            if tilt >= self.config.emergency_tilt else 0
        )
        if self.unsafe_tilt_samples < 3:
            return ""
        keep = max(1, int(np.searchsorted(
            plan.times, elapsed, side="right"
        )))
        plan.joints = plan.joints[:keep]
        plan.times = plan.times[:keep]
        if joints is not None:
            plan.joints = np.r_[plan.joints, joints[None, ...]]
            plan.times = np.r_[
                plan.times,
                max(float(plan.times[-1]), float(elapsed)) + 0.1,
            ]
        return (
            "连续运动紧急停止：IMU倾斜连续超过{:.1f}°，当前{:.2f}°"
            .format(self.config.emergency_tilt, tilt)
        )

# =============================================================================
# 总状态机：连接感知、规划与设备执行层
# =============================================================================

@dataclass(frozen=True)
class DockRobotState:
    """设备层每周期反馈；位置单位m，关节和旋转单位rad。"""

    joints: object = None
    body_position: object = None
    body_rotation: object = None
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
    """连接感知、运动规划和设备层，不包含具体硬件接口。"""

    IDLE = "idle"
    WAITING_TAG = "waiting_tag"
    SUPPORT = "support_adjustment"
    PREALIGN = "prealign"
    DESCENT = "descent"
    ALIGNED = "aligned"
    SUCCESS = "success"
    FAILED = "failed"
    TERMINAL_STATES = (SUCCESS, FAILED)

    def __init__(
        self, controller, perception=None, allow_inference=False,
        joint_reader=None, planner_config=None, time_scale=1.0, clock=None,
        require_lock_confirmation=False, trajectory_publisher=None,
        status_publisher=None, subscribe_joint_state=True,
        publish_trajectory=True,
    ):
        self.controller = controller
        self.allow_inference = bool(allow_inference)
        self.require_lock_confirmation = bool(require_lock_confirmation)
        self.perception = perception or DockPerception(
            allow_inference=self.allow_inference
        )
        # 独立ROS入口可读/joint_states；正式实机循环应直接注入反馈。
        if joint_reader is not None:
            self.joints = joint_reader
        elif subscribe_joint_state:
            self.joints = body.JointReader()
        else:
            self.joints = type("InjectedJointState", (), {"values": None})()
        self.motion_planner = DockMotionPlanner(
            planner_config or DockPlannerConfig(), time_scale, clock
        )
        self.trajectory_publisher = trajectory_publisher
        if self.trajectory_publisher is None and publish_trajectory:
            self.trajectory_publisher = rospy.Publisher(
                "/dock_mode6/joint_trajectory", JointTrajectory, queue_size=1
            )
        self.status_publisher = status_publisher or rospy.Publisher(
            "/dock_mode6/status", String, queue_size=1, latch=True
        )
        self.active = False
        self.foot_anchors_base = None
        self._clear(self.IDLE)

    @property
    def clock(self):
        return self.motion_planner.clock

    def _clear(self, state, reason=""):
        self.state, self.reason = state, reason
        self.plan = self.pin_pose = None
        self.adjustments = 0
        self.plan_started = 0.0
        self._support_last_joints = None
        self._support_stable_samples = 0

    def configure_motion_planner(self, time_scale=1.0, config=None):
        self.motion_planner = DockMotionPlanner(
            config or self.motion_planner.config,
            time_scale,
            self.motion_planner.clock,
        )
        return self.motion_planner

    def set_joint_state(self, joints):
        self.joints.values = np.asarray(
            joints, dtype=np.float64
        ).reshape(6, 3).copy()

    @staticmethod
    def _field(source, name):
        if source is None:
            return None
        return (
            source.get(name)
            if isinstance(source, Mapping) else getattr(source, name, None)
        )

    def _ingest_joints(self, robot_state):
        joints = self._field(robot_state, "joints")
        if joints is not None:
            self.set_joint_state(joints)

    def _body_pose(self, robot_state):
        position = self._field(robot_state, "body_position")
        rotation = self._field(robot_state, "body_rotation")
        if position is None or rotation is None:
            return None
        try:
            pose = (
                np.asarray(position, dtype=float).reshape(3),
                np.asarray(rotation, dtype=float).reshape(3, 3),
            )
        except (TypeError, ValueError):
            return None
        return pose if all(np.isfinite(value).all() for value in pose) else None

    def _feedback_pose(self, robot_state):
        raw = self._field(robot_state, "lock_from_pin")
        body_pose = self._body_pose(robot_state)
        if raw is None and body_pose and self.motion_planner.route_locked:
            raw = self.motion_planner.current_target(body_pose)
        if raw is None:
            observed = self.perception.latest()
            raw = (
                observed.lock_from_pin
                if getattr(observed, "valid", False) else None
            )
        try:
            pose = np.asarray(raw, dtype=float).reshape(4, 4)
        except (TypeError, ValueError):
            return None
        return pose.copy() if np.isfinite(pose).all() else None

    def enter(self, foot_positions_base):
        self.foot_anchors_base = np.asarray(
            foot_positions_base, dtype=float
        ).reshape(6, 3).copy()
        self.active = True
        self.perception.reset()
        if self.motion_planner.session_started <= 0.0:
            self.motion_planner.begin_session()
        else:
            # 自主重定位后重新进入时仍计入同一个总时限。
            self.motion_planner.reset_route()
            self.motion_planner.set_phase("等待识别")
        self._clear(
            self.WAITING_TAG, "waiting for a complete decoded tag id"
        )
        self.status_publisher.publish(self.reason)

    def _set_state(self, state, reason):
        changed = (state, reason) != (self.state, self.reason)
        self.state, self.reason = state, reason
        if changed and reason:
            self.status_publisher.publish(reason)

    def _not_ready(self, reason):
        self._set_state(self.WAITING_TAG, reason)
        return False

    def _fail(self, reason):
        """旧仿真适配器将规划失败视为可重新定位的等待状态。"""

        return self._not_ready(reason)

    def _terminal_failure(self, reason):
        self._set_state(self.FAILED, reason)
        return False

    def fail_execution(self, reason):
        """由公共执行层拒绝目标时明确结束本次对接。"""

        self._terminal_failure(reason)

    def _publish_plan(self, elapsed=0.0):
        if self.trajectory_publisher is None:
            return
        plan = self.plan if elapsed <= 0.0 else body.remaining_plan(
            self.plan, elapsed, self.joints.values
        )
        self.trajectory_publisher.publish(body.trajectory_message(plan))

    def _activate_plan(self, lock_from_pin):
        if self.joints.values is None:
            return self._terminal_failure("missing joint state")
        try:
            plan, anchors, duration = self.motion_planner.prepare_route_step(
                lock_from_pin,
                self.joints.values,
                allow_support=self.adjustments < 6,
                place_foot=getattr(self, "place_support_foot", None),
            )
        except body.IKError as error:
            reason = "六次调足后仍不可达" if self.adjustments >= 6 else str(error)
            return self._terminal_failure(reason)

        if plan.kind == "support":
            self.adjustments += 1
            self._support_last_joints = None
            self._support_stable_samples = 0
            self.motion_planner.support_total = self.adjustments
            state, reason = self.SUPPORT, "支撑调整 {}/6：移动{}腿".format(
                self.adjustments, body.LEG_NAMES[plan.moving_leg]
            )
        else:
            self.foot_anchors_base = anchors.copy()
            state, reason = self.PREALIGN, "开始连续预对准：{}点，预计{:.2f}s".format(
                len(plan.joints), duration
            )

        self.plan = plan
        self.plan_started = self.clock()
        self.motion_planner.note_plan(plan)
        self._set_state(state, reason)
        self._publish_plan()
        return True

    def start(self, observed=None, current_joints=None, robot_state=None):
        """冻结至少一个完整ID结果，并启动调足或连续对接。"""

        if not self.active:
            raise RuntimeError("enter() must be called before start()")
        if current_joints is not None:
            self.set_joint_state(current_joints)
        self._ingest_joints(robot_state)
        observed = self.perception.latest() if observed is None else observed
        if not observed.valid:
            return self._not_ready(observed.reason)
        if not observed.decoded_ids:
            return self._not_ready("waiting for a complete decoded tag id")
        if observed.inferred_ids and not self.allow_inference:
            return self._not_ready("inferred tag ids are disabled")
        if self.joints.values is None:
            return self._not_ready("missing joint state")

        try:
            pose = np.asarray(
                observed.lock_from_pin, dtype=float
            ).reshape(4, 4)
        except (TypeError, ValueError):
            return self._not_ready("invalid pin pose")
        if not np.isfinite(pose).all():
            return self._not_ready("invalid pin pose")
        if body.measured_pre_dock_pin(pose)[2] >= 0.0:
            return self._not_ready("卡紧机构中心已经低于插销")

        self.pin_pose = pose.copy()
        self.motion_planner.ensure_session()
        body_pose = self._body_pose(robot_state)
        if body_pose:
            self.motion_planner.lock_route(pose, (), body_pose)
        return self._activate_plan(pose)

    def _finish_alignment(self, reason, confirmed):
        if not self.require_lock_confirmation:
            self._set_state(self.SUCCESS, reason)
        elif confirmed is True:
            self._set_state(self.SUCCESS, "锁紧确认完成；{}".format(reason))
        elif confirmed is False:
            self._terminal_failure("已完成插入，但锁紧机构确认失败")
        else:
            self._set_state(
                self.ALIGNED, "已完成插入，等待锁紧机构确认；{}".format(reason)
            )

    def _advance_dock(self, robot_state, elapsed):
        pose = self._feedback_pose(robot_state)
        if pose is None:
            return self._terminal_failure(
                "闭环对接失败：缺少实时插销位姿或机身位姿"
            )
        segment_finished = elapsed >= float(self.plan.times[-1])
        try:
            update = self.motion_planner.update_dock_plan(
                self.plan, elapsed, pose, self.joints.values
            )
        except body.IKError as error:
            return self._terminal_failure("连续轨迹IK不可达：{}".format(error))
        if update.state == self.FAILED:
            return self._terminal_failure(update.reason)
        if update.state == self.SUCCESS:
            self._finish_alignment(
                update.reason, self._field(robot_state, "lock_confirmed")
            )
        elif segment_finished:
            state = self.DESCENT if self.motion_planner.descent_started else self.PREALIGN
            self._set_state(state, update.message or self.reason)
            self._publish_plan(elapsed)
        return True

    def _result(self, target=None):
        if target is None and self.state in (
            self.ALIGNED, self.SUCCESS, self.FAILED
        ) and self.joints.values is not None:
            target = self.joints.values.copy()
        return DockResult(
            self.foot_anchors_base, self.active,
            self.state == self.SUCCESS, self.state == self.FAILED,
            self.state == self.ALIGNED, self.reason, target, self.state,
        )

    def update(self, robot_state=None, target_accepted=True, reject_reason=""):
        """推进一个控制周期并返回当前18关节目标。"""

        if not self.active:
            raise RuntimeError("enter() must be called before update()")
        self._ingest_joints(robot_state)

        deadline = self.motion_planner.deadline_status(self.state)
        if deadline.warning:
            self.status_publisher.publish(deadline.warning)
        if deadline.failure:
            self._terminal_failure(deadline.failure)
        if self.state in self.TERMINAL_STATES:
            return self._result()

        if self.state == self.ALIGNED:
            confirmed = self._field(robot_state, "lock_confirmed")
            if confirmed is True:
                self._set_state(self.SUCCESS, "锁紧机构确认完成")
            elif confirmed is False:
                self._terminal_failure("已完成插入，但锁紧机构确认失败")
            return self._result()
        if self.state == self.WAITING_TAG:
            self.start(robot_state=robot_state)
            if self.state == self.WAITING_TAG:
                return self._result()
        if not target_accepted:
            self._terminal_failure(reject_reason or "执行层拒绝对接关节目标")
            return self._result()
        if self.plan is None or self.joints.values is None:
            self._terminal_failure("对接执行缺少轨迹或关节反馈")
            return self._result()

        elapsed = max(0.0, self.clock() - self.plan_started)
        target = body.sample_plan(self.plan, elapsed)
        if self.state == self.SUPPORT and elapsed >= float(self.plan.times[-1]):
            current = self.joints.values
            stable = (
                self._support_last_joints is not None
                and np.max(np.abs(current - self._support_last_joints))
                <= np.deg2rad(0.3)
            )
            self._support_stable_samples = (
                self._support_stable_samples + 1 if stable else 0
            )
            self._support_last_joints = current.copy()
            target = self.plan.joints[-1].copy()
            if self._support_stable_samples >= 8:
                pose = self._feedback_pose(robot_state)
                if pose is None:
                    self._terminal_failure("调足后缺少新的插销位姿")
                else:
                    self._activate_plan(pose)
                    target = body.sample_plan(self.plan, 0.0)
        elif self.state in (self.PREALIGN, self.DESCENT):
            self._advance_dock(robot_state, elapsed)

        if self.state in (self.ALIGNED, self.SUCCESS, self.FAILED):
            target = self.joints.values.copy()
        return self._result(target)

    def descent_has_started(self):
        return self.motion_planner.descent_started

    def exit(self):
        self.motion_planner.cancel_session()
        self.active = False
        self.foot_anchors_base = None
        self._clear(self.IDLE)


__all__ = (
    "DockMode", "DockResult", "DockRobotState", "DockPerception",
    "TAG_IDS", "TAG_SIZE", "LOCK_FROM_CAMERA", "PIN_FROM_TAG",
    "REAL_CALIBRATED", "load_dock_system",
)
