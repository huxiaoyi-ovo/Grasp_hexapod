"""基于黑白同心圆的实机视觉对接模式。

本文件包含完整感知与动作状态机，不依赖原 dock_mode.py。改名为
dock_mode.py 后，run_real 可沿用原有导入接口。足端目标仍交给公共
GraspController 执行 DLS 和舵机控制。默认 SUCCESS 表示收腿完成，
由 run_real 的行为树请求随后执行夹紧；要求锁紧确认时需外部夹紧执行器。
"""

from dataclasses import dataclass
from threading import Lock

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image

from typing import Mapping
import yaml
from kinematics import JOINT_VELOCITY_LIMIT
from utils import package_config_path, transform_points


TAG_IDS = (0, 1, 2, 3)


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



# SolidWorks目标尺寸。这里的环宽是相邻外径之差，单边径向宽度为其一半。
PIN_DIAMETER_M = 0.0435
RING_OUTER_DIAMETERS_M = np.array(
    (0.060, 0.080, 0.092, 0.112, 0.126, 0.146, 0.162, 0.182),
    dtype=np.float64,
)
RING_COLORS = ("white", "black", "white", "black",
               "white", "black", "white", "black")
OUTER_RING_DIAMETER_M = float(RING_OUTER_DIAMETERS_M[-1])
RING_DIAMETER_RATIOS = RING_OUTER_DIAMETERS_M / OUTER_RING_DIAMETER_M

# dock_camera.yaml中去畸变图像的投影内参。
DEFAULT_CAMERA_MATRIX = np.array(
    (
        (599.885800, 0.0, 675.797400),
        (0.0, 624.318580, 364.069370),
        (0.0, 0.0, 1.0),
    ),
    dtype=np.float64,
)


@dataclass(frozen=True)
class RingDetection:
    """一帧图像中的同心圆检测结果，像素坐标采用去畸变图像坐标。"""

    valid: bool = False
    center_px: object = None
    major_diameter_px: float = 0.0
    minor_diameter_px: float = 0.0
    major_angle_rad: float = 0.0
    matched_boundaries: int = 0
    mean_ratio_error: float = float("inf")
    reason: str = "no ring detection"


@dataclass(frozen=True)
class PerceptionResult:
    """圆环中心（即插销轴线）相对卡紧机构的位姿。"""

    valid: bool = False
    lock_from_pin: object = None
    stamp: object = None
    detection: object = None
    reason: str = "no perception result"


def _gray_image(image):
    image = np.asarray(image)
    if image.ndim == 2:
        return image.astype(np.uint8, copy=False)
    if image.ndim == 3 and image.shape[2] == 3:
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if image.ndim == 3 and image.shape[2] == 4:
        return cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
    raise ValueError("ring image must be mono, BGR, or BGRA")


def _circle_candidates(edges):
    """从完整圆或画面边缘处的局部圆弧生成圆模型。"""

    height, width = edges.shape
    candidates = []
    for contour in cv2.findContours(
        edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE
    )[-2]:
        points = contour[:, 0].astype(np.float64)
        inside = (
            (points[:, 0] > 3) & (points[:, 0] < width - 4)
            & (points[:, 1] > 3) & (points[:, 1] < height - 4)
        )
        indices = np.flatnonzero(inside)
        border_arcs = np.split(
            points[indices], np.flatnonzero(np.diff(indices) > 1) + 1
        )
        arcs = []
        for arc in border_arcs:
            if len(arc) < 30:
                continue
            step = max(3, min(10, len(arc) // 30))
            incoming = arc - np.roll(arc, step, axis=0)
            outgoing = np.roll(arc, -step, axis=0) - arc
            cosine = np.sum(incoming * outgoing, axis=1) / (
                np.linalg.norm(incoming, axis=1)
                * np.linalg.norm(outgoing, axis=1) + 1e-9
            )
            corners = np.flatnonzero(
                (cosine < 0.45)
                & (np.arange(len(arc)) > step)
                & (np.arange(len(arc)) < len(arc) - step)
            )
            selected = []
            for index in corners[np.argsort(cosine[corners])]:
                if all(abs(index - old) > 2 * step for old in selected):
                    selected.append(int(index))
            bounds = [0] + sorted(selected) + [len(arc)]
            arcs.extend(arc[start + (step if start else 0):
                            stop - (step if stop < len(arc) else 0)]
                        for start, stop in zip(bounds, bounds[1:]))

        for arc in arcs:
            if len(arc) < 30:
                continue
            x, y = arc.T
            a, b, c = np.linalg.lstsq(
                np.column_stack((x, y, np.ones(len(arc)))),
                -(x * x + y * y), rcond=None,
            )[0]
            center = np.array((-0.5 * a, -0.5 * b))
            radius_squared = float(center @ center - c)
            if radius_squared <= 0.0:
                continue
            radius = np.sqrt(radius_squared)
            radial = np.hypot(x - center[0], y - center[1])
            residual = float(np.median(np.abs(radial - radius)))
            phase = np.unwrap(np.arctan2(y - center[1], x - center[0]))
            coverage = float(np.ptp(phase))
            if (20.0 < radius < 8.0 * max(edges.shape)
                    and residual <= 3.0 and coverage >= 0.08):
                candidates.append((center, float(radius), residual, coverage))

    unique = []
    for candidate in sorted(candidates, key=lambda item: (-item[3], item[2])):
        if any(np.linalg.norm(candidate[0] - old[0]) < 4.0
               and abs(candidate[1] - old[1]) < 0.015 * old[1]
               for old in unique):
            continue
        unique.append(candidate)
    return unique[:32]


def _model_support(distance, gray, threshold, center, outer_radius):
    """用全部可见边缘和黑白顺序共同验证一个同心圆模型。"""

    height, width = gray.shape
    theta = np.linspace(0.0, 2.0 * np.pi, 360, endpoint=False)
    cos_theta, sin_theta = np.cos(theta), np.sin(theta)
    inner = np.r_[PIN_DIAMETER_M / OUTER_RING_DIAMETER_M,
                  RING_DIAMETER_RATIOS[:-1]]
    strong = color_count = 0
    support_sum = 0.0
    for index, ratio in enumerate(RING_DIAMETER_RATIOS):
        x = np.rint(center[0] + outer_radius * ratio * cos_theta).astype(int)
        y = np.rint(center[1] + outer_radius * ratio * sin_theta).astype(int)
        visible = (x > 0) & (x < width - 1) & (y > 0) & (y < height - 1)
        if np.count_nonzero(visible) >= 8:
            support = float(np.mean(distance[y[visible], x[visible]] <= 3.0))
            support_sum += support
            strong += support >= 0.28

        middle = 0.5 * (inner[index] + ratio)
        x = np.rint(center[0] + outer_radius * middle * cos_theta).astype(int)
        y = np.rint(center[1] + outer_radius * middle * sin_theta).astype(int)
        visible = (x > 0) & (x < width - 1) & (y > 0) & (y < height - 1)
        if np.count_nonzero(visible) >= 8:
            white = gray[y[visible], x[visible]] > threshold
            expected = RING_COLORS[index] == "white"
            color_count += np.mean(white == expected) >= 0.65
    return int(strong), support_sum, int(color_count)


def detect_concentric_rings(image):
    """以已知半径和黑白顺序联合识别完整圆环及局部圆弧。"""

    gray = _gray_image(image)
    if min(gray.shape) < 64:
        return RingDetection(reason="image is too small")
    blurred = cv2.GaussianBlur(gray, (5, 5), 0.0)
    threshold, _ = cv2.threshold(
        blurred, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU
    )
    edges = cv2.Canny(blurred, max(20, int(0.35 * threshold)),
                      max(60, int(0.90 * threshold)))
    candidates = _circle_candidates(edges)
    if not candidates:
        return RingDetection(reason="no usable ring arcs")

    distance = cv2.distanceTransform(255 - edges, cv2.DIST_L2, 3)
    best = None
    for center, radius, residual, _coverage in candidates:
        for ratio in RING_DIAMETER_RATIOS:
            outer_radius = radius / ratio
            if 2.0 * outer_radius < 0.35 * min(gray.shape):
                continue
            strong, support, colors = _model_support(
                distance, blurred, threshold, center, outer_radius
            )
            score = (strong, colors, support, -residual)
            if strong >= 3 and colors >= 3 and (best is None or score > best[0]):
                best = (score, center, outer_radius, strong, support)

    if best is None:
        return RingDetection(reason="fewer than three credible ring boundaries")
    _, center, outer_radius, matched, support = best
    diameter = 2.0 * outer_radius
    return RingDetection(
        valid=True, center_px=center, major_diameter_px=diameter,
        minor_diameter_px=diameter, major_angle_rad=0.0,
        matched_boundaries=matched,
        mean_ratio_error=max(0.0, 1.0 - support / matched),
        reason="{} ring boundaries".format(matched),
    )


def camera_from_ring(detection, camera_matrix):
    """由外环已知直径计算圆环中心在相机光学坐标系中的位置。"""

    if not detection.valid:
        raise ValueError("valid ring detection is required")
    camera_matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    fx, fy = float(camera_matrix[0, 0]), float(camera_matrix[1, 1])
    cx, cy = float(camera_matrix[0, 2]), float(camera_matrix[1, 2])
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera focal lengths must be positive")

    # 弱透视下圆的长轴基本不受平面倾斜压缩，用长轴估距更直接。
    direction = detection.major_angle_rad
    focal = float(np.hypot(fx * np.cos(direction), fy * np.sin(direction)))
    z = focal * OUTER_RING_DIAMETER_M / detection.major_diameter_px
    u, v = np.asarray(detection.center_px, dtype=np.float64).reshape(2)
    pose = np.eye(4, dtype=np.float64)
    pose[:3, 3] = ((u - cx) * z / fx, (v - cy) * z / fy, z)
    return pose


class RingPerception:
    """订阅底部相机并保存最新的同心圆位姿。"""

    def __init__(
        self,
        image_topic=None,
        camera_info_topic=None,
        max_age=0.35,
        camera_matrix=None,
        lock_from_camera=None,
        dock_system_path=None,
        image_is_rectified=None,
        **_compatibility,
    ):
        dock_system = load_dock_system(dock_system_path)
        image_topic = image_topic or rospy.get_param(
            "~dock_image_topic", "/dock_camera/image_rect_color"
        )
        camera_info_topic = camera_info_topic or rospy.get_param(
            "~dock_camera_info_topic", "/dock_camera/camera_info"
        )
        self.image_is_rectified = bool(
            rospy.get_param("~dock_image_is_rectified", "image_rect" in image_topic)
            if image_is_rectified is None else image_is_rectified
        )
        self.max_age = float(max_age)
        if not np.isfinite(self.max_age) or self.max_age <= 0.0:
            raise ValueError("dock perception max_age must be finite and positive")
        self.camera_matrix = np.asarray(
            DEFAULT_CAMERA_MATRIX if camera_matrix is None else camera_matrix,
            dtype=np.float64,
        ).reshape(3, 3).copy()
        self.lock_from_camera = np.asarray(
            dock_system["lock_from_camera"]
            if lock_from_camera is None else lock_from_camera,
            dtype=np.float64,
        ).reshape(4, 4).copy()
        self.bridge = CvBridge()
        self.lock = Lock()
        self.raw_calibration = None
        self.calibration_size = None
        self.camera_info_received = camera_matrix is not None and self.image_is_rectified
        self.result = PerceptionResult()
        self.image_subscriber = rospy.Subscriber(
            image_topic, Image, self._image_callback, queue_size=1
        )
        self.info_subscriber = rospy.Subscriber(
            camera_info_topic, CameraInfo, self._camera_info_callback, queue_size=1
        )

    def _camera_info_callback(self, message):
        projection = np.asarray(message.P, dtype=np.float64).reshape(3, 4)
        intrinsic = projection[:, :3]
        if intrinsic[0, 0] <= 0.0 or intrinsic[1, 1] <= 0.0:
            intrinsic = np.asarray(message.K, dtype=np.float64).reshape(3, 3)
        if (np.isfinite(intrinsic).all()
                and intrinsic[0, 0] > 0.0 and intrinsic[1, 1] > 0.0):
            with self.lock:
                self.camera_matrix = intrinsic.copy()
                self.calibration_size = (message.width, message.height)
                self.camera_info_received = True
                raw_k = np.asarray(message.K, dtype=np.float64).reshape(3, 3)
                distortion = np.asarray(message.D, dtype=np.float64)
                rectification = np.asarray(message.R, dtype=np.float64).reshape(3, 3)
                self.raw_calibration = None
                if (message.distortion_model in ("plumb_bob", "rational_polynomial")
                        and np.isfinite(raw_k).all() and np.isfinite(distortion).all()
                        and np.isfinite(rectification).all()
                        and raw_k[0, 0] > 0.0 and raw_k[1, 1] > 0.0):
                    self.raw_calibration = (raw_k.copy(), distortion.copy(),
                                            rectification.copy())

    def _image_callback(self, message):
        try:
            gray = self.bridge.imgmsg_to_cv2(message, desired_encoding="mono8")
            with self.lock:
                camera_matrix = self.camera_matrix.copy()
                raw_calibration = self.raw_calibration
                info_received = self.camera_info_received
                calibration_size = self.calibration_size
            if not info_received:
                raise ValueError("waiting for valid camera_info")
            image_size = (gray.shape[1], gray.shape[0])
            if calibration_size is not None and image_size != calibration_size:
                raise ValueError("image size does not match camera_info")
            if not self.image_is_rectified:
                if raw_calibration is None:
                    raise ValueError("raw image requires valid supported camera calibration")
                raw_k, distortion, rectification = raw_calibration
                map_x, map_y = cv2.initUndistortRectifyMap(
                    raw_k, distortion, rectification, camera_matrix,
                    image_size, cv2.CV_32FC1,
                )
                gray = cv2.remap(gray, map_x, map_y, cv2.INTER_LINEAR)
            detection = detect_concentric_rings(gray)
            camera_pose = camera_from_ring(detection, camera_matrix) if detection.valid else None
        except (CvBridgeError, TypeError, ValueError, cv2.error) as error:
            detection = RingDetection(reason=str(error))

        with self.lock:
            if not detection.valid:
                self.result = PerceptionResult(
                    stamp=message.header.stamp,
                    detection=detection,
                    reason=detection.reason,
                )
                return
            lock_pose = self.lock_from_camera @ camera_pose
            self.result = PerceptionResult(
                valid=True,
                lock_from_pin=lock_pose,
                stamp=message.header.stamp,
                detection=detection,
                reason=detection.reason,
            )

    def reset(self):
        with self.lock:
            self.result = PerceptionResult()

    def latest(self):
        with self.lock:
            result = self.result
        if result.stamp is None:
            return result
        age = (rospy.Time.now() - result.stamp).to_sec()
        if age < 0.0 or age > self.max_age:
            return PerceptionResult(stamp=result.stamp, reason="ring image is stale")
        return result


# run_real切换导入模块后可继续使用原有的DockPerception名称。
DockPerception = RingPerception


class DockMode:
    """同心圆感知与完整对接动作；可直接作为 dock_mode 模块使用。"""
    IDLE = "idle"
    CLIMB_TERMINAL_ENTRY = "climb_terminal_entry"
    BODY_RAISE = "body_raise"
    SEARCHING_TAG = "searching_tag"
    WAITING_RING = "waiting_ring"
    WAITING_TAG = WAITING_RING
    PREALIGN = "prealign"
    PRE_DESCENT_SETTLE = "pre_descent_settle"
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
        BODY_RAISE: "对接初始姿态抬升",
        SEARCHING_TAG: "平面扫描AprilTag",
        WAITING_TAG: "等待同心圆",
        PREALIGN: "视觉预对准",
        PRE_DESCENT_SETTLE: "下降前稳定",
        DESCENT: "机械导向下降",
        SIT_SETTLE: "下坐稳定等待",
        LEG_LIFT: "腿部腾空",
        ALIGNED: "等待锁紧确认",
        SUCCESS: "对接成功",
        FAILED: "对接失败",
    }

    ENTRY_TRACKING_TOLERANCE = np.deg2rad(2.0)
    # 只用于从视觉调整切换到机械导向下降，不作为成功或失败限制。
    PREALIGN_POSITION_REFERENCE = 0.004
    LINEAR_SPEED_M_S = 0.050
    BODY_RAISE_HEIGHT_M = 0.040
    TAG_SEARCH_RADIUS_M = 0.020
    TAG_SEARCH_SPEED_M_S = 0.020
    PRE_DESCENT_SETTLE_DURATION_S = 0.5
    LEG_LIFT_HEIGHT_M = 0.060
    LEG_LIFT_SPEED_M_S = 0.050
    LEG_LIFT_LEVEL_TOLERANCE_M = 0.003
    SIT_SETTLE_DURATION_S = 0.5
    # None表示在进入下降时使用TF的垂直距离；也可在本文件中改为固定米数。
    DESCENT_DISTANCE_M = None

    HOLE_DEPTH_M = 0.043

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
        self.body_raise_start_feet = None
        self.body_raise_progress = 0.0
        self.search_anchor_feet = None
        self.search_body_offset = np.zeros(2)
        self.search_angle = -0.5 * np.pi
        self.search_on_circle = False
        self.descent_total = 0.0
        self.descent_remaining = 0.0
        self.descent_duration = 0.0
        self.pre_descent_elapsed = 0.0
        self.pre_descent_feet = None
        self.sit_settle_elapsed = 0.0
        self.sit_settle_feet = None
        self.leg_lift_start_feet = None
        self.leg_lift_progress = 0.0
        self.cached_pose = None
        self.cached_ids = ()
        self.last_visual_target = None

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
        self.body_raise_start_feet = None
        self.body_raise_progress = 0.0
        self.search_anchor_feet = None
        self.search_body_offset[:] = 0.0
        self.search_angle = -0.5 * np.pi
        self.search_on_circle = False
        self.descent_total = 0.0
        self.descent_remaining = 0.0
        self.descent_duration = 0.0
        self.pre_descent_elapsed = 0.0
        self.pre_descent_feet = None
        self.sit_settle_elapsed = 0.0
        self.sit_settle_feet = None
        self.leg_lift_start_feet = None
        self.leg_lift_progress = 0.0
        self.cached_pose = None
        self.cached_ids = ()
        self.last_visual_target = None
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
        self.body_raise_start_feet = None
        self.body_raise_progress = 0.0
        self.search_anchor_feet = None
        self.search_body_offset[:] = 0.0
        self.search_angle = -0.5 * np.pi
        self.search_on_circle = False
        self.descent_total = 0.0
        self.descent_remaining = 0.0
        self.descent_duration = 0.0
        self.pre_descent_elapsed = 0.0
        self.pre_descent_feet = None
        self.sit_settle_elapsed = 0.0
        self.sit_settle_feet = None
        self.leg_lift_start_feet = None
        self.leg_lift_progress = 0.0
        self.cached_pose = None
        self.cached_ids = ()
        self.last_visual_target = None
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
            self.body_raise_start_feet = self._synced_feet(current).copy()
            self.body_raise_progress = 0.0
            self._set_state(
                self.BODY_RAISE,
                "攀爬结束姿态指令已完成，关节误差{:.2f}deg（{}）；开始足端向下移动40mm".format(
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

    def _body_raise_step(self, current):
        if self.body_raise_start_feet is None:
            self.body_raise_start_feet = self._synced_feet(current).copy()
        self.body_raise_progress = min(
            self.BODY_RAISE_HEIGHT_M,
            self.body_raise_progress + self.linear_speed_m_s * self.update_dt,
        )
        feet = self.body_raise_start_feet.copy()
        feet[:, 2] -= self.body_raise_progress
        if self.body_raise_progress >= self.BODY_RAISE_HEIGHT_M:
            self.last_visual_target = feet.copy()
            self._set_state(self.WAITING_RING, "初始抬升完成，等待同心圆")
        else:
            self._set_state(
                self.BODY_RAISE,
                "六足同步向下：{:.1f}/40.0mm".format(
                    self.body_raise_progress * 1000.0
                ),
            )
        return self._result(feet=feet)

    def _visual_step(self, current, pose):
        correction = -pose[:2, 3]
        distance = float(np.linalg.norm(correction))
        if distance > self.linear_speed_m_s * self.update_dt:
            correction *= self.linear_speed_m_s * self.update_dt / distance
        increment = transform((correction[0], correction[1], 0.0))
        if self.last_visual_target is None:
            self.last_visual_target = self._actual_feet(current)
        self.cached_pose = increment @ pose
        self.last_visual_target = transform_points(
            increment, self.last_visual_target
        )
        return self.last_visual_target

    def _pre_descent_settle_step(self, current):
        self.pre_descent_elapsed = min(
            self.PRE_DESCENT_SETTLE_DURATION_S,
            self.pre_descent_elapsed + self.update_dt,
        )
        if self.pre_descent_elapsed >= self.PRE_DESCENT_SETTLE_DURATION_S:
            self._set_state(self.DESCENT, "下降前姿态已稳定，开始机械导向下降")
            return self._descent_step(current)
        self._set_state(
            self.PRE_DESCENT_SETTLE,
            "保持预对接姿态：{:.2f}/{:.2f}s".format(
                self.pre_descent_elapsed, self.PRE_DESCENT_SETTLE_DURATION_S
            ),
        )
        return self._result(feet=self.pre_descent_feet.copy())

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
            self._set_state(
                self.LEG_LIFT,
                "下坐稳定完成，开始将六腿收至同一高度（至少抬升60mm）",
            )
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
        target_z = (
            float(np.max(self.leg_lift_start_feet[:, 2]))
            + self.LEG_LIFT_HEIGHT_M
        )
        travel = target_z - float(np.min(self.leg_lift_start_feet[:, 2]))

        if self.leg_lift_progress >= travel:
            feet = self.leg_lift_start_feet.copy()
            feet[:, 2] = target_z
            actual_z = self._actual_feet(current)[:, 2]
            actual_spread = float(np.ptp(actual_z))
            target_error = float(np.max(np.abs(actual_z - target_z)))
            if max(actual_spread, target_error) > self.LEG_LIFT_LEVEL_TOLERANCE_M:
                self._set_state(
                    self.LEG_LIFT,
                    "收腿收敛中：高度差{:.1f}mm，目标误差{:.1f}mm".format(
                        actual_spread * 1000.0, target_error * 1000.0
                    ),
                )
                return self._result(feet=feet)
            if self.require_lock_confirmation:
                self._set_state(self.ALIGNED, "六腿已收至同一高度，等待锁紧机构确认")
            else:
                self._set_state(self.SUCCESS, "六腿已收至同一高度，对接结束")
            return self._result(joints=current.copy())

        self.leg_lift_progress = min(
            travel,
            self.leg_lift_progress + self.leg_lift_speed_m_s * self.update_dt,
        )
        feet = self.leg_lift_start_feet.copy()
        feet[:, 2] = np.minimum(
            feet[:, 2] + self.leg_lift_progress,
            target_z,
        )
        self._set_state(
            self.LEG_LIFT,
            "六腿同步抬起中：{:.1f}/{:.1f}mm，目标同高".format(
                self.leg_lift_progress * 1000.0, travel * 1000.0
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

    def _perception_pose(self, robot_state):
        raw = self._field(robot_state, "lock_from_pin")
        if raw is None:
            self.perception_elapsed += self.update_dt
            refresh = (
                not self.perception_sampled
                or self.perception_elapsed + 1e-12 >= self.perception_period
            )
            if not refresh:
                return (
                    None if self.cached_pose is None else self.cached_pose.copy(),
                    self.last_perception_reason,
                )
            self.perception_elapsed = 0.0
            self.perception_sampled = True
            observed = self.perception.latest()
            if not getattr(observed, "valid", False):
                self.cached_pose = None
                self.last_perception_reason = observed.reason
                return None, observed.reason
            raw = observed.lock_from_pin
            self.last_perception_reason = ""
        try:
            pose = np.asarray(raw, dtype=np.float64).reshape(4, 4)
        except (TypeError, ValueError):
            return None, "invalid lock_from_pin shape"
        if not np.isfinite(pose).all():
            return None, "non-finite lock_from_pin"
        self.cached_pose = pose.copy()
        return pose, ""

    def _update_once(self, robot_state=None):
        if self.state in self.TERMINAL_STATES:
            joints = self._field(robot_state, "joints")
            try:
                joints = None if joints is None else np.asarray(
                    joints, dtype=np.float64
                ).reshape(6, 3)
            except (TypeError, ValueError):
                joints = None
            return self._result(joints=None if joints is None else joints.copy())

        current = self._field(robot_state, "joints")
        try:
            current = np.asarray(current, dtype=np.float64).reshape(6, 3)
        except (TypeError, ValueError):
            self.fail_execution("对接执行缺少有效的18关节反馈")
            return self._result()
        if not np.isfinite(current).all():
            self.fail_execution("关节反馈包含非有限值")
            return self._result()

        if self.state == self.CLIMB_TERMINAL_ENTRY:
            return self._update_entry(current)
        if self.state == self.BODY_RAISE:
            return self._body_raise_step(current)
        if self.state == self.PRE_DESCENT_SETTLE:
            return self._pre_descent_settle_step(current)
        if self.state == self.DESCENT:
            return self._descent_step(current)
        if self.state == self.SIT_SETTLE:
            return self._sit_settle_step(current)
        if self.state == self.LEG_LIFT:
            return self._leg_lift_step(current)

        if self.state == self.ALIGNED:
            if self._field(robot_state, "lock_confirmed") is True:
                self._set_state(self.SUCCESS, "锁紧机构已确认，对接成功")
            return self._result(joints=current.copy())

        pose, perception_reason = self._perception_pose(robot_state)
        if pose is None:
            # 失去目标时停在实测姿态，重获目标后不累计失效前的足端指令。
            self.last_visual_target = self._synced_feet(current).copy()
            self._set_state(
                self.WAITING_RING,
                "等待同心圆：" + (perception_reason or "no ring detection"),
            )
            return self._result(joints=current.copy())

        horizontal = float(np.linalg.norm(pose[:2, 3]))
        if horizontal <= self.PREALIGN_POSITION_REFERENCE:
            # 只保证导向行程充足；机身的实际下止点由导向锥机械限定。
            self.descent_total = (
                max(0.0, -float(pose[2, 3])) + self.HOLE_DEPTH_M
                if self.DESCENT_DISTANCE_M is None
                else max(0.0, float(self.DESCENT_DISTANCE_M))
            )
            self.descent_remaining = self.descent_total
            self.descent_duration = self.descent_total / self.linear_speed_m_s
            self.pre_descent_elapsed = 0.0
            self.pre_descent_feet = self._synced_feet(current).copy()
            self.last_visual_target = self.pre_descent_feet.copy()
            self._set_state(
                self.PRE_DESCENT_SETTLE,
                "同心圆到达下降参考：水平{:.1f}mm；稳定等待0.5s".format(
                    horizontal * 1000.0
                ),
            )
            return self._result(feet=self.pre_descent_feet.copy())

        self._set_state(
            self.PREALIGN,
            "同心圆水平调整：水平{:.1f}mm".format(horizontal * 1000.0),
        )
        return self._result(feet=self._visual_step(current, pose))

    def descent_has_started(self):
        return self.state in (
            self.DESCENT, self.SIT_SETTLE, self.LEG_LIFT,
            self.ALIGNED, self.SUCCESS
        )


def _synthetic_ring_image(center=(680, 350), outer_diameter=360, y_scale=1.0):
    """生成仅供self_check使用的标准目标。"""

    image = np.full((720, 1280), 255, dtype=np.uint8)
    scale = float(outer_diameter) / OUTER_RING_DIAMETER_M
    for diameter, color in reversed(list(zip(RING_OUTER_DIAMETERS_M, RING_COLORS))):
        radius_x = int(round(0.5 * diameter * scale))
        radius_y = int(round(radius_x * y_scale))
        cv2.ellipse(
            image, tuple(center), (radius_x, radius_y), 0.0, 0.0, 360.0,
            255 if color == "white" else 0, -1,
        )
    pin_radius_x = int(round(0.5 * PIN_DIAMETER_M * scale))
    pin_radius_y = int(round(pin_radius_x * y_scale))
    cv2.ellipse(
        image, tuple(center), (pin_radius_x, pin_radius_y),
        0.0, 0.0, 360.0, 128, -1,
    )
    return image


def self_check():
    """不依赖相机和舵机的圆环识别及动作状态机自检。"""

    from types import SimpleNamespace

    detection = detect_concentric_rings(_synthetic_ring_image())
    if not detection.valid or detection.matched_boundaries < 6:
        raise AssertionError("front-view ring detection self-check failed")
    if np.linalg.norm(detection.center_px - np.array((680.0, 350.0))) > 2.0:
        raise AssertionError("ring center self-check failed")
    shifted = detect_concentric_rings(
        _synthetic_ring_image(center=(610, 390), outer_diameter=300)
    )
    if not shifted.valid or np.linalg.norm(
        shifted.center_px - np.array((610.0, 390.0))
    ) > 3.0:
        raise AssertionError("shifted ring detection self-check failed")
    partial_image = _synthetic_ring_image(
        center=(610, 390), outer_diameter=300
    )[:, :625]
    partial = detect_concentric_rings(partial_image)
    if not partial.valid or np.linalg.norm(
        partial.center_px - np.array((610.0, 390.0))
    ) > 5.0 or abs(partial.major_diameter_px - 300.0) > 12.0:
        raise AssertionError("cropped ring-arc detection self-check failed")
    occluded_image = _synthetic_ring_image(
        center=(610, 390), outer_diameter=300
    )
    cv2.rectangle(occluded_image, (600, 0), (1279, 719), 255, -1)
    occluded = detect_concentric_rings(occluded_image)
    if not occluded.valid or np.linalg.norm(
        occluded.center_px - np.array((610.0, 390.0))
    ) > 5.0 or abs(occluded.major_diameter_px - 300.0) > 12.0:
        raise AssertionError("occluded ring-arc detection self-check failed")
    three = detect_concentric_rings(
        _synthetic_ring_image(center=(1600, 360), outer_diameter=900)
    )
    if not three.valid or three.matched_boundaries != 3:
        raise AssertionError("three-boundary detection self-check failed")
    distractor = np.full((300, 300), 255, dtype=np.uint8)
    cv2.circle(distractor, (150, 150), 80, 0, 8)
    if detect_concentric_rings(distractor).valid:
        raise AssertionError("single circle must not be accepted")

    camera_pose = camera_from_ring(detection, DEFAULT_CAMERA_MATRIX)
    if camera_pose[2, 3] <= 0.0 or not np.isfinite(camera_pose).all():
        raise AssertionError("ring metric pose self-check failed")

    class Controller:
        dt = 0.1

        def __init__(self):
            self.feet = np.zeros((6, 3))
            self.kinematic = SimpleNamespace(
                forward_base=lambda joints: self.feet.copy()
            )

        @staticmethod
        def _smooth_step(phase):
            return phase

        def _sync_actual_feet(self, joints):
            return self.feet.copy()

    class Perception:
        def __init__(self, pose):
            self.pose = pose

        def reset(self):
            pass

        def latest(self):
            return PerceptionResult(valid=True, lock_from_pin=self.pose)

    state = {"joints": np.zeros((6, 3))}
    mode = DockMode(Controller(), Perception(transform((0.0, 0.0, -0.030))))
    mode.active, mode.state = True, mode.WAITING_RING
    if mode.update(state).state != mode.PRE_DESCENT_SETTLE:
        raise AssertionError("ring-to-descent transition self-check failed")
    mode.perception.latest = lambda: (_ for _ in ()).throw(
        AssertionError("descent must not read the ring image again")
    )
    for _ in range(60):
        result = mode.update(state)
        if result.foot_positions_base is not None:
            mode.controller.feet = result.foot_positions_base.copy()
        if result.state in mode.TERMINAL_STATES:
            break
    if mode.state != mode.SUCCESS:
        raise AssertionError("ring docking completion self-check failed")
    if mode.update(None).state != mode.SUCCESS:
        raise AssertionError("terminal HOLD must preserve success without feedback")

    stalled = DockMode(Controller(), Perception(transform((0.0, 0.0, -0.030))))
    stalled.active, stalled.state = True, stalled.LEG_LIFT
    for _ in range(60):
        stalled.update(state)
    if stalled.state != stalled.LEG_LIFT:
        raise AssertionError("equal-height feet below target must not report success")

    confirmed = DockMode(Controller(), Perception(transform((0.020, 0.0, -0.030))),
                         require_lock_confirmation=True)
    confirmed.active, confirmed.state = True, confirmed.WAITING_RING
    if confirmed.update(dict(state, lock_confirmed=True)).state == confirmed.SUCCESS:
        raise AssertionError("early lock confirmation must not bypass docking")
    confirmed.state = confirmed.LEG_LIFT
    for _ in range(60):
        result = confirmed.update(state)
        if result.foot_positions_base is not None:
            confirmed.controller.feet = result.foot_positions_base.copy()
        if result.state == confirmed.ALIGNED:
            break
    if confirmed.state != confirmed.ALIGNED or not result.request_lock:
        raise AssertionError("confirmation mode must request lock after leg lift")
    if confirmed.update(dict(state, lock_confirmed=True)).state != confirmed.SUCCESS:
        raise AssertionError("aligned mode must accept lock confirmation")
    return True


__all__ = (
    "load_dock_system",
    "transform",
    "invert_transform",
    "LOCK_FROM_CAMERA",
    "DockMode",
    "DockResult",
    "DockRobotState",
    "DockPerception",
    "RingPerception",
    "PerceptionResult",
    "RingDetection",
    "PIN_DIAMETER_M",
    "RING_OUTER_DIAMETERS_M",
    "RING_COLORS",
    "OUTER_RING_DIAMETER_M",
    "DEFAULT_CAMERA_MATRIX",
    "detect_concentric_rings",
    "camera_from_ring",
    "self_check",
)
