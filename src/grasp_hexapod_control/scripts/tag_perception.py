#!/usr/bin/env python3
"""Three-AprilTag perception for Xiaolan side scanning.

Side tags continuously publish the angle between camera optical +Z and tag +X.
The rear tag publishes a debounced request for a future path planner.  This
node never sends velocity, gait, joint, or servo commands.
"""

import json
import math
import statistics
from collections import deque
from dataclasses import dataclass
from threading import Lock

import rospy
import cv2
from apriltag_ros.msg import AprilTagDetectionArray
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String


SIDE_ROLES = {"left", "right"}


@dataclass(frozen=True)
class TagSpec:
    role: str
    size_m: float


class AngleFilter:
    """Reject pose spikes, then smooth the remaining angle measurements."""

    def __init__(self, window_size=7, alpha=0.25, max_rate_deg_s=120.0,
                 reset_timeout_s=0.7):
        if window_size < 1 or window_size % 2 == 0:
            raise ValueError("angle filter window must be a positive odd number")
        if not 0.0 < alpha <= 1.0:
            raise ValueError("angle filter alpha must be in (0, 1]")
        if max_rate_deg_s <= 0.0 or reset_timeout_s <= 0.0:
            raise ValueError("angle filter rate and reset timeout must be positive")
        self.samples = deque(maxlen=window_size)
        self.alpha = alpha
        self.max_rate_deg_s = max_rate_deg_s
        self.reset_timeout_s = reset_timeout_s
        self.value = None
        self.last_time_s = None

    def update(self, raw_angle_deg, now_s):
        if (self.last_time_s is None
                or now_s - self.last_time_s > self.reset_timeout_s):
            self.samples.clear()
            self.value = raw_angle_deg
            self.last_time_s = now_s
            self.samples.append(raw_angle_deg)
            return self.value

        self.samples.append(raw_angle_deg)
        target = statistics.median(self.samples)
        smoothed = self.value + self.alpha * (target - self.value)
        dt = max(1e-3, now_s - self.last_time_s)
        max_step = self.max_rate_deg_s * dt
        self.value += max(-max_step, min(max_step, smoothed - self.value))
        self.last_time_s = now_s
        return self.value


def parse_tag_specs(raw):
    if not isinstance(raw, dict) or not raw:
        raise ValueError("~tags must be a non-empty dictionary")
    result = {}
    roles = set()
    for raw_id, value in raw.items():
        tag_id = int(raw_id)
        if tag_id < 0 or not isinstance(value, dict):
            raise ValueError("each tag needs a non-negative ID and dictionary config")
        role = str(value.get("role", "")).strip().lower()
        size_m = float(value.get("size_m", 0.0))
        if role not in SIDE_ROLES | {"rear"}:
            raise ValueError("tag role must be left, right, or rear")
        if role in roles:
            raise ValueError("tag roles must be unique")
        if size_m <= 0.0:
            raise ValueError("tag size_m must be positive")
        result[tag_id] = TagSpec(role, size_m)
        roles.add(role)
    if not SIDE_ROLES.issubset(roles) or "rear" not in roles:
        raise ValueError("~tags must define exactly one left, right, and rear role")
    return result


def camera_normal_to_tag_x_angle_deg(orientation):
    """Return angle(camera optical +Z, tag +X), in the range [0, 180]."""
    norm = math.sqrt(
        orientation.x ** 2 + orientation.y ** 2
        + orientation.z ** 2 + orientation.w ** 2
    )
    if norm < 1e-12:
        raise ValueError("zero-length orientation quaternion")
    x = orientation.x / norm
    y = orientation.y / norm
    z = orientation.z / norm
    w = orientation.w / norm
    # R[2,0] is dot(camera +Z, tag +X).
    cosine = 2.0 * (x * z - w * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def detection_payload(detection, stamp, spec, angle_deg=None):
    pose = detection.pose.pose.pose
    payload = {
        "tag_id": int(detection.id[0]),
        "role": spec.role,
        "stamp": stamp.to_sec(),
        "frame_id": detection.pose.header.frame_id,
        "configured_size_m": spec.size_m,
        "position_m": {
            "x": pose.position.x, "y": pose.position.y, "z": pose.position.z,
        },
        "orientation_xyzw": {
            "x": pose.orientation.x, "y": pose.orientation.y,
            "z": pose.orientation.z, "w": pose.orientation.w,
        },
    }
    if angle_deg is not None:
        payload["angle_deg"] = angle_deg
        payload["angle_definition"] = "camera_+Z_to_tag_+X"
    return payload


class XiaolanTagNode:
    def __init__(self):
        self.tags = parse_tag_specs(rospy.get_param("~tags"))
        self.rear_confirmation_frames = int(
            rospy.get_param("~rear_confirmation_frames", 5)
        )
        self.rear_lost_timeout = float(rospy.get_param("~rear_lost_timeout", 0.7))
        self.rear_repeat_cooldown = float(rospy.get_param("~rear_repeat_cooldown", 5.0))
        self.log_side_angles = bool(rospy.get_param("~log_side_angles", True))
        filter_window = int(rospy.get_param("~angle_filter_window", 7))
        filter_alpha = float(rospy.get_param("~angle_filter_alpha", 0.25))
        filter_max_rate = float(rospy.get_param("~angle_filter_max_rate_deg_s", 120.0))
        filter_reset_timeout = float(rospy.get_param("~angle_filter_reset_timeout", 0.7))
        if self.rear_confirmation_frames < 1:
            raise ValueError("~rear_confirmation_frames must be >= 1")
        if self.rear_lost_timeout <= 0.0 or self.rear_repeat_cooldown < 0.0:
            raise ValueError("rear timeout must be positive and cooldown non-negative")

        self.side_pub = rospy.Publisher(
            rospy.get_param("~side_angle_topic", "/grasp_hexapod/perception/side_angle"),
            String, queue_size=20,
        )
        self.angle_pubs = {
            "left": rospy.Publisher(
                rospy.get_param("~left_angle_topic", "/grasp_hexapod/perception/left_angle_deg"),
                Float64, queue_size=10,
            ),
            "right": rospy.Publisher(
                rospy.get_param("~right_angle_topic", "/grasp_hexapod/perception/right_angle_deg"),
                Float64, queue_size=10,
            ),
        }
        self.plan_pub = rospy.Publisher(
            rospy.get_param("~side_scan_request_topic", "/grasp_hexapod/navigation/side_scan_request"),
            String, queue_size=5,
        )
        self.event_pub = rospy.Publisher(
            rospy.get_param("~event_topic", "/grasp_hexapod/perception/tag_event"),
            String, queue_size=20,
        )
        self.angle_image_pub = rospy.Publisher(
            rospy.get_param(
                "~angle_image_topic",
                "/grasp_hexapod/perception/angle_image",
            ),
            Image, queue_size=1,
        )

        self.lock = Lock()
        self.bridge = CvBridge()
        self.latest_side_angles = {}
        self.angle_filters = {
            role: AngleFilter(filter_window, filter_alpha, filter_max_rate,
                              filter_reset_timeout)
            for role in SIDE_ROLES
        }
        self.rear_count = 0
        self.rear_confirmed = False
        self.rear_last_seen = rospy.Time(0)
        self.rear_last_trigger = rospy.Time(0)
        self.subscriber = rospy.Subscriber(
            rospy.get_param("~detections_topic", "/tag_detections"),
            AprilTagDetectionArray, self._callback, queue_size=1, tcp_nodelay=True,
        )
        self.image_subscriber = rospy.Subscriber(
            rospy.get_param("~debug_image_topic", "/tag_detections_image"),
            Image, self._image_callback, queue_size=1, buff_size=2 ** 24,
        )
        self.timer = rospy.Timer(rospy.Duration(0.1), self._check_rear_lost)
        rospy.loginfo("Xiaolan three-tag node ready: %s", {
            tag_id: spec.role for tag_id, spec in sorted(self.tags.items())
        })

    @staticmethod
    def _stamp(message):
        return message.header.stamp if message.header.stamp != rospy.Time(0) else rospy.Time.now()

    @staticmethod
    def _json(payload):
        return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))

    def _publish_side(self, detection, stamp, spec):
        try:
            raw_angle = camera_normal_to_tag_x_angle_deg(
                detection.pose.pose.pose.orientation
            )
        except ValueError as error:
            rospy.logwarn_throttle(2.0, "Invalid pose for tag %d: %s", detection.id[0], error)
            return
        now = rospy.Time.now()
        angle = self.angle_filters[spec.role].update(raw_angle, now.to_sec())
        payload = detection_payload(detection, stamp, spec, angle)
        payload["raw_angle_deg"] = raw_angle
        payload["angle_filter"] = "median_ema_slew_limit"
        with self.lock:
            self.latest_side_angles[spec.role] = (
                int(detection.id[0]), angle, now
            )
        self.side_pub.publish(String(data=self._json(payload)))
        self.angle_pubs[spec.role].publish(Float64(data=angle))
        if self.log_side_angles:
            rospy.loginfo_throttle(
                0.2, "%s tag ID=%d | angle = %.2f deg (raw %.2f)",
                spec.role, detection.id[0], angle, raw_angle,
            )

    def _image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logwarn_throttle(2.0, "Cannot convert tag debug image: %s", error)
            return
        now = rospy.Time.now()
        with self.lock:
            angles = dict(self.latest_side_angles)
        lines = []
        for role in ("left", "right"):
            value = angles.get(role)
            if value is not None and (now - value[2]).to_sec() <= 0.5:
                lines.append("%s ID %d: %.2f deg" % (
                    role.upper(), value[0], value[1]
                ))
        if not lines:
            lines.append("NO SIDE TAG")
        for index, line in enumerate(lines):
            origin = (16, 34 + index * 34)
            cv2.putText(image, line, origin, cv2.FONT_HERSHEY_SIMPLEX,
                        0.85, (0, 0, 0), 5, cv2.LINE_AA)
            cv2.putText(image, line, origin, cv2.FONT_HERSHEY_SIMPLEX,
                        0.85, (0, 255, 255), 2, cv2.LINE_AA)
        output = Image()
        output.header = message.header
        output.height, output.width = image.shape[:2]
        output.encoding = "bgr8"
        output.is_bigendian = False
        output.step = output.width * 3
        output.data = image.tobytes()
        self.angle_image_pub.publish(output)

    def _handle_rear(self, detection, stamp, spec):
        publish_request = False
        with self.lock:
            self.rear_last_seen = rospy.Time.now()
            if self.rear_confirmed:
                return
            self.rear_count += 1
            if self.rear_count < self.rear_confirmation_frames:
                return
            now = rospy.Time.now()
            if (now - self.rear_last_trigger).to_sec() < self.rear_repeat_cooldown:
                return
            self.rear_confirmed = True
            self.rear_last_trigger = now
            publish_request = True
        if publish_request:
            payload = detection_payload(detection, stamp, spec)
            payload.update({
                "event": "rear_tag_confirmed",
                "request": "plan_to_side_scan_pose",
                "motion_executed": False,
            })
            message = String(data=self._json(payload))
            self.plan_pub.publish(message)
            self.event_pub.publish(message)
            rospy.logwarn(
                "Rear tag ID=%d confirmed: side-scan planning requested; no motion executed",
                detection.id[0],
            )

    def _callback(self, message):
        stamp = self._stamp(message)
        rear_seen = False
        for detection in message.detections:
            if not detection.id:
                continue
            tag_id = int(detection.id[0])
            spec = self.tags.get(tag_id)
            if spec is None:
                rospy.logwarn_throttle(2.0, "Ignoring unconfigured AprilTag ID %d", tag_id)
                continue
            if spec.role in SIDE_ROLES:
                self._publish_side(detection, stamp, spec)
            else:
                rear_seen = True
                self._handle_rear(detection, stamp, spec)
        if not rear_seen:
            with self.lock:
                if not self.rear_confirmed:
                    self.rear_count = 0

    def _check_rear_lost(self, _event):
        now = rospy.Time.now()
        with self.lock:
            if not self.rear_confirmed:
                return
            if (now - self.rear_last_seen).to_sec() <= self.rear_lost_timeout:
                return
            self.rear_confirmed = False
            self.rear_count = 0
        payload = {"event": "rear_tag_lost", "stamp": now.to_sec()}
        self.event_pub.publish(String(data=self._json(payload)))
        rospy.loginfo("Rear tag lost; trigger re-armed after cooldown")


def main():
    rospy.init_node("xiaolan_tag_perception")
    XiaolanTagNode()
    rospy.spin()


if __name__ == "__main__":
    main()
