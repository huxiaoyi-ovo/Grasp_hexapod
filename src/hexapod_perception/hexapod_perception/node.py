"""ROS 1 Noetic node publishing board and Xiaolan relative poses."""

import json
from dataclasses import replace

import cv2
import message_filters
import numpy as np
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import tf2_ros

from .geometry import (estimate_relative_pose, estimate_relative_position,
                       orient_pose_from_pixels)
from .segmenter import YoloSegmenter


def _image_array(message):
    if message.encoding in ("rgb8", "bgr8"):
        array = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
        array = array[:, :message.width * 3].reshape(message.height, message.width, 3)
        return array if message.encoding == "rgb8" else array[:, :, ::-1]
    if message.encoding == "16UC1":
        dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
        row = np.frombuffer(message.data, dtype).reshape(message.height, message.step // 2)
        return row[:, :message.width]
    if message.encoding == "32FC1":
        dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
        row = np.frombuffer(message.data, dtype).reshape(message.height, message.step // 4)
        return row[:, :message.width]
    raise ValueError(f"unsupported encoding: {message.encoding}")


def _transform_matrix(transform):
    t, q = transform.translation, transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


def _matrix_to_quaternion(rotation):
    # Stable branch-based conversion, returns ROS xyzw ordering.
    m = rotation
    trace = np.trace(m)
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        return np.array([(m[2,1]-m[1,2])/s, (m[0,2]-m[2,0])/s,
                         (m[1,0]-m[0,1])/s, 0.25*s])
    index = int(np.argmax(np.diag(m)))
    if index == 0:
        s = np.sqrt(1 + m[0,0] - m[1,1] - m[2,2]) * 2
        return np.array([0.25*s, (m[0,1]+m[1,0])/s,
                         (m[0,2]+m[2,0])/s, (m[2,1]-m[1,2])/s])
    if index == 1:
        s = np.sqrt(1 + m[1,1] - m[0,0] - m[2,2]) * 2
        return np.array([(m[0,1]+m[1,0])/s, 0.25*s,
                         (m[1,2]+m[2,1])/s, (m[0,2]-m[2,0])/s])
    s = np.sqrt(1 + m[2,2] - m[0,0] - m[1,1]) * 2
    return np.array([(m[0,2]+m[2,0])/s, (m[1,2]+m[2,1])/s,
                     0.25*s, (m[1,0]-m[0,1])/s])


def _mask_center_scale(mask):
    if mask is None or not np.any(mask):
        return None, 0.0
    ys, xs = np.nonzero(mask)
    center = np.array([np.mean(xs), np.mean(ys)], np.float64)
    scale = float(max(xs.max() - xs.min() + 1, ys.max() - ys.min() + 1))
    return center, scale


def _feature_near_mask(detections, mask, margin_ratio=0.25):
    """Reject full-frame feature false positives far from Xiaolan."""
    center, scale = _mask_center_scale(mask)
    if center is None:
        return []
    ys, xs = np.nonzero(mask)
    margin = margin_ratio * scale
    x1, x2 = xs.min() - margin, xs.max() + margin
    y1, y2 = ys.min() - margin, ys.max() + margin
    return [(point, score) for point, score in detections
            if x1 <= point[0] <= x2 and y1 <= point[1] <= y2]


def _mask_detection(mask, score):
    center, _ = _mask_center_scale(mask)
    return [] if center is None else [(center, score)]


def _mask_detections(mask, score, maximum=2):
    count, labels, stats, centers = cv2.connectedComponentsWithStats(
        np.asarray(mask, np.uint8), connectivity=8)
    indices = sorted(range(1, count), key=lambda i: stats[i, cv2.CC_STAT_AREA],
                     reverse=True)[:maximum]
    return [(centers[index], score) for index in indices
            if stats[index, cv2.CC_STAT_AREA] >= 8]


def _mask_is_complete(mask, score, minimum_score=0.5, minimum_pixels=500,
                      border_margin=8):
    """Require a confident body silhouette fully contained in the image."""
    if score < minimum_score or mask is None:
        return False
    ys, xs = np.nonzero(mask)
    if len(xs) < minimum_pixels:
        return False
    height, width = mask.shape
    return bool(xs.min() >= border_margin and ys.min() >= border_margin
                and xs.max() < width - border_margin
                and ys.max() < height - border_margin)


def _orient_pose_like_rotation(pose, reference_rotation):
    """Resolve only the current geometric long-axis sign from prior heading."""
    if pose is None or reference_rotation is None:
        return None
    x_axis = pose.rotation[:, 0].copy()
    normal = pose.rotation[:, 2].copy()
    if np.dot(x_axis, reference_rotation[:, 0]) < 0:
        x_axis = -x_axis
    y_axis = np.cross(normal, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, normal)
    return replace(pose, rotation=np.column_stack((x_axis, y_axis, normal)))


class PerceptionNode:
    def __init__(self):
        defaults = {
            "model_path": "", "output_frame": "base_link",
            "color_topic": "/camera/color/image_raw",
            "depth_topic": "/camera/depth/image_raw",
            "camera_info_topic": "/camera/color/camera_info",
            "board_class": "board", "xiaolan_class": "platform_robot",
            "confidence": 0.45, "front_feature_confidence": 0.15,
            "green_feature_confidence": 0.15,
            "image_size": 640, "device": "cpu",
            "erosion_pixels": 2, "depth_stride": 3, "maximum_plane_rmse": 0.03,
            "minimum_plane_normal_z": 0.55,
            "front_hold_seconds": 0.5,
            "green_crop_margin": 0.28,
            "green_pair_min_separation_ratio": 0.12,
            "xiaolan_complete_confidence": 0.5,
            "xiaolan_complete_minimum_pixels": 500,
            "xiaolan_complete_border_margin": 8,
            "direction_jump_degrees": 60.0,
            "direction_confirmation_frames": 3,
        }
        p = lambda name: rospy.get_param("~" + name, defaults[name])
        if not p("model_path"):
            raise RuntimeError("parameter 'model_path' must point to trained best.pt or ONNX")
        self.output_frame = p("output_frame")
        self.board_class, self.xiaolan_class = p("board_class"), p("xiaolan_class")
        self.depth_stride, self.maximum_rmse = p("depth_stride"), p("maximum_plane_rmse")
        self.minimum_normal_z = float(p("minimum_plane_normal_z"))
        self.front_hold_seconds = float(p("front_hold_seconds"))
        self.green_pair_min_separation_ratio = float(
            p("green_pair_min_separation_ratio"))
        self.front_feature_confidence = float(p("front_feature_confidence"))
        self.green_feature_confidence = float(p("green_feature_confidence"))
        self.xiaolan_complete_confidence = float(p("xiaolan_complete_confidence"))
        self.xiaolan_complete_minimum_pixels = int(
            p("xiaolan_complete_minimum_pixels"))
        self.xiaolan_complete_border_margin = int(
            p("xiaolan_complete_border_margin"))
        self.direction_jump_degrees = float(p("direction_jump_degrees"))
        self.direction_confirmation_frames = int(p("direction_confirmation_frames"))
        self.segmenter = YoloSegmenter(
            p("model_path"), (self.board_class, self.xiaolan_class,
                              "front_feature", "top_green_feature"),
            p("confidence"), p("image_size"), p("device"), p("erosion_pixels"))
        self.k = None
        self._last_front_rotation = None
        self._last_front_stamp = None
        self._pending_rotation = None
        self._pending_rotation_count = 0
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.board_pub = rospy.Publisher("~board_pose", PoseStamped, queue_size=10)
        self.xiaolan_pub = rospy.Publisher("~xiaolan_pose", PoseStamped, queue_size=10)
        self.board_position_pub = rospy.Publisher(
            "~board_position", PointStamped, queue_size=10)
        self.xiaolan_position_pub = rospy.Publisher(
            "~xiaolan_position", PointStamped, queue_size=10)
        self.status_pub = rospy.Publisher("~status", String, queue_size=10)
        self.info_sub = rospy.Subscriber(
            p("camera_info_topic"), CameraInfo, self._info, queue_size=1)
        color = message_filters.Subscriber(p("color_topic"), Image, queue_size=2)
        depth = message_filters.Subscriber(p("depth_topic"), Image, queue_size=2)
        sync = message_filters.ApproximateTimeSynchronizer(
            [color, depth], queue_size=15, slop=0.04)
        sync.registerCallback(self._frame)
        self._sync_handles = (color, depth, sync)
        rospy.loginfo("hexapod perception ready")

    def _info(self, message):
        # ROS 1 sensor_msgs/CameraInfo uses an uppercase K field.
        self.k = np.asarray(message.K, np.float64).reshape(3, 3)

    def _publish_pose(self, publisher, estimate, stamp):
        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.output_frame
        message.pose.position.x, message.pose.position.y, message.pose.position.z = estimate.position
        q = _matrix_to_quaternion(estimate.rotation)
        message.pose.orientation.x, message.pose.orientation.y = q[0], q[1]
        message.pose.orientation.z, message.pose.orientation.w = q[2], q[3]
        publisher.publish(message)

    def _publish_position(self, publisher, position, stamp):
        message = PointStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.output_frame
        message.point.x, message.point.y, message.point.z = position
        publisher.publish(message)

    def _accept_direction(self, pose):
        """Suppress isolated large heading changes, including 180° flips."""
        if pose is None:
            return None
        if self._last_front_rotation is None:
            self._pending_rotation = None
            self._pending_rotation_count = 0
            return pose
        old_axis = self._last_front_rotation[:, 0]
        new_axis = pose.rotation[:, 0]
        angle = float(np.degrees(np.arccos(np.clip(np.dot(old_axis, new_axis), -1, 1))))
        if angle <= self.direction_jump_degrees:
            self._pending_rotation = None
            self._pending_rotation_count = 0
            return pose
        if (self._pending_rotation is not None
                and np.dot(self._pending_rotation[:, 0], new_axis)
                >= np.cos(np.radians(20.0))):
            self._pending_rotation_count += 1
        else:
            self._pending_rotation = pose.rotation.copy()
            self._pending_rotation_count = 1
        if self._pending_rotation_count < self.direction_confirmation_frames:
            return None
        self._pending_rotation = None
        self._pending_rotation_count = 0
        return pose

    def _frame(self, color_message, depth_message):
        if self.k is None:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                # D2C depth pixels follow color-camera rays and therefore use
                # the color optical frame together with color CameraInfo.
                self.output_frame, color_message.header.frame_id,
                depth_message.header.stamp, rospy.Duration(0.05))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as error:
            rospy.logwarn_throttle(2.0, "TF unavailable: %s", error)
            return
        rgb = _image_array(color_message)
        raw = _image_array(depth_message)
        depth = raw.astype(np.float32)
        if depth_message.encoding == "16UC1":
            depth *= 0.001
            depth[raw == np.iinfo(np.uint16).max] = np.nan
        depth[(depth <= 0) | ~np.isfinite(depth)] = np.nan
        if depth.shape != rgb.shape[:2]:
            rospy.logerr_throttle(2.0, "RGB/depth shapes differ; enable D2C registration")
            return
        masks, scores = self.segmenter(rgb)
        xiaolan_mask = masks[self.xiaolan_class]
        body_center, body_scale = _mask_center_scale(xiaolan_mask)
        xiaolan_complete = _mask_is_complete(
            xiaolan_mask, scores[self.xiaolan_class],
            self.xiaolan_complete_confidence,
            self.xiaolan_complete_minimum_pixels,
            self.xiaolan_complete_border_margin)
        front_detections = (_feature_near_mask(_mask_detection(
            masks["front_feature"], scores["front_feature"]), xiaolan_mask)
            if xiaolan_complete and scores["front_feature"] >= self.front_feature_confidence else [])
        front_pixel = front_detections[0][0] if front_detections else None
        front_score = front_detections[0][1] if front_detections else 0.0
        green_detections = (_mask_detections(
            masks["top_green_feature"], scores["top_green_feature"])
            if xiaolan_complete and scores["top_green_feature"] >= self.green_feature_confidence else [])
        green_score = max((item[1] for item in green_detections), default=0.0)
        green_pair_valid = False
        if len(green_detections) >= 2 and body_scale > 0:
            separation = np.linalg.norm(
                green_detections[1][0] - green_detections[0][0])
            green_pair_valid = bool(
                separation >= self.green_pair_min_separation_ratio * body_scale)
        matrix = _transform_matrix(transform.transform)
        board = estimate_relative_pose(
            depth, masks[self.board_class], self.k, matrix,
            stride=self.depth_stride, maximum_rmse=self.maximum_rmse,
            minimum_normal_z=self.minimum_normal_z)
        xiaolan = estimate_relative_pose(
            depth, masks[self.xiaolan_class], self.k, matrix,
            stride=self.depth_stride, maximum_rmse=self.maximum_rmse,
            minimum_normal_z=self.minimum_normal_z)
        board_position = (board.position if board is not None else
                          estimate_relative_position(
                              depth, masks[self.board_class], self.k, matrix,
                              stride=self.depth_stride))
        xiaolan_position = (xiaolan.position if xiaolan is not None else
                            estimate_relative_position(
                                depth, masks[self.xiaolan_class], self.k, matrix,
                                stride=self.depth_stride))
        board_position_valid = board_position is not None
        xiaolan_position_valid = xiaolan_position is not None
        yaw_valid = False
        yaw_held = False
        front_orientation_valid = False
        green_orientation_valid = False
        direction_source = "none"
        if xiaolan is not None and front_pixel is not None:
            oriented_xiaolan = orient_pose_from_pixels(
                xiaolan, body_center, front_pixel, self.k, matrix)
            oriented_xiaolan = self._accept_direction(oriented_xiaolan)
            if oriented_xiaolan is not None:
                xiaolan = oriented_xiaolan
                self._last_front_rotation = xiaolan.rotation.copy()
                self._last_front_stamp = color_message.header.stamp
                yaw_valid = True
                front_orientation_valid = True
                direction_source = "front_feature"
        if (xiaolan is not None and not yaw_valid and green_pair_valid
                and self._last_front_rotation is not None):
            # The two distinct top bumps provide a currently observed
            # unsigned front/back axis.  The front rectangle anchors its sign;
            # while hidden, select the current geometric long-axis sign that
            # remains continuous with that last anchored heading.
            oriented_xiaolan = _orient_pose_like_rotation(
                xiaolan, self._last_front_rotation)
            oriented_xiaolan = self._accept_direction(oriented_xiaolan)
            if oriented_xiaolan is not None:
                xiaolan = oriented_xiaolan
                self._last_front_rotation = xiaolan.rotation.copy()
                self._last_front_stamp = color_message.header.stamp
                yaw_valid = True
                green_orientation_valid = True
                direction_source = "green_track"
        if (xiaolan is not None and not yaw_valid
                and self._last_front_rotation is not None
                and self._last_front_stamp is not None):
            age = (color_message.header.stamp - self._last_front_stamp).to_sec()
            if 0.0 <= age <= self.front_hold_seconds:
                # Keep the current XYZ estimate but reuse the last semantic
                # heading.  Never fall back to the rectangle long axis, whose
                # direction has an unavoidable 180-degree ambiguity.
                xiaolan = replace(xiaolan, rotation=self._last_front_rotation.copy())
                yaw_valid = True
                yaw_held = True
                direction_source = "held"
        if board is not None:
            self._publish_pose(self.board_pub, board, color_message.header.stamp)
        if board_position_valid:
            self._publish_position(
                self.board_position_pub, board_position, color_message.header.stamp)
        if xiaolan_position_valid:
            self._publish_position(
                self.xiaolan_position_pub, xiaolan_position,
                color_message.header.stamp)
        if xiaolan is not None and yaw_valid:
            self._publish_pose(self.xiaolan_pub, xiaolan, color_message.header.stamp)
        status = {
            "frame": self.output_frame,
            "board_valid": board is not None,
            "board_position_valid": board_position_valid,
            "xiaolan_valid": xiaolan_position_valid and yaw_valid,
            "xiaolan_position_valid": xiaolan_position_valid,
            "xiaolan_complete": xiaolan_complete,
            "xiaolan_yaw_valid": yaw_valid,
            "xiaolan_yaw_held": yaw_held,
            "xiaolan_direction_source": direction_source,
            "board_score": scores[self.board_class],
            "xiaolan_score": scores[self.xiaolan_class],
            "xiaolan_front_valid": front_orientation_valid,
            # Retain legacy names so existing monitors keep working.  The
            # count now means front-feature instances rather than pose points.
            "xiaolan_front_keypoint_count": 1 if front_pixel is not None else 0,
            "xiaolan_front_score": front_score,
            "xiaolan_front_feature_count": len(front_detections),
            "xiaolan_green_feature_count": len(green_detections),
            "xiaolan_green_pair_valid": green_pair_valid,
            "xiaolan_green_orientation_valid": green_orientation_valid,
            "xiaolan_green_score": green_score,
            "board_plane_rmse": None if board is None else board.plane_rmse,
            "xiaolan_plane_rmse": None if xiaolan is None else xiaolan.plane_rmse,
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))


def main():
    rospy.init_node("hexapod_perception")
    PerceptionNode()
    rospy.spin()
