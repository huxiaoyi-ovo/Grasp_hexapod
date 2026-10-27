"""Dependency-light RGB-D geometry used by the ROS node."""

from dataclasses import dataclass
from dataclasses import replace
from typing import Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class RelativePose:
    position: np.ndarray       # XYZ in output frame, metres
    rotation: np.ndarray       # 3x3 object-to-output rotation
    polygon_xy: np.ndarray     # visible fitted rectangle in output XY
    plane_rmse: float
    valid_points: int


def _deproject(depth_m, mask, k, transform, stride=3,
               min_depth=0.15, max_depth=8.0):
    depth = np.asarray(depth_m, np.float64)
    mask = np.asarray(mask, bool)
    k = np.asarray(k, np.float64).reshape(3, 3)
    transform = np.asarray(transform, np.float64)
    if depth.ndim != 2 or mask.shape != depth.shape:
        raise ValueError("depth and mask must have identical HxW shapes")
    if transform.shape != (4, 4):
        raise ValueError("transform must have shape (4,4)")
    sampled = np.zeros_like(mask)
    sampled[::stride, ::stride] = True
    valid = mask & sampled & np.isfinite(depth)
    valid &= (depth >= min_depth) & (depth <= max_depth)
    v, u = np.nonzero(valid)
    if not len(u):
        return np.empty((0, 3))
    z = depth[v, u]
    camera = np.column_stack(((u - k[0, 2]) * z / k[0, 0],
                              (v - k[1, 2]) * z / k[1, 1], z))
    return camera @ transform[:3, :3].T + transform[:3, 3]


def _robust_plane(points, iterations=3):
    kept = np.asarray(points, np.float64)
    if len(kept) < 30:
        raise ValueError("not enough depth points")
    for _ in range(iterations):
        center = np.median(kept, axis=0)
        _, _, vh = np.linalg.svd(kept - center, full_matrices=False)
        normal = vh[-1]
        residual = np.abs((kept - center) @ normal)
        median = np.median(residual)
        mad = np.median(np.abs(residual - median))
        limit = max(0.008, median + 3.5 * 1.4826 * mad)
        inliers = residual <= limit
        if inliers.all() or np.count_nonzero(inliers) < 30:
            break
        kept = kept[inliers]
    center = np.mean(kept, axis=0)
    _, _, vh = np.linalg.svd(kept - center, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0:
        normal = -normal
    residual = (kept - center) @ normal
    rmse = float(np.sqrt(np.mean(residual ** 2)))
    return kept, center, normal, rmse


def estimate_relative_pose(depth_m, mask, k, output_from_camera,
                           *, stride=3, maximum_rmse=0.03,
                           minimum_normal_z=0.55) -> Optional[RelativePose]:
    """Estimate a rectangular support's 6-DoF pose in the output frame."""
    points = _deproject(depth_m, mask, k, output_from_camera, stride=stride)
    try:
        points, plane_center, normal, rmse = _robust_plane(points)
    except (ValueError, np.linalg.LinAlgError):
        return None
    if normal[2] < minimum_normal_z or rmse > maximum_rmse:
        return None

    rectangle = cv2.minAreaRect(points[:, :2].astype(np.float32))
    polygon = cv2.boxPoints(rectangle).astype(np.float64)
    center_xy = np.asarray(rectangle[0], np.float64)
    z = plane_center[2] - (
        normal[0] * (center_xy[0] - plane_center[0])
        + normal[1] * (center_xy[1] - plane_center[1])) / normal[2]
    position = np.array([center_xy[0], center_xy[1], z])

    edges = np.roll(polygon, -1, axis=0) - polygon
    long_edge = edges[np.argmax(np.linalg.norm(edges, axis=1))]
    x_axis = np.array([long_edge[0], long_edge[1], 0.0])
    x_axis -= normal * np.dot(x_axis, normal)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(normal, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, normal)
    rotation = np.column_stack((x_axis, y_axis, normal))
    return RelativePose(position, rotation, polygon, rmse, len(points))


def estimate_relative_position(depth_m, mask, k, output_from_camera,
                               *, stride=3, minimum_points=5):
    """Estimate XYZ without requiring a reliable surface orientation.

    Position and orientation have different observability at steep views.  A
    noisy plane normal must not discard otherwise useful depth.  The XY center
    comes from the robust point footprint and Z from its median depth.
    """
    points = _deproject(depth_m, mask, k, output_from_camera, stride=stride)
    if len(points) < minimum_points:
        return None
    center = np.median(points, axis=0)
    distances = np.linalg.norm(points - center, axis=1)
    median = np.median(distances)
    mad = np.median(np.abs(distances - median))
    limit = median + 3.5 * 1.4826 * mad
    kept = points[distances <= max(limit, 0.02)]
    if len(kept) < minimum_points:
        return None
    rectangle = cv2.minAreaRect(kept[:, :2].astype(np.float32))
    center_xy = np.asarray(rectangle[0], np.float64)
    return np.array([center_xy[0], center_xy[1], np.median(kept[:, 2])])


def orient_pose_toward_pixel(pose, pixel_xy, k, output_from_camera):
    """Resolve the 180-degree long-axis ambiguity using a semantic front pixel.

    The geometric long axis remains the yaw axis.  The learned pixel selects
    only its sign, so a single off-centre front endpoint cannot bias the angle.
    """
    if pose is None or pixel_xy is None:
        return pose
    k = np.asarray(k, np.float64).reshape(3, 3)
    transform = np.asarray(output_from_camera, np.float64)
    u, v = map(float, pixel_xy)
    ray_camera = np.array([(u - k[0, 2]) / k[0, 0],
                           (v - k[1, 2]) / k[1, 1], 1.0])
    origin = transform[:3, 3]
    ray = transform[:3, :3] @ ray_camera
    normal = pose.rotation[:, 2]
    denominator = float(np.dot(normal, ray))
    if abs(denominator) < 1e-8:
        return None
    distance = float(np.dot(normal, pose.position - origin) / denominator)
    if distance <= 0:
        return None
    front = origin + distance * ray
    toward_front = front - pose.position
    toward_front -= normal * np.dot(toward_front, normal)
    if np.linalg.norm(toward_front) < 0.02:
        return None
    x_axis = pose.rotation[:, 0].copy()
    # A point on either front corner is sufficient: only the projection onto
    # the long axis matters.  Reject a point exactly on the lateral midline.
    signed_forward = float(np.dot(x_axis, toward_front))
    if abs(signed_forward) < 0.005:
        return None
    if signed_forward < 0:
        x_axis = -x_axis
    y_axis = np.cross(normal, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, normal)
    rotation = np.column_stack((x_axis, y_axis, normal))
    return replace(pose, rotation=rotation)


def orient_pose_from_pixels(pose, center_pixel_xy, front_pixel_xy, k,
                            output_from_camera):
    """Set heading directly from the body-center to semantic-front pixels.

    Both rays are intersected with the fitted object plane.  Unlike
    ``orient_pose_toward_pixel``, the semantic feature determines the full
    in-plane angle, not merely the sign of an unstable silhouette long axis.
    """
    if pose is None or center_pixel_xy is None or front_pixel_xy is None:
        return None
    k = np.asarray(k, np.float64).reshape(3, 3)
    transform = np.asarray(output_from_camera, np.float64)
    normal = pose.rotation[:, 2].copy()
    origin = transform[:3, 3]

    def plane_point(pixel):
        u, v = map(float, pixel)
        camera_ray = np.array([(u - k[0, 2]) / k[0, 0],
                               (v - k[1, 2]) / k[1, 1], 1.0])
        ray = transform[:3, :3] @ camera_ray
        denominator = float(np.dot(normal, ray))
        if abs(denominator) < 1e-8:
            return None
        distance = float(np.dot(normal, pose.position - origin) / denominator)
        if distance <= 0:
            return None
        return origin + distance * ray

    center = plane_point(center_pixel_xy)
    front = plane_point(front_pixel_xy)
    if center is None or front is None:
        return None
    x_axis = front - center
    x_axis -= normal * np.dot(x_axis, normal)
    if np.linalg.norm(x_axis) < 0.02:
        return None
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(normal, x_axis)
    if np.linalg.norm(y_axis) < 1e-8:
        return None
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, normal)
    x_axis /= np.linalg.norm(x_axis)
    return replace(pose, rotation=np.column_stack((x_axis, y_axis, normal)))
