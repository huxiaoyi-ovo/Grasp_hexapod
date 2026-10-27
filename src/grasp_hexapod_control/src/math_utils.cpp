#include "grasp_hexapod_control_cpp/math_utils.h"

#include <cmath>

namespace grasp_hexapod_control_cpp {

Matrix4d translation(double x, double y, double z) {
  Matrix4d transform = Matrix4d::Identity();
  transform(0, 3) = x;
  transform(1, 3) = y;
  transform(2, 3) = z;
  return transform;
}

Matrix4d rotationX(double angle) {
  const double cos_angle = std::cos(angle);
  const double sin_angle = std::sin(angle);
  Matrix4d transform = Matrix4d::Identity();
  transform(1, 1) = cos_angle;
  transform(1, 2) = -sin_angle;
  transform(2, 1) = sin_angle;
  transform(2, 2) = cos_angle;
  return transform;
}

Matrix4d rotationZ(double angle) {
  const double cos_angle = std::cos(angle);
  const double sin_angle = std::sin(angle);
  Matrix4d transform = Matrix4d::Identity();
  transform(0, 0) = cos_angle;
  transform(0, 1) = -sin_angle;
  transform(1, 0) = sin_angle;
  transform(1, 1) = cos_angle;
  return transform;
}

double wrapAngle(double angle) {
  const double remainder = std::fmod(angle + M_PI, 2.0 * M_PI);
  return (remainder < 0.0 ? remainder + 2.0 * M_PI : remainder) - M_PI;
}

bool poseToTransform(const Eigen::Quaterniond& quaternion,
                     const Vector3d& position, Matrix4d& transform_out) {
  const double norm = std::sqrt(quaternion.x() * quaternion.x() +
                                quaternion.y() * quaternion.y() +
                                quaternion.z() * quaternion.z() +
                                quaternion.w() * quaternion.w());
  if (norm == 0.0) {
    return false;
  }
  const double x = quaternion.x() / norm;
  const double y = quaternion.y() / norm;
  const double z = quaternion.z() / norm;
  const double w = quaternion.w() / norm;

  Matrix4d transform = Matrix4d::Identity();
  transform(0, 0) = 1.0 - 2.0 * (y * y + z * z);
  transform(0, 1) = 2.0 * (x * y - z * w);
  transform(0, 2) = 2.0 * (x * z + y * w);
  transform(1, 0) = 2.0 * (x * y + z * w);
  transform(1, 1) = 1.0 - 2.0 * (x * x + z * z);
  transform(1, 2) = 2.0 * (y * z - x * w);
  transform(2, 0) = 2.0 * (x * z - y * w);
  transform(2, 1) = 2.0 * (y * z + x * w);
  transform(2, 2) = 1.0 - 2.0 * (x * x + y * y);
  transform(0, 3) = position.x();
  transform(1, 3) = position.y();
  transform(2, 3) = position.z();
  if (!transform.allFinite()) {
    return false;
  }
  transform_out = transform;
  return true;
}

namespace {

Vector3d transformPointHomogeneous(const Matrix4d& transform,
                                   const Vector3d& point) {
  const Eigen::Vector4d homogeneous(point.x(), point.y(), point.z(), 1.0);
  const Eigen::Vector4d result = transform * homogeneous;
  return Vector3d(result[0], result[1], result[2]);
}

}  // namespace

std::vector<Vector3d> transformPoints(const Matrix4d& transform,
                                      const std::vector<Vector3d>& points) {
  std::vector<Vector3d> transformed;
  transformed.reserve(points.size());
  for (const Vector3d& point : points) {
    transformed.push_back(transformPointHomogeneous(transform, point));
  }
  return transformed;
}

std::vector<Vector2d> transformPoints2(const Matrix4d& transform,
                                       const std::vector<Vector2d>& points) {
  std::vector<Vector2d> transformed;
  transformed.reserve(points.size());
  for (const Vector2d& point : points) {
    const Vector3d result =
        transformPointHomogeneous(transform, Vector3d(point.x(), point.y(), 0.0));
    transformed.emplace_back(result.x(), result.y());
  }
  return transformed;
}

double segmentDistance(const Vector3d& a_start, const Vector3d& a_end,
                       const Vector3d& b_start, const Vector3d& b_end) {
  const Vector3d direction_a = a_end - a_start;
  const Vector3d direction_b = b_end - b_start;
  const Vector3d start_delta = a_start - b_start;

  const double length_a_sq = direction_a.dot(direction_a);
  const double length_b_sq = direction_b.dot(direction_b);
  const double direction_dot = direction_a.dot(direction_b);
  const double a_start_dot = direction_a.dot(start_delta);
  const double b_start_dot = direction_b.dot(start_delta);

  const double denominator = length_a_sq * length_b_sq - direction_dot * direction_dot;
  double scale_a;
  if (denominator > 1e-12) {
    scale_a = std::max(0.0, std::min(
                                1.0,
                                (direction_dot * b_start_dot - a_start_dot * length_b_sq) /
                                    denominator));
  } else {
    scale_a = 0.0;
  }

  double scale_b = (direction_dot * scale_a + b_start_dot) / length_b_sq;

  // 如果 B 的最近点落在线段外，把它固定在端点，再重新求 A 线段上的最近点。
  if (scale_b < 0.0) {
    scale_b = 0.0;
    scale_a = std::max(0.0, std::min(1.0, -a_start_dot / length_a_sq));
  } else if (scale_b > 1.0) {
    scale_b = 1.0;
    scale_a =
        std::max(0.0, std::min(1.0, (direction_dot - a_start_dot) / length_a_sq));
  }

  const Vector3d closest_a = a_start + scale_a * direction_a;
  const Vector3d closest_b = b_start + scale_b * direction_b;
  return (closest_a - closest_b).norm();
}

double polygonArea(const std::vector<Vector2d>& polygon) {
  if (polygon.size() < 3) {
    return 0.0;
  }
  double area = 0.0;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const Vector2d& current = polygon[i];
    const Vector2d& next = polygon[(i + 1) % polygon.size()];
    area += current.x() * next.y() - current.y() * next.x();
  }
  return 0.5 * area;
}

std::vector<double> distanceToPolygonBoundary(
    const std::vector<Vector2d>& points,
    const std::vector<Vector2d>& polygon) {
  std::vector<Vector2d> segment_start;
  std::vector<Vector2d> segment;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const Vector2d& start = polygon[i];
    const Vector2d& end = polygon[(i + 1) % polygon.size()];
    const Vector2d delta = end - start;
    if (delta.squaredNorm() > 1e-15) {
      segment_start.push_back(start);
      segment.push_back(delta);
    }
  }

  std::vector<double> distances;
  distances.reserve(points.size());
  if (segment_start.empty()) {
    distances.assign(points.size(), std::numeric_limits<double>::infinity());
    return distances;
  }

  for (const Vector2d& point : points) {
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < segment_start.size(); ++i) {
      const double projection = (point - segment_start[i]).dot(segment[i]) /
                                segment[i].squaredNorm();
      const double clipped = std::max(0.0, std::min(1.0, projection));
      const Vector2d closest = segment_start[i] + clipped * segment[i];
      best = std::min(best, (point - closest).norm());
    }
    distances.push_back(best);
  }
  return distances;
}

std::vector<bool> pointsInPolygon(const std::vector<Vector2d>& points,
                                  const std::vector<Vector2d>& polygon) {
  std::vector<bool> inside(points.size(), false);
  if (polygon.empty()) {
    return inside;
  }

  for (std::size_t p = 0; p < points.size(); ++p) {
    const double x = points[p].x();
    const double y = points[p].y();
    bool point_inside = false;
    Vector2d previous = polygon.back();
    for (const Vector2d& current : polygon) {
      const bool crosses =
          ((current.y() > y) != (previous.y() > y)) &&
          (x < (previous.x() - current.x()) * (y - current.y()) /
                       (previous.y() - current.y() + 1e-15) +
                   current.x());
      if (crosses) {
        point_inside = !point_inside;
      }
      previous = current;
    }
    inside[p] = point_inside;
  }

  // 落在边界上的点（<=1e-9）也视为内部。
  const std::vector<double> boundary =
      distanceToPolygonBoundary(points, polygon);
  for (std::size_t p = 0; p < points.size(); ++p) {
    inside[p] = inside[p] || boundary[p] <= 1e-9;
  }
  return inside;
}

std::vector<Vector2d> sampleSegment(const Vector2d& start, const Vector2d& end,
                                    double spacing) {
  const double distance = (end - start).norm();
  const int sample_count =
      std::max(2, static_cast<int>(std::ceil(distance / spacing)) + 1);
  std::vector<Vector2d> samples;
  samples.reserve(sample_count);
  const double step = 1.0 / (sample_count - 1);
  for (int i = 0; i < sample_count; ++i) {
    const double ratio = i * step;
    samples.push_back(start + ratio * (end - start));
  }
  return samples;
}

}  // namespace grasp_hexapod_control_cpp
