#include "grasp_hexapod_control_cpp/config_io.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <set>
#include <sstream>

namespace grasp_hexapod_control_cpp {

WorkspaceBoundary loadWorkspaceBoundary(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    throw ConfigError("cannot open workspace boundary: " + path);
  }
  WorkspaceBoundary boundary;
  std::string line;
  while (std::getline(file, line)) {
    const std::size_t first = line.find_first_not_of(" \t\r\n");
    if (first == std::string::npos || line[first] == '#') {
      continue;
    }
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream fields(line);
    double z = 0.0;
    double rho_min = 0.0;
    double rho_max = 0.0;
    if (!(fields >> z >> rho_min >> rho_max)) {
      throw ConfigError("malformed workspace boundary row: " + line);
    }
    boundary.z.push_back(z);
    boundary.rho_min.push_back(rho_min);
    boundary.rho_max.push_back(rho_max);
  }
  if (boundary.size() < 2) {
    throw ConfigError("workspace boundary must contain at least two rows");
  }
  return boundary;
}

double interpMonotonic(double x, const std::vector<double>& xs,
                       const std::vector<double>& ys) {
  // np.interp：x 越界时取端点值；否则在相邻两格之间线性插值。
  if (x <= xs.front()) {
    return ys.front();
  }
  if (x >= xs.back()) {
    return ys.back();
  }
  // upper_bound 对应 np.searchsorted(side='left') 的右邻格。
  const std::size_t upper =
      static_cast<std::size_t>(
          std::upper_bound(xs.begin(), xs.end(), x) - xs.begin());
  const std::size_t high = upper;
  const std::size_t low = high - 1;
  const double weight = (x - xs[low]) / (xs[high] - xs[low]);
  return ys[low] + weight * (ys[high] - ys[low]);
}

namespace {

Matrix4d rigidTransform(const Vector3d& translation, const Matrix3d& rotation) {
  Matrix4d result = Matrix4d::Identity();
  result.block<3, 3>(0, 0) = rotation;
  result.block<3, 1>(0, 3) = translation;
  return result;
}

const int kExpectedTagIds[] = {0, 1, 2, 3};

}  // namespace

DockSystem loadDockSystem(const std::string& path) {
  YAML::Node config;
  try {
    config = YAML::LoadFile(path);
  } catch (const YAML::Exception& error) {
    throw ConfigError("dock_system.yaml cannot be parsed: " + std::string(error.what()));
  }
  if (!config.IsMap()) {
    throw ConfigError("dock_system.yaml must be a mapping");
  }

  std::vector<int> tag_ids;
  if (config["tag_ids"]) {
    for (const YAML::Node& item : config["tag_ids"]) {
      tag_ids.push_back(item.as<int>());
    }
  }
  if (tag_ids.size() != 4 ||
      !std::equal(tag_ids.begin(), tag_ids.end(), kExpectedTagIds)) {
    throw ConfigError("dock_system.yaml tag_ids must be exactly 0..3");
  }

  const double tag_size = config["tag_size_m"].as<double>(0.0);
  if (config["tag_family"].as<std::string>("") != "tag36h11" || tag_size <= 0.0) {
    throw ConfigError("dock_system.yaml requires tag36h11 and positive size");
  }

  const YAML::Node descriptions = config["standalone_tags"];
  if (!descriptions.IsSequence()) {
    throw ConfigError("dock_system.yaml standalone_tags must match tag_ids/size");
  }
  std::map<int, double> sizes_by_id;
  std::map<int, std::string> tag_frames;
  for (const YAML::Node& item : descriptions) {
    sizes_by_id[item["id"].as<int>(-1)] = item["size"].as<double>(0.0);
    tag_frames[item["id"].as<int>()] = item["name"].as<std::string>("");
  }
  if (sizes_by_id.size() != 4 || tag_frames.size() != 4) {
    throw ConfigError("dock_system.yaml standalone_tags must match tag_ids/size");
  }
  for (const int tag_id : kExpectedTagIds) {
    if (std::abs(sizes_by_id[tag_id] - tag_size) > 1e-9 * std::max(1.0, std::abs(tag_size))) {
      throw ConfigError("dock_system.yaml standalone_tags must match tag_ids/size");
    }
  }
  std::set<std::string> frame_names;
  for (const auto& [tag_id, frame] : tag_frames) {
    (void)tag_id;
    if (frame.empty() || !frame_names.insert(frame).second) {
      throw ConfigError("dock_system.yaml tag names must be nonempty and unique");
    }
  }

  const YAML::Node lock = config["lock_from_camera"];
  Vector3d lock_translation = Vector3d::Zero();
  Matrix3d lock_rotation = Matrix3d::Identity();
  if (lock.IsMap() && lock["translation_m"] && lock["rotation"]) {
    for (int i = 0; i < 3; ++i) {
      lock_translation[i] = lock["translation_m"][i].as<double>();
    }
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        lock_rotation(i, j) = lock["rotation"][i][j].as<double>();
      }
    }
  } else {
    lock_translation = Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  }
  if (!lock_translation.allFinite() || !lock_rotation.allFinite()) {
    throw ConfigError("dock_system.yaml lock_from_camera must be finite");
  }
  if (!((lock_rotation.transpose() * lock_rotation - Matrix3d::Identity())
            .cwiseAbs()
            .maxCoeff() <= 1e-8)) {
    throw ConfigError("dock_system.yaml lock rotation must be orthogonal");
  }

  const YAML::Node pin_source = config["pin_from_tag_m"];
  std::map<int, Matrix4d> pin_from_tag;
  if (!pin_source.IsMap() || pin_source.size() != 4) {
    throw ConfigError("dock_system.yaml pin_from_tag_m ids must match tag_ids");
  }
  for (const int tag_id : kExpectedTagIds) {
    const YAML::Node entry = pin_source[std::to_string(tag_id)];
    if (!entry || !entry.IsSequence() || entry.size() != 3) {
      throw ConfigError("dock_system.yaml pin_from_tag_m must be finite xyz");
    }
    Vector3d translation = Vector3d::Zero();
    for (int i = 0; i < 3; ++i) {
      translation[i] = entry[i].as<double>();
    }
    if (!translation.allFinite()) {
      throw ConfigError("dock_system.yaml pin_from_tag_m must be finite xyz");
    }
    pin_from_tag[tag_id] = rigidTransform(translation, Matrix3d::Identity());
  }

  const YAML::Node calibrated = config["real_calibrated"];
  if (!calibrated || !calibrated.IsScalar()) {
    throw ConfigError("dock_system.yaml real_calibrated must be bool");
  }
  DockSystem system;
  system.path = path;
  system.tag_ids.assign(kExpectedTagIds, kExpectedTagIds + 4);
  system.tag_size_m = tag_size;
  system.tag_frames = std::move(tag_frames);
  system.lock_from_camera = rigidTransform(lock_translation, lock_rotation);
  system.pin_from_tag = std::move(pin_from_tag);
  system.real_calibrated = calibrated.as<bool>();
  return system;
}

}  // namespace grasp_hexapod_control_cpp
