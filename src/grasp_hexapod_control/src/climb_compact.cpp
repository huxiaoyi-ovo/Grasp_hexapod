#include "grasp_hexapod_control_cpp/climb_compact.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <set>
#include <unordered_map>

namespace grasp_hexapod_control_cpp {

using nlohmann::json;

Matrix4d worldFromBase(const BasePose& base) {
  const double x = base[0], y = base[1], z = base[2];
  const double roll = base[3], pitch = base[4], yaw = base[5];
  const double cosine_r = std::cos(roll), sine_r = std::sin(roll);
  const double cosine_p = std::cos(pitch), sine_p = std::sin(pitch);
  Matrix3d rotation_x;
  rotation_x << 1.0, 0.0, 0.0, 0.0, cosine_r, -sine_r, 0.0, sine_r, cosine_r;
  Matrix3d rotation_y;
  rotation_y << cosine_p, 0.0, sine_p, 0.0, 1.0, 0.0, -sine_p, 0.0, cosine_p;
  Matrix4d output = Matrix4d::Identity();
  if (yaw == 0.0) {
    // 与历史 5 元素实现逐位一致（Rz(0) 是精确单位阵）。
    output.block<3, 3>(0, 0) = rotation_y * rotation_x;
  } else {
    const double cosine_y = std::cos(yaw), sine_y = std::sin(yaw);
    Matrix3d rotation_z;
    rotation_z << cosine_y, -sine_y, 0.0, sine_y, cosine_y, 0.0, 0.0, 0.0, 1.0;
    // Python: rotation_z @ rotation_y @ rotation_x（自左向右结合）。
    output.block<3, 3>(0, 0) = (rotation_z * rotation_y) * rotation_x;
  }
  output(0, 3) = x;
  output(1, 3) = y;
  output(2, 3) = z;
  return output;
}

Matrix4d worldFromBase(const std::array<double, 5>& base) {
  return worldFromBase(BasePose{base[0], base[1], base[2], base[3], base[4], 0.0});
}

namespace {

bool allCloseAbs(double a, double b, double atol) {
  return std::abs(a - b) <= atol;
}

// ---------------------------------------------------------------------------
// JSON 读取辅助：形状/有限性与 Python _array 相同的错误消息。
// ---------------------------------------------------------------------------

[[noreturn]] void throwInvalidField(const std::string& path) {
  throw ConfigError("invalid compact field: " + path);
}

const json& atPath(const json& config, const std::string& path) {
  const json* value = &config;
  std::string key;
  std::size_t start = 0;
  while (start <= path.size()) {
    const std::size_t dot = path.find('.', start);
    key = path.substr(start, dot == std::string::npos ? std::string::npos : dot - start);
    if (!value->contains(key)) {
      throwInvalidField(path);
    }
    value = &(*value)[key];
    if (dot == std::string::npos) {
      break;
    }
    start = dot + 1;
  }
  return *value;
}

double finiteNumberAt(const json& config, const std::string& path) {
  const json& value = atPath(config, path);
  if (!value.is_number()) {
    throwInvalidField(path);
  }
  const double number = value.get<double>();
  if (!std::isfinite(number)) {
    throwInvalidField(path);
  }
  return number;
}

// 读取机身位姿：front 为 6 元素，侧向为 5 元素（yaw 存 0）。
BasePose poseAt(const json& config, const std::string& path, bool front) {
  const json& value = atPath(config, path);
  const std::size_t expected = front ? 6 : 5;
  if (!value.is_array() || value.size() != expected) {
    throwInvalidField(path);
  }
  BasePose result{};
  for (std::size_t i = 0; i < expected; ++i) {
    if (!value[i].is_number() || !std::isfinite(value[i].get<double>())) {
      throwInvalidField(path);
    }
    result[i] = value[i].get<double>();
  }
  return result;
}

Vector3d vector3At(const json& value, const std::string& path) {
  if (!value.is_array() || value.size() != 3) {
    throwInvalidField(path);
  }
  Vector3d result = Vector3d::Zero();
  for (int i = 0; i < 3; ++i) {
    if (!value[i].is_number() || !std::isfinite(value[i].get<double>())) {
      throwInvalidField(path);
    }
    result[i] = value[i].get<double>();
  }
  return result;
}

JointAngles joints63At(const json& config, const std::string& path) {
  const json& value = atPath(config, path);
  if (!value.is_array() || value.size() != kLegCount) {
    throwInvalidField(path);
  }
  JointAngles result{};
  for (int leg = 0; leg < kLegCount; ++leg) {
    result[leg] = vector3At(value[leg], path);
  }
  return result;
}

std::array<Vector3d, 6> anchors6At(const json& value, const std::string& path) {
  if (!value.is_array() || value.size() != kLegCount) {
    throwInvalidField(path);
  }
  std::array<Vector3d, 6> result{};
  for (int leg = 0; leg < kLegCount; ++leg) {
    result[leg] = vector3At(value[leg], path);
  }
  return result;
}

// 宽松读取：只要求“每个点是 3 个有限数”；整体形状由校验器判定。
// 行数/列数不合法时返回 false（对应 numpy ndim/shape 检查失败）。
bool readLooseKnotRows(const json& value, std::vector<std::array<Vector3d, 6>>& rows) {
  if (!value.is_array()) {
    return false;
  }
  rows.clear();
  for (const json& knot : value) {
    if (!knot.is_array() || knot.size() != kLegCount) {
      return false;
    }
    std::array<Vector3d, 6> row{};
    for (int leg = 0; leg < kLegCount; ++leg) {
      if (!knot[leg].is_array() || knot[leg].size() != 3) {
        return false;
      }
      for (int i = 0; i < 3; ++i) {
        if (!knot[leg][i].is_number() || !std::isfinite(knot[leg][i].get<double>())) {
          return false;
        }
        row[leg][i] = knot[leg][i].get<double>();
      }
    }
    rows.push_back(row);
  }
  return true;
}

bool readLoosePointRows(const json& value, std::vector<std::vector<Vector3d>>& rows) {
  if (!value.is_array()) {
    return false;
  }
  rows.clear();
  for (const json& row : value) {
    if (!row.is_array()) {
      return false;
    }
    std::vector<Vector3d> points;
    for (const json& point : row) {
      if (!point.is_array() || point.size() != 3) {
        return false;
      }
      Vector3d parsed = Vector3d::Zero();
      for (int i = 0; i < 3; ++i) {
        if (!point[i].is_number() || !std::isfinite(point[i].get<double>())) {
          return false;
        }
        parsed[i] = point[i].get<double>();
      }
      points.push_back(parsed);
    }
    rows.push_back(points);
  }
  return true;
}

// 解析一个阶段；类型级非法抛 "invalid compact stage fields: name"。
CompactStage parseStage(const json& stage, int index, bool front) {
  if (!stage.is_object()) {
    throw ConfigError("compact stage is invalid");
  }
  CompactStage parsed;
  parsed.name = stage.contains("name") && stage["name"].is_string()
                    ? stage["name"].get<std::string>()
                    : std::string();
  if (parsed.name.empty()) {
    throw ConfigError("compact stage is invalid");
  }
  parsed.pose_start = poseAt(stage, "pose_start", front);
  parsed.pose_end = poseAt(stage, "pose_end", front);

  const std::string prefix = "stages." + std::to_string(index) + ".";
  const json& knots = stage.contains("anchor_knots") ? stage["anchor_knots"] : json();
  if (!readLooseKnotRows(knots, parsed.anchor_knots)) {
    throw ConfigError("invalid compact stage fields: " + parsed.name);
  }
  if (stage.contains("segment_durations_s")) {
    const json& durations = stage["segment_durations_s"];
    if (durations.is_array()) {
      for (const json& item : durations) {
        if (!item.is_number()) {
          throw ConfigError("invalid compact stage fields: " + parsed.name);
        }
        parsed.segment_durations_s.push_back(item.get<double>());
      }
    } else {
      throw ConfigError("invalid compact stage fields: " + parsed.name);
    }
  }
  if (stage.contains("active_legs")) {
    const json& active = stage["active_legs"];
    if (active.is_array()) {
      parsed.active_legs_present = true;
      for (const json& item : active) {
        if (!item.is_number_integer()) {
          throw ConfigError("invalid compact stage fields: " + parsed.name);
        }
        parsed.active_legs.push_back(item.get<int>());
      }
    } else {
      throw ConfigError("invalid compact stage fields: " + parsed.name);
    }
  }
  if (stage.contains("anchor_curve") && stage["anchor_curve"].is_string()) {
    parsed.anchor_curve = stage["anchor_curve"].get<std::string>();
  }
  if (stage.contains("pose_curve") && stage["pose_curve"].is_string()) {
    parsed.pose_curve = stage["pose_curve"].get<std::string>();
  }
  if (stage.contains("settle_s")) {
    if (!stage["settle_s"].is_number()) {
      throw ConfigError("invalid compact stage fields: " + parsed.name);
    }
    parsed.settle_s = stage["settle_s"].get<double>();
  }
  if (stage.contains("settle_persistence_s")) {
    if (!stage["settle_persistence_s"].is_number()) {
      throw ConfigError("invalid compact stage fields: " + parsed.name);
    }
    parsed.has_settle_persistence = true;
    parsed.settle_persistence_s = stage["settle_persistence_s"].get<double>();
  }
  if (stage.contains("relative_swing_height_m")) {
    if (!stage["relative_swing_height_m"].is_number()) {
      throw ConfigError("invalid compact stage fields: " + parsed.name);
    }
    parsed.relative_swing_height_m = stage["relative_swing_height_m"].get<double>();
  }
  if (stage.contains("active_base_knots_m")) {
    if (!readLoosePointRows(stage["active_base_knots_m"], parsed.active_base_knots_m)) {
      throw ConfigError("invalid compact stage fields: " + parsed.name);
    }
    parsed.has_active_base_knots = true;
  }
  if (stage.contains("active_base_velocities_m_s")) {
    if (!readLoosePointRows(stage["active_base_velocities_m_s"],
                            parsed.active_base_velocities_m_s)) {
      throw ConfigError("invalid compact stage fields: " + parsed.name);
    }
    parsed.has_active_base_velocities = true;
  }
  if (stage.contains("pose_knots") && !stage["pose_knots"].is_null()) {
    parsed.has_pose_knots = true;
    const json& rows = stage["pose_knots"];
    bool wellformed = rows.is_array();
    if (wellformed) {
      for (const json& row : rows) {
        if (!row.is_array() || row.size() != 6) {
          wellformed = false;
          break;
        }
        BasePose pose{};
        for (int i = 0; i < 6; ++i) {
          if (!row[i].is_number() || !std::isfinite(row[i].get<double>())) {
            wellformed = false;
            break;
          }
          pose[i] = row[i].get<double>();
        }
        if (!wellformed) {
          break;
        }
        parsed.pose_knots.push_back(pose);
      }
    }
    parsed.pose_knots_wellformed = wellformed;
    if (!wellformed) {
      parsed.pose_knots.clear();
    }
  }
  for (const char* key : {"support_legs", "airborne_legs_start", "airborne_legs_end",
                          "segment_support_legs"}) {
    if (stage.contains(key)) {
      parsed.contact_roles[key] = stage[key];
    }
  }
  if (stage.contains("continuous_air_transition")) {
    if (!stage["continuous_air_transition"].is_boolean()) {
      throw ConfigError("continuous air transition marker must be bool");
    }
    parsed.continuous_air_transition = stage["continuous_air_transition"].get<bool>();
  }
  return parsed;
}

}  // namespace

std::vector<int> CompactStage::roleLegs(const std::string& key) const {
  std::vector<int> legs;
  const auto found = contact_roles.find(key);
  if (found != contact_roles.end() && found->second.is_array()) {
    for (const json& item : found->second) {
      legs.push_back(item.get<int>());
    }
  }
  return legs;
}

bool CompactStage::hasSegmentSupportLegs() const {
  const auto found = contact_roles.find("segment_support_legs");
  return found != contact_roles.end() && !found->second.is_null();
}

std::vector<int> CompactStage::segmentSupportLegs(std::size_t segment) const {
  std::vector<int> legs;
  const auto found = contact_roles.find("segment_support_legs");
  if (found != contact_roles.end() && found->second.is_array() &&
      segment < found->second.size() && found->second[segment].is_array()) {
    for (const json& item : found->second[segment]) {
      legs.push_back(item.get<int>());
    }
  }
  return legs;
}

ClimbCompactConfig climbCompactFromJson(const json& document) {
  if (!document.is_object()) {
    throw ConfigError("invalid simulation-only compact climb config");
  }
  ClimbCompactConfig config;
  if (document.contains("schema") && document["schema"].is_string()) {
    config.schema = document["schema"].get<std::string>();
  }
  config.simulation_only =
      document.contains("simulation_only") && document["simulation_only"].is_boolean() &&
      document["simulation_only"].get<bool>();
  config.simulation_candidate_only =
      document.contains("simulation_candidate_only") &&
      document["simulation_candidate_only"].is_boolean() &&
      document["simulation_candidate_only"].get<bool>();
  {
    const json& value = atPath(document, "xiaolan_translation");
    for (int i = 0; i < 3; ++i) {
      if (!value[i].is_number()) {
        throwInvalidField("xiaolan_translation");
      }
    }
    config.xiaolan_translation = Vector3d(
        value[0].get<double>(), value[1].get<double>(), value[2].get<double>());
  }
  config.p0.q_rad = joints63At(document, "p0.q_rad");
  if (document.contains("climb_orientation") && document["climb_orientation"].is_string()) {
    config.climb_orientation = document["climb_orientation"].get<std::string>();
  }
  const bool front = config.isFront();
  if (document.contains("contact_plan_version") &&
      !document["contact_plan_version"].is_null()) {
    const json& version = document["contact_plan_version"];
    config.contact_plan_version_present = true;
    // Python: True == 1 与 1.0 == 1 均成立，其余（字符串/列表/其他数值）不等于 1。
    if (version.is_boolean()) {
      config.contact_plan_version_is_one = version.get<bool>();
    } else if (version.is_number()) {
      config.contact_plan_version_is_one = version.get<double>() == 1.0;
    }
  }
  if (document.contains("simulation_joint_velocity_limit_rad_s") &&
      !document["simulation_joint_velocity_limit_rad_s"].is_null()) {
    const json& cap = document["simulation_joint_velocity_limit_rad_s"];
    if (cap.is_number() && !cap.is_boolean()) {
      config.simulation_joint_velocity_limit_rad_s = cap.get<double>();
    } else {
      config.simulation_joint_velocity_limit_malformed = true;
    }
  }
  {
    const json& value = atPath(document, "p0.base");
    const std::size_t expected = front ? 6 : 4;
    if (!value.is_array() || value.size() != expected) {
      throwInvalidField("p0.base");
    }
    std::array<double, 6> parsed{};
    for (std::size_t i = 0; i < expected; ++i) {
      if (!value[i].is_number() || !std::isfinite(value[i].get<double>())) {
        throwInvalidField("p0.base");
      }
      parsed[i] = value[i].get<double>();
    }
    if (front) {
      config.p0.pose = parsed;
    } else {
      for (int i = 0; i < 4; ++i) {
        config.p0.base[i] = parsed[i];
      }
      config.p0.pose = BasePose{parsed[0], parsed[1], parsed[2], 0.0, parsed[3], 0.0};
    }
  }
  config.p0.anchors_world_m = anchors6At(atPath(document, "p0.anchors_world_m"),
                                         "p0.anchors_world_m");
  config.terminal_q_rad = joints63At(document, "terminal_q_rad");

  if (document.contains("visual_validation_deferred_for_sim_finish") &&
      document["visual_validation_deferred_for_sim_finish"].is_array()) {
    for (const json& item : document["visual_validation_deferred_for_sim_finish"]) {
      if (!item.is_string()) {
        throw ConfigError("compact visual validation list is invalid");
      }
      config.visual_validation_deferred_for_sim_finish.push_back(item.get<std::string>());
    }
  }

  const json& stages = document.contains("stages") ? document["stages"] : json();
  if (stages.is_array()) {
    for (int i = 0; i < static_cast<int>(stages.size()); ++i) {
      config.stages.push_back(parseStage(stages[i], i, front));
    }
  }
  config.stage_count = document.contains("stage_count") &&
                               document["stage_count"].is_number_integer()
                           ? document["stage_count"].get<int>()
                           : -1;

  const json& gate = atPath(document, "settle_gate");
  auto gateNumber = [&gate](const char* key) {
    if (!gate.contains(key) || !gate[key].is_number()) {
      throw ConfigError("compact settle gate field is missing");
    }
    return gate[key].get<double>();
  };
  config.settle_gate.max_joint_tracking_error_rad = gateNumber("max_joint_tracking_error_rad");
  config.settle_gate.max_foot_target_error_m = gateNumber("max_foot_target_error_m");
  config.settle_gate.entry_max_joint_error_rad = gateNumber("entry_max_joint_error_rad");
  config.settle_gate.persistence_s = gateNumber("persistence_s");
  config.settle_gate.timeout_s = gateNumber("timeout_s");
  config.settle_gate.command_tracking_only_not_contact_proof =
      gate.contains("command_tracking_only_not_contact_proof") &&
      gate["command_tracking_only_not_contact_proof"].is_boolean() &&
      gate["command_tracking_only_not_contact_proof"].get<bool>();
  auto gateBool = [&gate](const char* key, bool& out) {
    if (!gate.contains(key) || !gate[key].is_boolean()) {
      throw ConfigError("compact settle gate flags must be bool");
    }
    out = gate[key].get<bool>();
  };
  gateBool("preview_time_only_stage_advance", config.settle_gate.preview_time_only_stage_advance);
  gateBool("tracking_errors_diagnostic_only", config.settle_gate.tracking_errors_diagnostic_only);
  return config;
}

ClimbCompactConfig loadClimbCompact(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    throw ConfigError("cannot open compact climb config: " + path);
  }
  json document;
  try {
    file >> document;
  } catch (const json::exception& error) {
    throw ConfigError("compact climb config is not valid JSON: " +
                      std::string(error.what()));
  }
  return climbCompactFromJson(document);
}

namespace {

bool allClosePose(const BasePose& a, const BasePose& b, double atol) {
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (!allCloseAbs(a[i], b[i], atol)) {
      return false;
    }
  }
  return true;
}

// Python: 非负整数腿号、范围 [0, 6)、无重复。
bool parseLegList(const json& value, std::vector<int>& legs) {
  legs.clear();
  if (!value.is_array()) {
    return false;
  }
  std::set<int> unique;
  for (const json& item : value) {
    if (!item.is_number_integer()) {
      return false;
    }
    const long long leg = item.get<long long>();
    if (leg < 0 || leg >= kLegCount) {
      return false;
    }
    legs.push_back(static_cast<int>(leg));
    unique.insert(static_cast<int>(leg));
  }
  return unique.size() == legs.size();
}

std::set<int> toSet(const std::vector<int>& legs) {
  return std::set<int>(legs.begin(), legs.end());
}

bool isSubset(const std::set<int>& small, const std::set<int>& large) {
  return std::includes(large.begin(), large.end(), small.begin(), small.end());
}

bool intersects(const std::set<int>& a, const std::set<int>& b) {
  for (int value : a) {
    if (b.count(value) != 0) {
      return true;
    }
  }
  return false;
}

constexpr const char* kRoleKeys[3] = {"support_legs", "airborne_legs_start",
                                      "airborne_legs_end"};

}  // namespace

void validateClimbCompact(const ClimbCompactConfig& config) {
  if (config.schema != kCompactSchema || !config.simulation_only ||
      !config.simulation_candidate_only) {
    throw ConfigError("invalid simulation-only compact climb config");
  }
  const JointAngles q_stand = makeQStand();
  for (int leg = 0; leg < kLegCount; ++leg) {
    if (!(config.p0.q_rad[leg].array() == q_stand[leg].array()).all()) {
      throw ConfigError("compact P0 must be Q_STAND");
    }
  }
  const bool is_front = config.isFront();
  const bool contact_plan_v1 = config.contact_plan_version_is_one;
  bool has_role_metadata = false;
  for (const CompactStage& stage : config.stages) {
    if (!stage.contact_roles.empty()) {
      has_role_metadata = true;
    }
  }
  if (config.contact_plan_version_present && !contact_plan_v1) {
    throw ConfigError("unsupported contact plan version");
  }
  if (contact_plan_v1 && !is_front) {
    throw ConfigError("planned contact roles are front-only");
  }
  if (!contact_plan_v1 && has_role_metadata) {
    throw ConfigError("planned contact roles require contact_plan_version=1");
  }
  if (config.simulation_joint_velocity_limit_rad_s.has_value() ||
      config.simulation_joint_velocity_limit_malformed) {
    const double cap = config.simulation_joint_velocity_limit_rad_s.value_or(0.0);
    if (!is_front || config.simulation_joint_velocity_limit_malformed ||
        !std::isfinite(cap) || !(cap > 0.0 && cap <= 8.0)) {
      throw ConfigError("invalid front simulation joint velocity cap");
    }
  }
  if (is_front) {
    // np.isclose(yaw, -pi/2): |a-b| <= 1e-8 + 1e-5*|b|。
    const double target_yaw = -M_PI / 2.0;
    if (!(std::abs(config.p0.pose[5] - target_yaw) <=
          1e-8 + 1e-5 * std::abs(target_yaw))) {
      throw ConfigError("front compact P0 must face Xiaolan along +x");
    }
  }
  if (config.stages.empty() || config.stage_count != static_cast<int>(config.stages.size())) {
    throw ConfigError("compact stage list is invalid");
  }

  BasePose previous_pose = config.p0.pose;
  std::array<Vector3d, 6> previous_anchors = config.p0.anchors_world_m;
  std::vector<int> previous_airborne_end;

  std::set<std::string> names;
  for (std::size_t stage_ordinal = 0; stage_ordinal < config.stages.size(); ++stage_ordinal) {
    const CompactStage& stage = config.stages[stage_ordinal];
    names.insert(stage.name);

    const std::size_t knot_count = stage.anchor_knots.size();
    const auto knotsShapeValid = [&]() {
      return knot_count >= 2 && stage.segment_durations_s.size() == knot_count - 1;
    };
    const bool durationsValid = [&]() {
      for (double duration : stage.segment_durations_s) {
        if (!std::isfinite(duration) || duration <= 0.0) {
          return false;
        }
      }
      return true;
    }();
    const bool activeValid = [&]() {
      if (!stage.active_legs_present) {
        return false;
      }
      std::set<int> unique;
      for (int leg : stage.active_legs) {
        if (leg < 0 || leg >= kLegCount || !unique.insert(leg).second) {
          return false;
        }
      }
      return true;
    }();
    const bool poseCurveValid = stage.pose_curve == "quintic_full_stage" ||
                                stage.pose_curve == "quintic_first_segment" ||
                                stage.pose_curve == "piecewise_quintic";

    const bool piecewise_pose_curve =
        stage.pose_curve == "piecewise_quintic" && is_front &&
        (stage.anchor_curve == "piecewise_quintic" ||
         stage.anchor_curve == "piecewise_base_quintic") &&
        stage.has_pose_knots && stage.pose_knots_wellformed &&
        stage.pose_knots.size() == knot_count &&
        allClosePose(stage.pose_knots.front(), stage.pose_start, 1e-12) &&
        allClosePose(stage.pose_knots.back(), stage.pose_end, 1e-12);

    const bool base_piecewise_curve = [&]() {
      if (stage.anchor_curve != "piecewise_base_quintic" ||
          stage.active_legs.empty()) {
        return false;
      }
      if (stage.active_base_knots_m.size() != knot_count) {
        return false;
      }
      for (const std::vector<Vector3d>& row : stage.active_base_knots_m) {
        if (row.size() != stage.active_legs.size()) {
          return false;
        }
      }
      if (stage.has_active_base_velocities) {
        if (stage.active_base_velocities_m_s.size() != stage.active_base_knots_m.size()) {
          return false;
        }
        for (std::size_t i = 0; i < stage.active_base_velocities_m_s.size(); ++i) {
          if (stage.active_base_velocities_m_s[i].size() !=
              stage.active_base_knots_m[i].size()) {
            return false;
          }
        }
      }
      return true;
    }();
    if (stage.has_active_base_velocities && stage.anchor_curve != "piecewise_base_quintic") {
      throw ConfigError("base velocities require base-piecewise curve");
    }

    const bool relative_curve = [&]() {
      return stage.anchor_curve == "relative_base_high_step" && knot_count == 2 &&
             stage.segment_durations_s.size() == 1 && stage.relative_swing_height_m &&
             std::isfinite(*stage.relative_swing_height_m) &&
             *stage.relative_swing_height_m > 0.0 && !stage.active_legs.empty();
    }();

    const bool first_segment_pose_curve =
        stage.pose_curve == "quintic_first_segment" && base_piecewise_curve &&
        (stage.active_legs.size() == 1 ||
         (is_front && (stage.active_legs == std::vector<int>{0, 3} ||
                       stage.active_legs == std::vector<int>{1, 4} ||
                       stage.active_legs == std::vector<int>{2, 5}))) &&
        stage.segment_durations_s.size() > 1;

    const bool settle_persistence_valid =
        !stage.has_settle_persistence ||
        (std::isfinite(stage.settle_persistence_s) && stage.settle_persistence_s > 0.0);

    const bool anchor_curve_ok =
        (stage.anchor_curve == "piecewise_quintic" &&
         (stage.active_legs.empty() || is_front)) ||
        base_piecewise_curve || relative_curve;

    if (!knotsShapeValid() || !durationsValid || !activeValid || !poseCurveValid ||
        (stage.has_pose_knots && !piecewise_pose_curve) ||
        (stage.pose_curve == "piecewise_quintic" && !piecewise_pose_curve) ||
        (stage.pose_curve == "quintic_first_segment" && !first_segment_pose_curve) ||
        !anchor_curve_ok || !std::isfinite(stage.settle_s) || stage.settle_s <= 0.0 ||
        !settle_persistence_valid) {
      throw ConfigError("invalid compact stage fields: " + stage.name);
    }

    if (contact_plan_v1) {
      for (const char* key : kRoleKeys) {
        if (stage.contact_roles.find(key) == stage.contact_roles.end()) {
          throw ConfigError("planned contact roles are incomplete: " + stage.name);
        }
      }
      std::vector<int> support_legs;
      std::vector<int> air_start_legs;
      std::vector<int> air_end_legs;
      std::vector<int>* const role_targets[3] = {&support_legs, &air_start_legs,
                                                 &air_end_legs};
      for (int k = 0; k < 3; ++k) {
        if (!parseLegList(stage.contact_roles.at(kRoleKeys[k]), *role_targets[k])) {
          throw ConfigError("invalid planned role list: " + stage.name);
        }
      }
      const std::set<int> active_set = toSet(stage.active_legs);
      const std::set<int> support_set = toSet(support_legs);
      const std::set<int> air_start_set = toSet(air_start_legs);
      const std::set<int> air_end_set = toSet(air_end_legs);
      std::set<int> expected_support;
      for (int leg = 0; leg < kLegCount; ++leg) {
        if (active_set.count(leg) == 0) {
          expected_support.insert(leg);
        }
      }
      if (support_set != expected_support || support_set.size() < 3 ||
          !isSubset(air_start_set, active_set) || !isSubset(air_end_set, active_set) ||
          (stage_ordinal == 0 && !air_start_set.empty()) ||
          (stage_ordinal > 0 && air_start_set != toSet(previous_airborne_end))) {
        throw ConfigError("invalid planned contact roles: " + stage.name);
      }
      if (intersects(air_start_set, air_end_set) &&
          stage.anchor_curve != "piecewise_base_quintic") {
        throw ConfigError("planned carried air requires base-piecewise curve: " + stage.name);
      }
      for (const std::array<Vector3d, 6>& knot : stage.anchor_knots) {
        for (int leg : support_legs) {
          for (int i = 0; i < 3; ++i) {
            if (!allCloseAbs(knot[leg][i], stage.anchor_knots.front()[leg][i], 1e-12)) {
              throw ConfigError("planned support anchors must remain world-fixed: " +
                                stage.name);
            }
          }
        }
      }
      if (stage.hasSegmentSupportLegs()) {
        const json& table = stage.contact_roles.at("segment_support_legs");
        if (!table.is_array() || table.size() != stage.segment_durations_s.size()) {
          throw ConfigError("invalid segment support table: " + stage.name);
        }
        for (std::size_t segment = 0; segment < table.size(); ++segment) {
          std::vector<int> declared;
          bool valid = parseLegList(table[segment], declared) && declared.size() >= 3 &&
                       isSubset(support_set, toSet(declared));
          if (valid) {
            for (int leg : declared) {
              for (int i = 0; i < 3; ++i) {
                if (!allCloseAbs(stage.anchor_knots[segment][leg][i],
                                 stage.anchor_knots[segment + 1][leg][i], 1e-12)) {
                  valid = false;
                }
              }
            }
          }
          if (!valid) {
            throw ConfigError("invalid segment support table: " + stage.name);
          }
        }
        if (intersects(toSet(stage.segmentSupportLegs(0)), air_start_set)) {
          throw ConfigError("segment support overlaps start air: " + stage.name);
        }
        if (intersects(toSet(stage.segmentSupportLegs(table.size() - 1)), air_end_set)) {
          throw ConfigError("segment support overlaps end air: " + stage.name);
        }
      }
      previous_airborne_end = air_end_legs;
    }

    if (!allClosePose(stage.pose_start, previous_pose, 1e-12)) {
      throw ConfigError("compact stage boundary mismatch: " + stage.name);
    }
    for (int leg = 0; leg < kLegCount; ++leg) {
      for (int i = 0; i < 3; ++i) {
        if (!allCloseAbs(stage.anchor_knots.front()[leg][i], previous_anchors[leg][i], 1e-12)) {
          throw ConfigError("compact stage boundary mismatch: " + stage.name);
        }
      }
    }

    if (base_piecewise_curve) {
      const std::vector<int> active = stage.active_legs;
      const auto checkEndpoint = [&](const std::array<Vector3d, 6>& endpoint,
                                     const BasePose& endpoint_pose, int knot_index) {
        const Matrix4d world = worldFromBase(endpoint_pose);
        const std::vector<Vector3d>& base_row =
            stage.active_base_knots_m[knot_index < 0
                                         ? stage.active_base_knots_m.size() + knot_index
                                         : static_cast<std::size_t>(knot_index)];
        for (std::size_t slot = 0; slot < active.size(); ++slot) {
          const Eigen::Vector4d homogeneous(base_row[slot].x(), base_row[slot].y(),
                                            base_row[slot].z(), 1.0);
          const Eigen::Vector4d transformed = world * homogeneous;
          for (int i = 0; i < 3; ++i) {
            if (!allCloseAbs(transformed[i], endpoint[active[slot]][i], 1e-9)) {
              throw ConfigError("base-relative endpoint mismatch: " + stage.name);
            }
          }
        }
      };
      checkEndpoint(stage.anchor_knots.front(), stage.pose_start, 0);
      checkEndpoint(stage.anchor_knots.back(), stage.pose_end, -1);
    }

    previous_pose = stage.pose_end;
    previous_anchors = stage.anchor_knots.back();
  }
  if (names.size() != config.stages.size()) {
    throw ConfigError("compact stage names must be unique");
  }
  if (contact_plan_v1 && !previous_airborne_end.empty()) {
    throw ConfigError("planned airborne legs require a following stage");
  }

  // 连续腾空过渡校验：侧向为冻结的 LM 对；front 按 contact_plan 角色逐腿校验。
  for (std::size_t index = 0; index < config.stages.size(); ++index) {
    const CompactStage& stage = config.stages[index];
    if (!stage.continuous_air_transition) {
      continue;
    }
    if (index + 1 >= config.stages.size()) {
      throw ConfigError("continuous air transition requires next stage");
    }
    const CompactStage& next_stage = config.stages[index + 1];
    if (!is_front) {
      const std::vector<int> frozen = {2};
      const std::string pair_a = stage.name;
      const std::string pair_b = next_stage.name;
      const bool pair_ok =
          (pair_a == "LM_LIFT" && pair_b == "BODY_ADVANCE_LM_AIR") ||
          (pair_a == "BODY_ADVANCE_LM_AIR" && pair_b == "LM_LEFT_FINAL_LAND");
      if (stage.active_legs != frozen || next_stage.active_legs != frozen || !pair_ok) {
        throw ConfigError("continuous air transition requires frozen LM pair");
      }
      for (int i = 0; i < 6; ++i) {
        if (!allCloseAbs(stage.pose_end[i], next_stage.pose_start[i], 1e-12)) {
          throw ConfigError("continuous air transition boundary mismatch");
        }
      }
      const std::array<int, 5> non_lm = {0, 1, 3, 4, 5};
      for (int leg : non_lm) {
        for (int i = 0; i < 3; ++i) {
          if (!allCloseAbs(stage.anchor_knots.back()[leg][i],
                           next_stage.anchor_knots.front()[leg][i], 1e-12)) {
            throw ConfigError("continuous air transition boundary mismatch");
          }
        }
      }
      if (!stage.has_active_base_velocities || !next_stage.has_active_base_velocities) {
        throw ConfigError("continuous air transition requires base velocities");
      }
      const Vector3d current_world_velocity =
          worldFromBase(stage.pose_end).block<3, 3>(0, 0) *
          stage.active_base_velocities_m_s.back()[0];
      const Vector3d next_world_velocity =
          worldFromBase(next_stage.pose_start).block<3, 3>(0, 0) *
          next_stage.active_base_velocities_m_s.front()[0];
      if (current_world_velocity.norm() <= 1e-9) {
        throw ConfigError("continuous air transition velocity mismatch");
      }
      for (int i = 0; i < 3; ++i) {
        if (!allCloseAbs(current_world_velocity[i], next_world_velocity[i], 1e-9)) {
          throw ConfigError("continuous air transition velocity mismatch");
        }
      }
      continue;
    }

    bool roles_present = contact_plan_v1;
    for (const char* key : kRoleKeys) {
      roles_present = roles_present &&
                      stage.contact_roles.find(key) != stage.contact_roles.end() &&
                      next_stage.contact_roles.find(key) != next_stage.contact_roles.end();
    }
    if (!roles_present) {
      throw ConfigError("invalid front continuous air transition");
    }
    const std::set<int> carried = toSet(stage.roleLegs("airborne_legs_end"));
    const std::set<int> next_carried = toSet(next_stage.roleLegs("airborne_legs_start"));
    bool knots_equal = true;
    for (int leg = 0; leg < kLegCount; ++leg) {
      for (int i = 0; i < 3; ++i) {
        if (!allCloseAbs(stage.anchor_knots.back()[leg][i],
                         next_stage.anchor_knots.front()[leg][i], 1e-12)) {
          knots_equal = false;
        }
      }
    }
    if (carried.empty() || carried != toSet(stage.active_legs) ||
        next_carried != carried || !isSubset(carried, toSet(next_stage.active_legs)) ||
        stage.anchor_curve != "piecewise_base_quintic" ||
        next_stage.anchor_curve != "piecewise_base_quintic" ||
        !allClosePose(stage.pose_end, next_stage.pose_start, 1e-12) || !knots_equal ||
        !stage.has_active_base_velocities || !next_stage.has_active_base_velocities) {
      throw ConfigError("invalid front continuous air transition");
    }
    for (int leg : carried) {
      const std::size_t current_index = static_cast<std::size_t>(
          std::find(stage.active_legs.begin(), stage.active_legs.end(), leg) -
          stage.active_legs.begin());
      const std::size_t next_index = static_cast<std::size_t>(
          std::find(next_stage.active_legs.begin(), next_stage.active_legs.end(), leg) -
          next_stage.active_legs.begin());
      const Vector3d current_world_velocity =
          worldFromBase(stage.pose_end).block<3, 3>(0, 0) *
          stage.active_base_velocities_m_s.back()[current_index];
      const Vector3d next_world_velocity =
          worldFromBase(next_stage.pose_start).block<3, 3>(0, 0) *
          next_stage.active_base_velocities_m_s.front()[next_index];
      bool velocity_ok = current_world_velocity.norm() > 1e-9;
      for (int i = 0; i < 3; ++i) {
        if (!allCloseAbs(current_world_velocity[i], next_world_velocity[i], 1e-9)) {
          velocity_ok = false;
        }
      }
      if (!velocity_ok) {
        throw ConfigError("continuous air transition velocity mismatch");
      }
    }
  }

  const CompactSettleGate& gate = config.settle_gate;
  const double gate_values[] = {
      gate.max_joint_tracking_error_rad, gate.max_foot_target_error_m,
      gate.entry_max_joint_error_rad, gate.persistence_s, gate.timeout_s};
  if (!gate.command_tracking_only_not_contact_proof ||
      gate.preview_time_only_stage_advance != gate.tracking_errors_diagnostic_only ||
      *std::min_element(gate_values, gate_values + 5) <= 0.0) {
    throw ConfigError("invalid simulation-only settle gate");
  }
}

std::string validateCompactClimbSide(const std::string& side) {
  if (side != "left" && side != "right") {
    throw ConfigError("climb side must be left or right");
  }
  return side;
}

namespace {

// 按紧凑腿槽位镜像：right 侧按 world x 中心取镜像腿行。
constexpr std::array<int, 6> kMirrorLegOrder = {3, 4, 5, 0, 1, 2};
constexpr int kMirrorLegIndex[6] = {3, 4, 5, 0, 1, 2};

const std::unordered_map<std::string, std::string>& mirrorNameTokens() {
  static const std::unordered_map<std::string, std::string> tokens = {
      {"LB", "RB"}, {"LF", "RF"}, {"LM", "RM"},
      {"RB", "LB"}, {"RF", "LF"}, {"RM", "LM"},
      {"LEFT", "RIGHT"}, {"RIGHT", "LEFT"}};
  return tokens;
}

std::string mirrorCompactStageName(const std::string& name) {
  std::string mirrored;
  std::size_t start = 0;
  while (start <= name.size()) {
    const std::size_t underscore = name.find('_', start);
    const std::string token = name.substr(
        start, underscore == std::string::npos ? std::string::npos : underscore - start);
    const auto found = mirrorNameTokens().find(token);
    mirrored += (found != mirrorNameTokens().end() ? found->second : token);
    if (underscore == std::string::npos) {
      break;
    }
    mirrored += '_';
    start = underscore + 1;
  }
  return mirrored;
}

std::array<Vector3d, 6> mirrorWorldAnchorRows(
    const std::array<Vector3d, 6>& rows, double center_x) {
  std::array<Vector3d, 6> mirrored{};
  for (int slot = 0; slot < kLegCount; ++slot) {
    mirrored[slot] = rows[kMirrorLegOrder[slot]];
    mirrored[slot].x() = 2.0 * center_x - mirrored[slot].x();
  }
  return mirrored;
}

}  // namespace

ClimbCompactConfig selectCompactClimbSide(const ClimbCompactConfig& config,
                                          const std::string& side) {
  validateCompactClimbSide(side);
  if (config.isFront() && side == "right") {
    throw ConfigError("front compact climbing has no right-side mirror");
  }
  ClimbCompactConfig selected = config;
  if (side == "left") {
    return selected;
  }

  const double center_x = selected.xiaolan_translation.x();
  selected.p0.base[0] = 2.0 * center_x - selected.p0.base[0];
  selected.p0.base[3] = -selected.p0.base[3];
  selected.p0.pose[0] = 2.0 * center_x - selected.p0.pose[0];
  selected.p0.pose[4] = -selected.p0.pose[4];
  selected.p0.anchors_world_m =
      mirrorWorldAnchorRows(selected.p0.anchors_world_m, center_x);
  {
    JointAngles mirrored{};
    for (int slot = 0; slot < kLegCount; ++slot) {
      mirrored[slot] = -selected.terminal_q_rad[kMirrorLegOrder[slot]];
    }
    selected.terminal_q_rad = mirrored;
  }

  for (CompactStage& stage : selected.stages) {
    for (BasePose* pose : {&stage.pose_start, &stage.pose_end}) {
      (*pose)[0] = 2.0 * center_x - (*pose)[0];
      (*pose)[4] = -(*pose)[4];
    }
    for (std::array<Vector3d, 6>& knot : stage.anchor_knots) {
      knot = mirrorWorldAnchorRows(knot, center_x);
    }
    std::vector<int> mirrored_active;
    mirrored_active.reserve(stage.active_legs.size());
    for (int leg : stage.active_legs) {
      mirrored_active.push_back(kMirrorLegIndex[leg]);
    }
    stage.active_legs = std::move(mirrored_active);
    for (std::vector<Vector3d>& knot : stage.active_base_knots_m) {
      for (Vector3d& point : knot) {
        point.x() = -point.x();
      }
    }
    stage.name = mirrorCompactStageName(stage.name);
  }

  for (std::string& name : selected.visual_validation_deferred_for_sim_finish) {
    name = mirrorCompactStageName(name);
  }
  return selected;
}

std::pair<Vector2d, Vector2d> binaryStlXyBounds(const std::string& mesh_path) {
  std::ifstream file(mesh_path, std::ios::binary);
  if (!file) {
    throw ConfigError("cannot open Xiaolan STL: " + mesh_path);
  }
  std::array<char, 84> header{};
  file.read(header.data(), 84);
  if (file.gcount() != 84) {
    throw ConfigError("Xiaolan STL is too short");
  }
  const std::uint32_t triangle_count = *reinterpret_cast<std::uint32_t*>(&header[80]);
  if (triangle_count == 0) {
    throw ConfigError("Xiaolan STL must be a non-empty binary STL");
  }
  Vector2d min_xy = Vector2d::Constant(std::numeric_limits<double>::infinity());
  Vector2d max_xy = Vector2d::Constant(-std::numeric_limits<double>::infinity());
  std::array<char, 50> facet{};
  for (std::uint32_t triangle = 0; triangle < triangle_count; ++triangle) {
    file.read(facet.data(), 50);
    if (file.gcount() != 50) {
      throw ConfigError("Xiaolan STL must be a non-empty binary STL");
    }
    for (int vertex = 0; vertex < 3; ++vertex) {
      for (int axis = 0; axis < 2; ++axis) {
        float coordinate = 0.0f;
        std::memcpy(&coordinate,
                    &facet[12 + vertex * 12 + axis * 4], sizeof(float));
        const double value = static_cast<double>(coordinate);
        if (!std::isfinite(value)) {
          throw ConfigError("Xiaolan STL contains non-finite vertices");
        }
        min_xy[axis] = std::min(min_xy[axis], value);
        max_xy[axis] = std::max(max_xy[axis], value);
      }
    }
  }
  return {min_xy, max_xy};
}

CompactApproachGeometry deriveCompactApproachGeometry(
    const ClimbCompactConfig& config, const std::string& xiaolan_mesh_path) {
  const Vector3d& translation = config.xiaolan_translation;

  CompactApproachGeometry geometry;
  for (const std::string side : {"left", "right"}) {
    const ClimbCompactConfig selected = selectCompactClimbSide(config, side);
    const std::array<double, 4>& base = selected.p0.base;
    const double cosine = std::cos(base[3]);
    const double sine = std::sin(base[3]);
    Matrix4d target = Matrix4d::Identity();
    target(0, 0) = cosine;
    target(0, 1) = -sine;
    target(1, 0) = sine;
    target(1, 1) = cosine;
    target.block<3, 1>(0, 3) =
        Vector3d(base[0], base[1], base[2]) - translation;
    geometry.targets[side] = target;
  }

  const auto [bounds_min, bounds_max] = binaryStlXyBounds(xiaolan_mesh_path);
  geometry.xiaolan_keepout_polygon_xy_m = {
      Vector2d(bounds_min.x(), bounds_min.y()),
      Vector2d(bounds_max.x(), bounds_min.y()),
      Vector2d(bounds_max.x(), bounds_max.y()),
      Vector2d(bounds_min.x(), bounds_max.y())};
  return geometry;
}

}  // namespace grasp_hexapod_control_cpp
