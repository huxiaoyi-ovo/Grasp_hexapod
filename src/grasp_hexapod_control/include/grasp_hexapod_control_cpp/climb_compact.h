// compact 攀爬配置：加载、校验、左右镜像与接近几何
// （对应 Python climb_mode.py 的 _load_config/_validate_config/terminal_joints
// 与 utils/climb.py 的 select_compact_climb_side/derive_compact_approach_geometry）。
//
// 这些函数只做确定性模型计算；校验失败抛 ConfigError，消息与 Python 一致。
#pragma once

#include "grasp_hexapod_control_cpp/types.h"

#include <nlohmann/json.hpp>

#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace grasp_hexapod_control_cpp {

inline constexpr const char* kCompactSchema = "SIMULATION_ONLY_CLIMB_COMPACT_V2";

// 根据 (x, y, z, roll, pitch) 生成世界坐标变换矩阵（ClimbMode._world_from_base）。
Matrix4d worldFromBase(const std::array<double, 5>& base);

// compact settle gate（settle_gate 字段）。
struct CompactSettleGate {
  double max_joint_tracking_error_rad = 0.0;
  double max_foot_target_error_m = 0.0;
  double entry_max_joint_error_rad = 0.0;
  double persistence_s = 0.0;
  double timeout_s = 0.0;
  bool command_tracking_only_not_contact_proof = false;
  bool preview_time_only_stage_advance = false;
  bool tracking_errors_diagnostic_only = false;
};

// compact 单阶段（stages[i]）。
struct CompactStage {
  std::string name;
  std::array<double, 5> pose_start{};
  std::array<double, 5> pose_end{};
  // K 个锚点关键帧，每个是 6 条腿的世界坐标 (x,y,z)。
  std::vector<std::array<Vector3d, 6>> anchor_knots;
  std::vector<double> segment_durations_s;
  std::vector<int> active_legs;
  bool active_legs_present = false;  // 区分缺失（非法）与空列表（合法）
  // 原始 JSON 字符串：piecewise_quintic / piecewise_base_quintic / relative_base_high_step
  std::string anchor_curve;
  // quintic_full_stage / quintic_first_segment
  std::string pose_curve;
  double settle_s = std::numeric_limits<double>::quiet_NaN();
  std::optional<double> relative_swing_height_m;
  bool has_settle_persistence = false;
  double settle_persistence_s = 0.0;
  // (K, len(active_legs), 3)，仅 piecewise_base_quintic 使用。
  bool has_active_base_knots = false;
  std::vector<std::vector<Vector3d>> active_base_knots_m;
  bool has_active_base_velocities = false;
  std::vector<std::vector<Vector3d>> active_base_velocities_m_s;
  bool continuous_air_transition = false;
};

struct CompactP0 {
  std::array<Vector3d, 6> anchors_world_m{};
  JointAngles q_rad{};
  std::array<double, 4> base{};
};

struct ClimbCompactConfig {
  std::string schema;
  bool simulation_only = false;
  bool simulation_candidate_only = false;
  Vector3d xiaolan_translation = Vector3d::Zero();
  std::vector<std::string> visual_validation_deferred_for_sim_finish;
  CompactP0 p0;
  JointAngles terminal_q_rad{};
  int stage_count = 0;
  std::vector<CompactStage> stages;
  CompactSettleGate settle_gate;
};

// 读取并检查 compact 配置（文件不存在/JSON 非法抛 ConfigError）。
ClimbCompactConfig loadClimbCompact(const std::string& path);

// 检查 compact 攀爬配置（ClimbMode._validate_config 的完整移植）。
void validateClimbCompact(const ClimbCompactConfig& config);

// 从 JSON 文档构造（金标测试使用内存 JSON）。
ClimbCompactConfig climbCompactFromJson(const nlohmann::json& document);

// 验证 compact 攀爬侧别；只接受 left/right。
std::string validateCompactClimbSide(const std::string& side);

// 返回 compact 的独立侧别副本，right 按小蓝 world x 中心镜像
// （utils/climb.py::select_compact_climb_side）。
ClimbCompactConfig selectCompactClimbSide(const ClimbCompactConfig& config,
                                          const std::string& side);

// 左右接近几何（utils/climb.py::derive_compact_approach_geometry）。
struct CompactApproachGeometry {
  std::map<std::string, Matrix4d> targets;  // "left"/"right" -> xiaolan_frame 位姿
  std::vector<Vector2d> xiaolan_keepout_polygon_xy_m;
};

// 读取 CLIMB 二进制小蓝 STL 的 XY 投影包围盒。
std::pair<Vector2d, Vector2d> binaryStlXyBounds(const std::string& mesh_path);

CompactApproachGeometry deriveCompactApproachGeometry(
    const ClimbCompactConfig& config, const std::string& xiaolan_mesh_path);

}  // namespace grasp_hexapod_control_cpp
