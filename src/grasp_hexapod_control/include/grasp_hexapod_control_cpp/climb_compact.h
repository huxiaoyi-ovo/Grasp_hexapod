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

// 机身位姿 [x, y, z, roll, pitch, yaw]；侧向 compact 的 5 元素位姿以 yaw=0 存放。
using BasePose = std::array<double, 6>;

// 根据 (x, y, z, roll, pitch[, yaw]) 生成世界坐标变换矩阵（ClimbMode._world_from_base）。
// 5 元素重载等价于 yaw=0，数值与历史 5 元素实现逐位一致。
Matrix4d worldFromBase(const std::array<double, 5>& base);
Matrix4d worldFromBase(const BasePose& base);

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
  // 侧向 compact 为 5 元素（yaw 存 0）；front compact 为完整 6 元素。
  BasePose pose_start{};
  BasePose pose_end{};
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
  // front-only: pose_curve == piecewise_quintic 的机身位姿关键帧 (K, 6)。
  // pose_knots_wellformed=false 表示存在但不是有限的 (K,6) 数组。
  bool has_pose_knots = false;
  bool pose_knots_wellformed = false;
  std::vector<BasePose> pose_knots;
  // contact_plan_version=1 角色元数据（support_legs / airborne_legs_start /
  // airborne_legs_end / segment_support_legs）：保留原始 JSON，由校验器判定。
  // 仅验证与诊断使用，其中 segment_support_legs 另会固定声明腿的世界锚点。
  std::map<std::string, nlohmann::json> contact_roles;

  // 读取角色列表；键不存在返回空。仅在校验通过后使用。
  std::vector<int> roleLegs(const std::string& key) const;
  // segment_support_legs 存在且非 null。
  bool hasSegmentSupportLegs() const;
  std::vector<int> segmentSupportLegs(std::size_t segment) const;
};

struct CompactP0 {
  std::array<Vector3d, 6> anchors_world_m{};
  JointAngles q_rad{};
  // 侧向 schema: [x, y, z, pitch]；front schema 不使用（全零），改用 pose。
  std::array<double, 4> base{};
  // 完整 P0 机身位姿 [x, y, z, roll, pitch, yaw]（侧向由 base 推出，roll=yaw=0）。
  BasePose pose{};
};

struct ClimbCompactConfig {
  std::string schema;
  bool simulation_only = false;
  bool simulation_candidate_only = false;
  // "front" 为 front compact（6 元素位姿、无右侧镜像）；空/其他为侧向 schema。
  std::string climb_orientation;
  // 仅仿真的 front 关节速度上限（JSON 缺失或 null 视为未设置）；
  // malformed=true 表示字段存在但类型非法（bool/非数值）。
  std::optional<double> simulation_joint_velocity_limit_rad_s;
  bool simulation_joint_velocity_limit_malformed = false;
  // contact_plan_version：present=字段存在且非 null；is_one=其值等于 1。
  bool contact_plan_version_present = false;
  bool contact_plan_version_is_one = false;
  // 调用方（ROS 参数 ~allow_front_hardware）授权 front+hardware_execution。
  // 绝不从 JSON 读取；默认 false，此时 front + hardware 被拒绝。
  bool allow_front_hardware = false;
  Vector3d xiaolan_translation = Vector3d::Zero();
  std::vector<std::string> visual_validation_deferred_for_sim_finish;
  CompactP0 p0;
  JointAngles terminal_q_rad{};
  int stage_count = 0;
  std::vector<CompactStage> stages;
  CompactSettleGate settle_gate;

  bool isFront() const { return climb_orientation == "front"; }
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
// （utils/climb.py::select_compact_climb_side）。front compact 没有右侧镜像：
// left 原样返回，right 抛 ConfigError。
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
