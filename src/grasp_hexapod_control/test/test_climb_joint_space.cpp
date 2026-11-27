// compact 攀爬阶段的关节空间摆动段（stage.joint_space）：解析/校验/运行时回归。
//
// 基线配置 config/climb_front_v16_1.json，阶段 MID_PLATFORM_285，腿 5（rm），
// 区间 knot 1 -> 2；关节节点取笛卡尔节点的精确 IK。
#include "grasp_hexapod_control_cpp/control.h"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#ifndef GRASP_CONTROL_CONFIG_DIR
#define GRASP_CONTROL_CONFIG_DIR "../grasp_hexapod_control/config"
#endif

namespace g = grasp_hexapod_control_cpp;
using nlohmann::json;

namespace {

constexpr const char* kStageName = "MID_PLATFORM_285";
constexpr int kLeg = 5;
constexpr int kFrom = 1;
constexpr int kTo = 2;

json loadBaseDocument() {
  std::ifstream file(std::string(GRASP_CONTROL_CONFIG_DIR) + "/climb_front_v16_1.json");
  if (!file) {
    throw std::runtime_error("cannot open climb_front_v16_1.json");
  }
  json document;
  file >> document;
  return document;
}

json& stageOf(json& document) {
  for (json& stage : document["stages"]) {
    if (stage["name"] == kStageName) {
      return stage;
    }
  }
  throw std::runtime_error("stage not found");
}

int slotOf(const json& stage, int leg) {
  const auto& active = stage["active_legs"];
  for (std::size_t i = 0; i < active.size(); ++i) {
    if (active[i].get<int>() == leg) {
      return static_cast<int>(i);
    }
  }
  throw std::runtime_error("leg not active");
}

// 迭代 DLS 求精确 IK：base_link 足端目标 -> 单腿关节角（从站姿出发）。
g::Vector3d exactIk(const g::GraspKinematic& kinematic, int leg, const g::Vector3d& base_target) {
  g::FootPositions base_feet{};
  base_feet.fill(g::Vector3d::Zero());
  base_feet[leg] = base_target;
  const g::Vector3d hip_target = kinematic.baseToHip(base_feet)[leg];
  g::Vector3d q = g::makeQClimbPrep()[leg];
  for (int iteration = 0; iteration < 500; ++iteration) {
    const g::Vector3d error = hip_target - kinematic.forwardLeg(leg, q);
    if (error.norm() < 1e-14) {
      break;
    }
    const g::Matrix3d jacobian = kinematic.jacobianLeg(leg, q);
    const g::Matrix3d damped =
        jacobian.transpose() * (jacobian * jacobian.transpose() +
                                1e-10 * g::Matrix3d::Identity()).inverse();
    q += damped * error;
  }
  return q;
}

json vec3Json(const g::Vector3d& v) { return json::array({v.x(), v.y(), v.z()}); }

g::Vector3d baseKnot(const json& stage, int knot) {
  const json& p = stage["active_base_knots_m"][knot][slotOf(stage, kLeg)];
  return g::Vector3d(p[0].get<double>(), p[1].get<double>(), p[2].get<double>());
}

// 构造含 joint_space 的配置：该腿 knot 1/2 的基座速度置零，关节节点为精确 IK。
json makeJointSpaceDocument(const g::Vector3d& to_offset = g::Vector3d::Zero()) {
  json document = loadBaseDocument();
  json& stage = stageOf(document);
  const int slot = slotOf(stage, kLeg);
  for (int knot : {kFrom, kTo}) {
    stage["active_base_velocities_m_s"][knot][slot] = json::array({0.0, 0.0, 0.0});
  }
  const g::GraspKinematic kinematic;
  const g::Vector3d q_from = exactIk(kinematic, kLeg, baseKnot(stage, kFrom));
  const g::Vector3d q_to = exactIk(kinematic, kLeg, baseKnot(stage, kTo) + to_offset);
  stage["joint_space"] = {
      {"legs", json::array({kLeg})},
      {"from_knot", kFrom},
      {"to_knot", kTo},
      {"q_knots_rad", json::array({json::array({vec3Json(q_from)}),
                                   json::array({vec3Json(q_to)})})},
  };
  return document;
}

void expectRejected(const json& document, const std::string& needle) {
  try {
    const g::ClimbCompactConfig config = g::climbCompactFromJson(document);
    g::validateClimbCompact(config);
    FAIL() << "expected rejection containing: " << needle;
  } catch (const g::ConfigError& error) {
    EXPECT_NE(std::string(error.what()).find(needle), std::string::npos)
        << "actual message: " << error.what();
  }
}

std::vector<double> stageCumulative(const json& stage) {
  std::vector<double> cumulative;
  double sum = 0.0;
  for (const json& d : stage["segment_durations_s"]) {
    sum += d.get<double>();
    cumulative.push_back(sum);
  }
  return cumulative;
}

}  // namespace

TEST(ClimbJointSpace, AcceptsExactIkKnots) {
  const json document = makeJointSpaceDocument();
  const g::ClimbCompactConfig config = g::climbCompactFromJson(document);
  EXPECT_NO_THROW(g::validateClimbCompact(config));
  bool found = false;
  for (const g::CompactStage& stage : config.stages) {
    if (stage.name == kStageName) {
      found = true;
      EXPECT_TRUE(stage.has_joint_space);
      EXPECT_EQ(stage.joint_space.legs, std::vector<int>{kLeg});
      EXPECT_EQ(stage.joint_space.q_knots_rad.size(), 2u);
    } else {
      EXPECT_FALSE(stage.has_joint_space);
    }
  }
  EXPECT_TRUE(found);
}

TEST(ClimbJointSpace, AcceptsZeroEndVelocitiesAndSubMillimetreOffset) {
  json document = makeJointSpaceDocument(g::Vector3d(0.0005, 0.0, 0.0));
  json& js = stageOf(document)["joint_space"];
  js["q_velocities_rad_s"] = json::array(
      {json::array({json::array({0.0, 0.0, 0.0})}), json::array({json::array({0.0, 0.0, 0.0})})});
  const g::ClimbCompactConfig config = g::climbCompactFromJson(document);
  EXPECT_NO_THROW(g::validateClimbCompact(config));
}

TEST(ClimbJointSpace, RejectsMalformedAndInconsistentFields) {
  {  // 腿不是活动腿
    json document = makeJointSpaceDocument();
    stageOf(document)["joint_space"]["legs"] = json::array({0});
    expectRejected(document, "not an active leg");
  }
  {  // 重复腿
    json document = makeJointSpaceDocument();
    json& js = stageOf(document)["joint_space"];
    js["legs"] = json::array({kLeg, kLeg});
    expectRejected(document, "invalid joint_space");
  }
  {  // 形状错误：多一行
    json document = makeJointSpaceDocument();
    json& js = stageOf(document)["joint_space"];
    js["q_knots_rad"].push_back(js["q_knots_rad"][1]);
    expectRejected(document, "q_knots_rad shape mismatch");
  }
  {  // from >= to
    json document = makeJointSpaceDocument();
    json& js = stageOf(document)["joint_space"];
    js["from_knot"] = 2;
    expectRejected(document, "knot range");
  }
  {  // to 越界
    json document = makeJointSpaceDocument();
    json& js = stageOf(document)["joint_space"];
    js["to_knot"] = 99;
    expectRejected(document, "knot range");
  }
  {  // 终点连续性偏差 5 mm
    const json document = makeJointSpaceDocument(g::Vector3d(0.005, 0.0, 0.0));
    expectRejected(document, "boundary FK does not match");
  }
  {  // 超出关节限位
    json document = makeJointSpaceDocument();
    json& js = stageOf(document)["joint_space"];
    js["q_knots_rad"][1][0][2] = 2.5;
    expectRejected(document, "outside joint limits");
  }
  {  // 末端速度非零
    json document = makeJointSpaceDocument();
    json& js = stageOf(document)["joint_space"];
    js["q_velocities_rad_s"] = json::array(
        {json::array({json::array({0.0, 0.0, 0.0})}),
         json::array({json::array({0.0, 0.0, 0.1})})});
    expectRejected(document, "end velocities must be zero");
  }
  {  // 基座速度在边界非零
    json document = makeJointSpaceDocument();
    json& stage = stageOf(document);
    stage["active_base_velocities_m_s"][kFrom][slotOf(stage, kLeg)] =
        json::array({0.0, 0.01, 0.0});
    expectRejected(document, "base velocities must be zero");
  }
  {  // 非 piecewise_base_quintic 阶段
    json document = makeJointSpaceDocument();
    json& stage = stageOf(document);
    stage["anchor_curve"] = "piecewise_quintic";
    stage.erase("active_base_knots_m");
    stage.erase("active_base_velocities_m_s");
    expectRejected(document, "requires piecewise_base_quintic");
  }
}

TEST(ClimbJointSpace, RightMirrorIsRejected) {
  // front 配置本身没有右侧镜像；侧向镜像不支持 joint_space 同样会被拒绝。
  const g::ClimbCompactConfig config = g::climbCompactFromJson(makeJointSpaceDocument());
  EXPECT_THROW(g::selectCompactClimbSide(config, "right"), g::ConfigError);
}

namespace {

struct Tick {
  g::JointAngles q_des;
  int stage_index = -1;
  double phase_time = 0.0;
  bool override_present = false;
  g::Vector3d override_value = g::Vector3d::Zero();
};

std::vector<Tick> replay(const json& document, int* final_state) {
  const std::string dir = GRASP_CONTROL_CONFIG_DIR;
  g::ControlPaths paths{dir + "/climb_compact.json", dir + "/workspace_bounds.csv"};
  g::GraspController controller(1.0 / 30.0, false, false, paths);
  g::ClimbCompactConfig config = g::climbCompactFromJson(document);
  config = g::selectCompactClimbSide(config, "left");
  g::JointAngles q = g::makeQClimbPrep();
  controller.enterClimb(q, config, 0, std::nullopt, false);
  std::vector<Tick> ticks;
  while (controller.climb_mode.state == g::ClimbState::Running && ticks.size() < 40000) {
    q = controller.update(q, Eigen::Vector4d::Zero());  // 理想舵机：q_cur = 上一帧 q_des
    Tick tick;
    tick.q_des = q;
    tick.stage_index = *controller.climb_mode.stage_index;
    tick.phase_time = controller.climb_mode.phase_time;
    tick.override_present = controller.joint_override[kLeg].has_value();
    if (tick.override_present) {
      tick.override_value = *controller.joint_override[kLeg];
    }
    for (int leg = 0; leg < g::kLegCount; ++leg) {
      if (leg != kLeg) {
        EXPECT_FALSE(controller.joint_override[leg].has_value());
      }
    }
    ticks.push_back(tick);
  }
  *final_state = static_cast<int>(controller.climb_mode.state);
  // 退出攀爬后覆盖必须被清空。
  controller.abortClimb();
  for (const auto& entry : controller.joint_override) {
    EXPECT_FALSE(entry.has_value());
  }
  return ticks;
}

// 与实现无关的参考：smoothstep 五次插值（无速度时与 piecewiseSlots 同一方案）。
g::Vector3d referenceQ(const json& joint_space, const std::vector<double>& durations,
                       double elapsed) {
  const int rows = static_cast<int>(joint_space["q_knots_rad"].size());
  const auto knot = [&](int row) {
    const json& q = joint_space["q_knots_rad"][row][0];
    return g::Vector3d(q[0].get<double>(), q[1].get<double>(), q[2].get<double>());
  };
  double remaining = elapsed;
  for (int segment = 0; segment < rows - 1; ++segment) {
    const double duration = durations[segment];
    if (remaining <= duration) {
      const double u = remaining / duration;
      const double w = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
      return knot(segment) * (1.0 - w) + knot(segment + 1) * w;
    }
    remaining -= duration;
  }
  return knot(rows - 1);
}

}  // namespace

TEST(ClimbJointSpace, RuntimeTracksJointKnotsAndLeavesOtherLegsUnchanged) {
  const json with_joint = makeJointSpaceDocument();
  json without_joint = with_joint;
  stageOf(without_joint).erase("joint_space");  // 同样已置零该腿基座速度

  int state_a = -1;
  int state_b = -1;
  const std::vector<Tick> a = replay(with_joint, &state_a);
  const std::vector<Tick> b = replay(without_joint, &state_b);
  EXPECT_EQ(state_a, static_cast<int>(g::ClimbState::Done));
  EXPECT_EQ(state_b, static_cast<int>(g::ClimbState::Done));
  ASSERT_EQ(a.size(), b.size());

  json doc_copy = with_joint;
  const json& stage = stageOf(doc_copy);
  int stage_index = -1;
  for (std::size_t i = 0; i < doc_copy["stages"].size(); ++i) {
    if (doc_copy["stages"][i]["name"] == kStageName) {
      stage_index = static_cast<int>(i);
    }
  }
  const std::vector<double> cumulative = stageCumulative(stage);
  const double t_from = cumulative[kFrom - 1];
  const double t_to = cumulative[kTo - 1];
  std::vector<double> durations;
  durations.push_back(stage["segment_durations_s"][kFrom].get<double>());

  int in_range_ticks = 0;
  double worst_joint = 0.0;
  double worst_other = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const bool in_range =
        a[i].stage_index == stage_index && a[i].phase_time >= t_from && a[i].phase_time <= t_to;
    EXPECT_EQ(a[i].override_present, in_range) << "tick " << i;
    if (in_range) {
      ++in_range_ticks;
      const g::Vector3d expected = referenceQ(stage["joint_space"], durations,
                                              a[i].phase_time - t_from);
      worst_joint = std::max(worst_joint, (a[i].q_des[kLeg] - expected).cwiseAbs().maxCoeff());
      EXPECT_LE((a[i].q_des[kLeg] - expected).cwiseAbs().maxCoeff(), 1e-9) << "tick " << i;
      EXPECT_LE((a[i].override_value - expected).cwiseAbs().maxCoeff(), 1e-9) << "tick " << i;
    }
    for (int leg = 0; leg < g::kLegCount; ++leg) {
      if (leg == kLeg) {
        continue;
      }
      const double diff = (a[i].q_des[leg] - b[i].q_des[leg]).cwiseAbs().maxCoeff();
      worst_other = std::max(worst_other, diff);
      EXPECT_LE(diff, 1e-12) << "tick " << i << " leg " << leg;
    }
  }
  EXPECT_GT(in_range_ticks, 20);
  std::printf("[joint-space] ticks=%zu in_range=%d max|q_des-q_ref|=%.3e max_other_leg_diff=%.3e\n",
              a.size(), in_range_ticks, worst_joint, worst_other);
}
