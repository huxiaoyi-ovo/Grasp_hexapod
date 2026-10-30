// 配置金标：compact 右侧镜像、dock_system.yaml 解析、接近几何（STL 包围盒）
// 与 Python 实现的解析结果逐项对比。
#include "golden_helpers.h"

#include <gtest/gtest.h>

#include <fstream>
#include <string>
#include <vector>

namespace g = grasp_hexapod_control_cpp;
namespace t = grasp_hexapod_control_cpp::testing;

namespace {

double maxDiffMatrix4(const g::Matrix4d& actual,
                      const std::vector<double>& expected_flat) {
  double worst = 0.0;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      worst = std::max(worst, std::abs(actual(row, col) - expected_flat[row * 4 + col]));
    }
  }
  return worst;
}

}  // namespace

TEST(GoldenConfig, CompactValidationAcceptsShippedConfig) {
  const g::ClimbCompactConfig config =
      g::loadClimbCompact(t::goldenPaths().climb_compact_json);
  g::validateClimbCompact(config);  // 不抛即通过。
  SUCCEED();
}

TEST(GoldenConfig, RightSideMirrorMatchesPython) {
  const t::json golden = t::loadGolden();
  const t::json& mirror = golden["climb_mirror"];

  g::ClimbCompactConfig config =
      g::loadClimbCompact(t::goldenPaths().climb_compact_json);
  const g::ClimbCompactConfig mirrored = g::selectCompactClimbSide(config, "right");

  EXPECT_NEAR(mirrored.xiaolan_translation.x(),
              mirror["xiaolan_translation"][0].get<double>(), 0.0);
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(mirrored.p0.base[i], mirror["p0_base"][i].get<double>(), 0.0);
  }
  EXPECT_EQ(t::maxAbsDiff(mirrored.p0.anchors_world_m,
                          t::toFeet(mirror["p0_anchors"].get<std::vector<double>>())),
            0.0);
  EXPECT_EQ(t::maxAbsDiff(mirrored.terminal_q_rad,
                          t::toJoints(mirror["terminal_q"].get<std::vector<double>>())),
            0.0);

  ASSERT_EQ(mirrored.stages.size(), mirror["stages"].size());
  for (std::size_t i = 0; i < mirrored.stages.size(); ++i) {
    const g::CompactStage& stage = mirrored.stages[i];
    const t::json& expected = mirror["stages"][i];
    EXPECT_EQ(stage.name, expected["name"].get<std::string>());
    for (int k = 0; k < 5; ++k) {
      EXPECT_EQ(stage.pose_start[k], expected["pose_start"][k].get<double>());
      EXPECT_EQ(stage.pose_end[k], expected["pose_end"][k].get<double>());
    }
    ASSERT_EQ(stage.active_legs.size(), expected["active_legs"].size());
    for (std::size_t k = 0; k < stage.active_legs.size(); ++k) {
      EXPECT_EQ(stage.active_legs[k], expected["active_legs"][k].get<int>());
    }
    ASSERT_EQ(stage.anchor_knots.size(), expected["anchor_knots"].size());
    for (std::size_t k = 0; k < stage.anchor_knots.size(); ++k) {
      EXPECT_EQ(t::maxAbsDiff(
                    stage.anchor_knots[k],
                    t::toFeet(expected["anchor_knots"][k].get<std::vector<double>>())),
                0.0)
          << "anchor knot mismatch at stage " << i << " knot " << k;
    }
  }
}

TEST(GoldenConfig, DockSystemMatchesPython) {
  const t::json golden = t::loadGolden();
  const t::json& expected = golden["dock_system"];

  const g::DockSystem system =
      g::loadDockSystem(std::string(GRASP_CONTROL_CONFIG_DIR) + "/dock_system.yaml");
  EXPECT_EQ(system.real_calibrated, expected["real_calibrated"].get<bool>());
  EXPECT_NEAR(system.tag_size_m, expected["tag_size_m"].get<double>(), 0.0);
  EXPECT_LE(maxDiffMatrix4(system.lock_from_camera,
                           expected["lock_from_camera"].get<std::vector<double>>()),
            0.0);
  for (int tag_id = 0; tag_id < 4; ++tag_id) {
    EXPECT_LE(maxDiffMatrix4(system.pin_from_tag.at(tag_id),
                             expected["pin_from_tag"][std::to_string(tag_id)]
                                      .get<std::vector<double>>()),
              0.0)
        << "pin_from_tag mismatch for tag " << tag_id;
  }
}

TEST(GoldenConfig, ApproachGeometryMatchesPython) {
  const t::json golden = t::loadGolden();
  const t::json& expected = golden["approach_geometry"];

  const g::ClimbCompactConfig config =
      g::loadClimbCompact(t::goldenPaths().climb_compact_json);
  const std::string mesh_path =
      std::string(GRASP_DESCRIPTION_MESH_DIR) + "/xiaolan/base_link_xiaolan.STL";
  const g::CompactApproachGeometry geometry =
      g::deriveCompactApproachGeometry(config, mesh_path);

  for (const std::string side : {"left", "right"}) {
    EXPECT_LE(maxDiffMatrix4(geometry.targets.at(side),
                             expected["targets"][side].get<std::vector<double>>()),
              1e-12)
        << "approach target mismatch for side " << side;
  }
  const std::vector<double> keepout =
      expected["keepout"].get<std::vector<double>>();
  ASSERT_EQ(geometry.xiaolan_keepout_polygon_xy_m.size(), keepout.size() / 2);
  for (std::size_t i = 0; i < geometry.xiaolan_keepout_polygon_xy_m.size(); ++i) {
    EXPECT_EQ(geometry.xiaolan_keepout_polygon_xy_m[i].x(), keepout[i * 2]);
    EXPECT_EQ(geometry.xiaolan_keepout_polygon_xy_m[i].y(), keepout[i * 2 + 1]);
  }
}

namespace {

t::json loadFrontDirectJson() {
  std::ifstream file(t::frontDirectConfigPath());
  if (!file) {
    throw std::runtime_error("cannot open " + t::frontDirectConfigPath());
  }
  t::json document;
  file >> document;
  return document;
}

// 期望校验抛 ConfigError 且消息完全相同。
void expectRejected(const t::json& document, const std::string& message) {
  try {
    g::validateClimbCompact(g::climbCompactFromJson(document));
    FAIL() << "expected rejection: " << message;
  } catch (const g::ConfigError& error) {
    EXPECT_EQ(std::string(error.what()), message);
  }
}

}  // namespace

TEST(GoldenConfig, FrontDirectParsesLikePython) {
  const t::json golden = t::loadGolden();
  const t::json& expected = golden["climb_front_direct"];

  const g::ClimbCompactConfig config = g::loadClimbCompact(t::frontDirectConfigPath());
  g::validateClimbCompact(config);  // 不抛即通过。
  EXPECT_TRUE(config.isFront());
  EXPECT_EQ(config.climb_orientation, expected["climb_orientation"].get<std::string>());
  EXPECT_EQ(config.stage_count, expected["stage_count"].get<int>());
  ASSERT_TRUE(config.simulation_joint_velocity_limit_rad_s.has_value());
  EXPECT_EQ(*config.simulation_joint_velocity_limit_rad_s,
            expected["simulation_joint_velocity_limit_rad_s"].get<double>());
  EXPECT_FALSE(config.allow_front_hardware) << "must never be set by JSON";
  EXPECT_EQ(config.xiaolan_translation.x(), expected["xiaolan_translation"][0].get<double>());
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(config.p0.pose[i], expected["p0_base"][i].get<double>());
  }
  EXPECT_EQ(t::maxAbsDiff(config.p0.anchors_world_m,
                          t::toFeet(expected["p0_anchors"].get<std::vector<double>>())),
            0.0);
  EXPECT_EQ(t::maxAbsDiff(config.terminal_q_rad,
                          t::toJoints(expected["terminal_q"].get<std::vector<double>>())),
            0.0);

  ASSERT_EQ(config.stages.size(), expected["stages"].size());
  for (std::size_t i = 0; i < config.stages.size(); ++i) {
    const g::CompactStage& stage = config.stages[i];
    const t::json& item = expected["stages"][i];
    EXPECT_EQ(stage.name, item["name"].get<std::string>());
    EXPECT_EQ(stage.pose_curve, item["pose_curve"].get<std::string>());
    EXPECT_EQ(stage.anchor_curve, item["anchor_curve"].get<std::string>());
    for (int k = 0; k < 6; ++k) {
      EXPECT_EQ(stage.pose_start[k], item["pose_start"][k].get<double>());
      EXPECT_EQ(stage.pose_end[k], item["pose_end"][k].get<double>());
    }
    ASSERT_EQ(stage.active_legs.size(), item["active_legs"].size());
    for (std::size_t k = 0; k < stage.active_legs.size(); ++k) {
      EXPECT_EQ(stage.active_legs[k], item["active_legs"][k].get<int>());
    }
    ASSERT_EQ(stage.segment_durations_s.size(), item["segment_durations_s"].size());
    for (std::size_t k = 0; k < stage.segment_durations_s.size(); ++k) {
      EXPECT_EQ(stage.segment_durations_s[k], item["segment_durations_s"][k].get<double>());
    }
    ASSERT_EQ(stage.anchor_knots.size(), item["anchor_knots"].size());
    for (std::size_t k = 0; k < stage.anchor_knots.size(); ++k) {
      EXPECT_EQ(t::maxAbsDiff(
                    stage.anchor_knots[k],
                    t::toFeet(item["anchor_knots"][k].get<std::vector<double>>())),
                0.0)
          << "anchor knot mismatch at stage " << i << " knot " << k;
    }
    ASSERT_EQ(stage.active_base_knots_m.size(), item["active_base_knots"].size());
    for (std::size_t k = 0; k < stage.active_base_knots_m.size(); ++k) {
      const std::vector<double> flat = item["active_base_knots"][k].get<std::vector<double>>();
      ASSERT_EQ(flat.size(), stage.active_base_knots_m[k].size() * 3);
      for (std::size_t slot = 0; slot < stage.active_base_knots_m[k].size(); ++slot) {
        for (int axis = 0; axis < 3; ++axis) {
          EXPECT_EQ(stage.active_base_knots_m[k][slot][axis], flat[slot * 3 + axis]);
        }
      }
    }
    if (item["active_base_velocities"].is_null()) {
      EXPECT_FALSE(stage.has_active_base_velocities);
    } else {
      ASSERT_TRUE(stage.has_active_base_velocities);
      ASSERT_EQ(stage.active_base_velocities_m_s.size(), item["active_base_velocities"].size());
      for (std::size_t k = 0; k < stage.active_base_velocities_m_s.size(); ++k) {
        const std::vector<double> flat =
            item["active_base_velocities"][k].get<std::vector<double>>();
        ASSERT_EQ(flat.size(), stage.active_base_velocities_m_s[k].size() * 3);
        for (std::size_t slot = 0; slot < stage.active_base_velocities_m_s[k].size(); ++slot) {
          for (int axis = 0; axis < 3; ++axis) {
            EXPECT_EQ(stage.active_base_velocities_m_s[k][slot][axis], flat[slot * 3 + axis]);
          }
        }
      }
    }
  }
}

TEST(GoldenConfig, FrontHasNoRightMirrorAndLeftIsUnchanged) {
  const t::json golden = t::loadGolden();
  const g::ClimbCompactConfig config = g::loadClimbCompact(t::frontDirectConfigPath());
  try {
    g::selectCompactClimbSide(config, "right");
    FAIL() << "front right-side mirror must be rejected";
  } catch (const g::ConfigError& error) {
    EXPECT_EQ(std::string(error.what()),
              golden["climb_front_direct"]["right_mirror_error"].get<std::string>());
  }
  const g::ClimbCompactConfig left = g::selectCompactClimbSide(config, "left");
  ASSERT_EQ(left.stages.size(), config.stages.size());
  for (std::size_t i = 0; i < left.stages.size(); ++i) {
    EXPECT_EQ(left.stages[i].name, config.stages[i].name);
    EXPECT_EQ(left.stages[i].pose_start, config.stages[i].pose_start);
    EXPECT_EQ(left.stages[i].pose_end, config.stages[i].pose_end);
  }
  EXPECT_EQ(left.p0.pose, config.p0.pose);
}

TEST(GoldenConfig, FrontValidationRulesMatchPython) {
  // 基线通过。
  g::validateClimbCompact(g::climbCompactFromJson(loadFrontDirectJson()));

  t::json bad_yaw = loadFrontDirectJson();
  bad_yaw["p0"]["base"][5] = 0.0;
  expectRejected(bad_yaw, "front compact P0 must face Xiaolan along +x");

  for (double cap : {0.0, -1.0, 8.5}) {
    t::json bad_cap = loadFrontDirectJson();
    bad_cap["simulation_joint_velocity_limit_rad_s"] = cap;
    expectRejected(bad_cap, "invalid front simulation joint velocity cap");
  }
  t::json bool_cap = loadFrontDirectJson();
  bool_cap["simulation_joint_velocity_limit_rad_s"] = true;
  expectRejected(bool_cap, "invalid front simulation joint velocity cap");
  t::json null_cap = loadFrontDirectJson();
  null_cap["simulation_joint_velocity_limit_rad_s"] = nullptr;
  g::validateClimbCompact(g::climbCompactFromJson(null_cap));  // null == 未设置。

  // 速度上限只对 front 合法。
  {
    std::ifstream file(t::goldenPaths().climb_compact_json);
    t::json side;
    file >> side;
    side["simulation_joint_velocity_limit_rad_s"] = 6.0;
    expectRejected(side, "invalid front simulation joint velocity cap");
    side.erase("simulation_joint_velocity_limit_rad_s");
    side["contact_plan_version"] = 1;
    expectRejected(side, "planned contact roles are front-only");
  }

  // 5 元素位姿在 front 下非法。
  t::json short_pose = loadFrontDirectJson();
  short_pose["stages"][0]["pose_end"] = t::json::array({0, 0, 0, 0, 0});
  EXPECT_THROW(g::climbCompactFromJson(short_pose), g::ConfigError);

  // contact_plan_version 规则。
  t::json version_two = loadFrontDirectJson();
  version_two["contact_plan_version"] = 2;
  expectRejected(version_two, "unsupported contact plan version");
  t::json orphan_roles = loadFrontDirectJson();
  orphan_roles["stages"][1]["support_legs"] = t::json::array({0, 1, 2});
  expectRejected(orphan_roles, "planned contact roles require contact_plan_version=1");

  // 完整合法的 contact plan（支撑腿 = 活动腿补集，无空中腿）被接受；缺字段/错角色被拒绝。
  t::json planned = loadFrontDirectJson();
  planned["contact_plan_version"] = 1;
  for (t::json& stage : planned["stages"]) {
    t::json support = t::json::array();
    for (int leg = 0; leg < 6; ++leg) {
      bool active = false;
      for (const t::json& item : stage["active_legs"]) {
        active = active || item.get<int>() == leg;
      }
      if (!active) {
        support.push_back(leg);
      }
    }
    stage["support_legs"] = support;
    stage["airborne_legs_start"] = t::json::array();
    stage["airborne_legs_end"] = t::json::array();
  }
  g::validateClimbCompact(g::climbCompactFromJson(planned));
  const g::ClimbCompactConfig planned_config = g::climbCompactFromJson(planned);
  EXPECT_EQ(g::ClimbMode::plannedSupportLegs(planned_config, 1),
            planned_config.stages[1].roleLegs("support_legs"));
  EXPECT_TRUE(g::ClimbMode::plannedAirborneLegs(planned_config, 1, "end").empty());

  t::json incomplete = planned;
  incomplete["stages"][2].erase("airborne_legs_end");
  expectRejected(incomplete,
                 "planned contact roles are incomplete: " +
                     incomplete["stages"][2]["name"].get<std::string>());
  t::json wrong_support = planned;
  wrong_support["stages"][2]["support_legs"] = t::json::array({0, 1});
  expectRejected(wrong_support,
                 "invalid planned contact roles: " +
                     wrong_support["stages"][2]["name"].get<std::string>());
  t::json first_air = planned;
  first_air["stages"][0]["airborne_legs_start"] = t::json::array({0});
  expectRejected(first_air,
                 "invalid planned contact roles: " +
                     first_air["stages"][0]["name"].get<std::string>());
}
