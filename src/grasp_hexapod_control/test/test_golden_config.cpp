// 配置金标：compact 右侧镜像、dock_system.yaml 解析、接近几何（STL 包围盒）
// 与 Python 实现的解析结果逐项对比。
#include "golden_helpers.h"

#include <gtest/gtest.h>

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
