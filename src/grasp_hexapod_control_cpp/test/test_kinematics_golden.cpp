// 运动学金标：FK / Jacobian / DLS / 连杆碰撞与 Python numpy 实现逐项对比。
#include "golden_helpers.h"

#include <gtest/gtest.h>

#include <vector>

namespace g = grasp_hexapod_control_cpp;
namespace t = grasp_hexapod_control_cpp::testing;

namespace {

double maxDiffMatrix(const std::array<g::Matrix3d, 6>& actual,
                     const std::vector<double>& expected_flat) {
  double worst = 0.0;
  for (int leg = 0; leg < g::kLegCount; ++leg) {
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        const double expected =
            expected_flat[leg * 9 + row * 3 + col];
        worst = std::max(worst, std::abs(actual[leg](row, col) - expected));
      }
    }
  }
  return worst;
}

}  // namespace

TEST(KinematicsGolden, ForwardJacobianDlsAndCollisionMatchPython) {
  const t::json golden = t::loadGolden();
  g::GraspController controller(1.0 / 30.0, false, false, t::goldenPaths());

  for (const t::json& sample : golden["kinematics"]["samples"]) {
    const g::JointAngles q = t::toJoints(sample["q"].get<std::vector<double>>());

    const g::FootPositions forward_hip = controller.kinematic.forward(q);
    EXPECT_EQ(t::maxAbsDiff(forward_hip,
                            t::toFeet(sample["forward_hip"].get<std::vector<double>>())),
              0.0)
        << "forward_hip must match bit-for-bit for q=" << sample["q"].dump();

    const g::FootPositions forward_base = controller.kinematic.forwardBase(q);
    EXPECT_EQ(t::maxAbsDiff(forward_base,
                            t::toFeet(sample["forward_base"].get<std::vector<double>>())),
              0.0)
        << "forward_base must match bit-for-bit for q=" << sample["q"].dump();

    const std::array<g::Matrix3d, 6> jacobians = controller.kinematic.jacobian(q);
    EXPECT_LE(maxDiffMatrix(jacobians,
                            sample["jacobian"].get<std::vector<double>>()),
              1e-12)
        << "jacobian must match Python within 1e-12";

    const std::array<g::Matrix3d, 6> damped =
        controller.kinematic.dampedInverseJacobian(q);
    // numpy inv 使用 LAPACK，Eigen 使用闭式解，容许 ulp 级差异。
    EXPECT_LE(maxDiffMatrix(damped,
                            sample["damped_inverse"].get<std::vector<double>>()),
              1e-12)
        << "damped inverse must match Python within 1e-12";

    const std::array<bool, 6> collision_free = controller.linkCollisionFree(q);
    const std::vector<double> expected =
        sample["link_collision_free"].get<std::vector<double>>();
    for (int leg = 0; leg < g::kLegCount; ++leg) {
      EXPECT_EQ(collision_free[leg], expected[leg] != 0.0)
          << "link collision verdict differs on leg " << leg;
    }
  }
}
