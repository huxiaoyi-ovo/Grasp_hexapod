#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "grasp_hexapod_bt_control/tag_guidance.h"

namespace grasp_hexapod_bt_control {
namespace {

const char* kFrame = "camera_color_optical_frame";
const double kPi = std::acos(-1.);

// Tag frame in the camera frame for a planar scene.  The tag x axis is image
// +x: x = (cos yaw, 0, sin yaw), tag y = camera y (down).  apriltag_ros
// convention A has z = x cross y (into the tag face); convention B (z out of the
// face) flips y and z: R_B = R_A * diag(1,-1,-1).  Both give the same x column.
// yaw = 0 faces the camera squarely; the outward normal is (sin yaw, 0, -cos yaw).
Eigen::Matrix3d planarRotation(double yaw, bool z_out = false) {
  const Eigen::Vector3d x(std::cos(yaw), 0., std::sin(yaw));
  const Eigen::Vector3d y(0., 1., 0.);
  Eigen::Matrix3d r;
  r.col(0) = x;
  r.col(1) = y;
  r.col(2) = x.cross(y);
  if (z_out) r = r * Eigen::Vector3d(1., -1., -1.).asDiagonal();
  return r;
}

TagObservation observation(double x, double z, double yaw, double stamp, int id = 0,
                           const std::string& frame = kFrame, bool z_out = false) {
  const Eigen::Quaterniond q(planarRotation(yaw, z_out));
  TagObservation o;
  o.id = id;
  o.x = x;
  o.y = 0.02;
  o.z = z;
  o.qx = q.x();
  o.qy = q.y();
  o.qz = q.z();
  o.qw = q.w();
  o.stamp = stamp;
  o.frame = frame;
  return o;
}

TagGuidanceConfig testConfig() {
  TagGuidanceConfig c;
  c.ref_x_m = 0.01;
  c.ref_z_m = 0.15;
  c.ref_yaw_rad = 0.05;
  return c;
}

// Feeds fresh samples at 30 Hz.
struct Rig {
  explicit Rig(TagGuidanceConfig c = testConfig()) : guidance(c) { guidance.begin(now); }
  Eigen::Vector4d step(double x, double z, double yaw, bool gait_stopped = true) {
    now += 1. / 30.;
    guidance.cache(observation(x, z, yaw, now - .01, 0, kFrame, z_out));
    return guidance.update(now, gait_stopped);
  }
  Eigen::Vector4d tick(bool gait_stopped = true) {  // control tick, no new sample
    now += 1. / 30.;
    return guidance.update(now, gait_stopped);
  }
  void handoff(double x, double z, double yaw) {
    for (int i = 0; i < 5 && !guidance.handedOff(); ++i) step(x, z, yaw);
  }
  TagGuidance guidance;
  double now = 100.0;
  bool z_out = false;
};

TEST(TagGuidance, YawFormulaOnSyntheticRotationsBothZConventions) {
  EXPECT_NEAR(tagYaw(planarRotation(0.)), 0., 1e-12);
  for (double yaw : {-1.2, -0.5, -0.1, 0.1, 0.5, 1.2}) {
    for (bool z_out : {false, true}) {
      const Eigen::Matrix3d r = planarRotation(yaw, z_out);
      ASSERT_NEAR(r.determinant(), 1., 1e-12);
      EXPECT_NEAR(tagYaw(r), yaw, 1e-12);
      const Eigen::Quaterniond q(r);
      double out = 0.;
      ASSERT_TRUE(tagYawFromQuaternion(2. * q.x(), 2. * q.y(), 2. * q.z(), 2. * q.w(), out));
      EXPECT_NEAR(out, yaw, 1e-9);
      // The outward normal (-z for A, +z for B) is atan2(n_x, -n_z) = yaw.
      const Eigen::Vector3d n = z_out ? Eigen::Vector3d(r.col(2)) : Eigen::Vector3d(-r.col(2));
      EXPECT_NEAR(std::atan2(n.x(), -n.z()), yaw, 1e-12);
    }
  }
  // Convention A: tag facing the camera rotated by alpha about camera +y is
  // R = Ry(alpha) and has yaw = -alpha; convention B multiplies by diag(1,-1,-1).
  for (double alpha : {-0.4, 0.3}) {
    const Eigen::Matrix3d ry = Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitY()).toRotationMatrix();
    EXPECT_NEAR(tagYaw(ry), -alpha, 1e-12);
    EXPECT_NEAR(tagYaw(ry * Eigen::Vector3d(1., -1., -1.).asDiagonal()), -alpha, 1e-12);
  }
  // Non-finite / zero quaternion and a heavily rolled tag (x axis leaves the
  // camera x-z plane by more than kMaxTagXAxisOutOfPlane) are rejected.
  double out = 0.;
  EXPECT_FALSE(tagYawFromQuaternion(0., 0., 0., 0., out));
  EXPECT_FALSE(tagYawFromQuaternion(std::numeric_limits<double>::quiet_NaN(), 0., 0., 1., out));
  const Eigen::Quaterniond rolled(Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()));  // R10 = sin 1 > .5
  EXPECT_FALSE(tagYawFromQuaternion(rolled.x(), rolled.y(), rolled.z(), rolled.w(), out));
  const Eigen::Quaterniond mild(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()));  // R10 = sin .4 < .5
  EXPECT_TRUE(tagYawFromQuaternion(mild.x(), mild.y(), mild.z(), mild.w(), out));
}

TEST(TagGuidance, WrapAngle) {
  EXPECT_NEAR(wrapAngle(3 * kPi / 2), -kPi / 2, 1e-12);
  EXPECT_NEAR(wrapAngle(-3 * kPi / 2), kPi / 2, 1e-12);
  EXPECT_NEAR(wrapAngle(0.2), 0.2, 1e-12);
}

TEST(TagGuidance, HandoffNeedsFiveConsecutiveNewValidDetections) {
  Rig rig;
  for (int i = 0; i < 4; ++i) {
    rig.step(0.05, 0.3, 0.1);
    EXPECT_FALSE(rig.guidance.handedOff());
    EXPECT_EQ(rig.guidance.confirmations(), i + 1);
  }
  EXPECT_EQ(rig.step(0.05, 0.3, 0.1), Eigen::Vector4d::Zero());
  EXPECT_TRUE(rig.guidance.handedOff());
  EXPECT_EQ(rig.guidance.state(), TagState::Guiding);
}

TEST(TagGuidance, DuplicateStampsDoNotCount) {
  Rig rig;
  rig.now += .1;
  const TagObservation o = observation(0., .3, 0., rig.now - .01);
  for (int i = 0; i < 20; ++i) {
    rig.guidance.cache(o);
    rig.guidance.update(rig.now, true);
  }
  EXPECT_EQ(rig.guidance.confirmations(), 1);
  EXPECT_FALSE(rig.guidance.handedOff());
  // Older (reordered) stamp neither counts nor resets.
  rig.guidance.cache(observation(0., .3, 0., rig.now - .05));
  rig.guidance.update(rig.now, true);
  EXPECT_EQ(rig.guidance.confirmations(), 1);
}

TEST(TagGuidance, WrongIdFrameStaleAndPreBeginDoNotCount) {
  Rig rig;
  rig.now += .1;
  rig.guidance.cache(observation(0., .3, 0., rig.now - .01, 1));  // wrong id
  rig.guidance.update(rig.now, true);
  EXPECT_EQ(rig.guidance.confirmations(), 0);
  rig.guidance.cache(observation(0., .3, 0., rig.now - .01, 0, "other_frame"));
  rig.guidance.update(rig.now, true);
  EXPECT_EQ(rig.guidance.confirmations(), 0);
  rig.guidance.cache(observation(0., .3, 0., rig.now - .30));  // age > 0.25
  rig.guidance.update(rig.now, true);
  EXPECT_EQ(rig.guidance.confirmations(), 0);
  rig.guidance.cache(observation(0., .3, 0., 99.0));  // before begin()
  rig.guidance.update(rig.now, true);
  EXPECT_EQ(rig.guidance.confirmations(), 0);
  TagObservation nan = observation(0., .3, 0., rig.now - .01);
  nan.x = std::numeric_limits<double>::quiet_NaN();
  rig.guidance.cache(nan);
  rig.guidance.update(rig.now, true);
  EXPECT_EQ(rig.guidance.confirmations(), 0);
  rig.guidance.cache(observation(0., -.3, 0., rig.now - .01));  // behind camera
  rig.guidance.update(rig.now, true);
  EXPECT_EQ(rig.guidance.confirmations(), 0);
  TagObservation rolled = observation(0., .3, 0., rig.now - .01);  // bogus roll
  const Eigen::Quaterniond rq(Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()));
  rolled.qx = rq.x();
  rolled.qy = rq.y();
  rolled.qz = rq.z();
  rolled.qw = rq.w();
  rig.guidance.cache(rolled);
  rig.guidance.update(rig.now, true);
  EXPECT_EQ(rig.guidance.confirmations(), 0);
  rig.guidance.cache(observation(0., .3, 0., rig.now - .01, 0, kFrame, true));  // z-out convention counts
  rig.guidance.update(rig.now, true);
  EXPECT_EQ(rig.guidance.confirmations(), 1);
}

TEST(TagGuidance, InvalidOrGapResetsConsecutiveCount) {
  Rig rig;
  rig.step(0., .3, 0.);
  rig.step(0., .3, 0.);
  EXPECT_EQ(rig.guidance.confirmations(), 2);
  rig.now += 1. / 30.;
  rig.guidance.cache(observation(0., .3, 0., rig.now - .01, 0, "other_frame"));
  rig.guidance.update(rig.now, true);
  EXPECT_EQ(rig.guidance.confirmations(), 0);
  rig.step(0., .3, 0.);
  EXPECT_EQ(rig.guidance.confirmations(), 1);
  for (int i = 0; i < 10; ++i) rig.tick();  // gap longer than max age
  EXPECT_EQ(rig.guidance.confirmations(), 0);
}

TEST(TagGuidance, ServoSignsClampAndDeadband) {
  TagGuidance g(testConfig());
  // Tag right of / farther than reference, normal tilted right: all positive.
  Eigen::Vector4d c = g.servo(0.01 + 0.015, 0.15 + 0.015, 0.05 + 0.05);
  EXPECT_GT(c[0], 0.);
  EXPECT_GT(c[1], 0.);
  EXPECT_EQ(c[2], 0.);
  EXPECT_GT(c[3], 0.);
  c = g.servo(0.01 - 0.015, 0.15 - 0.015, 0.05 - 0.05);
  EXPECT_LT(c[0], 0.);
  EXPECT_LT(c[1], 0.);
  EXPECT_LT(c[3], 0.);
  // Inside tolerance on every axis -> exact zero.
  EXPECT_EQ(g.servo(0.01 + 0.009, 0.15 - 0.009, 0.05 + 0.03), Eigen::Vector4d::Zero());
  // Clamp.
  c = g.servo(5., 5., 1.5);
  EXPECT_DOUBLE_EQ(c[0], 0.02);
  EXPECT_DOUBLE_EQ(c[1], 0.02);
  EXPECT_DOUBLE_EQ(c[3], 0.10);
  c = g.servo(-5., -5., -1.5);
  EXPECT_DOUBLE_EQ(c[0], -0.02);
  EXPECT_DOUBLE_EQ(c[1], -0.02);
  EXPECT_DOUBLE_EQ(c[3], -0.10);
  // Yaw error is wrapped: ref+3.2 rad is measured as ref+3.2-2pi, error -3.08 -> negative.
  c = g.servo(0.01, 0.15, wrapAngle(0.05 + 3.2));
  EXPECT_DOUBLE_EQ(c[3], -0.10);
}

// Planar world: tag fixed at (0, 1) with outward normal n_w = (sin phi, -cos phi).
// Robot pose (px, py, th): right = (cos th, sin th), forward = (-sin th, cos th);
// th increases CCW (yaw command > 0).  The camera sees the tag with
// x = d.right, z = d.forward, yaw = atan2(n.right, n.forward * -1).
struct World {
  double tx = 0., ty = 1., phi = 0.1;
  double px = 0., py = 0., th = 0.;
  Eigen::Vector3d seen() const {  // x, z, yaw in camera frame
    const Eigen::Vector2d r(std::cos(th), std::sin(th)), f(-std::sin(th), std::cos(th));
    const Eigen::Vector2d d(tx - px, ty - py), n(std::sin(phi), -std::cos(phi));
    return Eigen::Vector3d(d.dot(r), d.dot(f), std::atan2(n.dot(r), -n.dot(f)));
  }
  void apply(const Eigen::Vector4d& cmd, double dt) {
    const Eigen::Vector2d r(std::cos(th), std::sin(th)), f(-std::sin(th), std::cos(th));
    px += (cmd[0] * r[0] + cmd[1] * f[0]) * dt;
    py += (cmd[0] * r[1] + cmd[1] * f[1]) * dt;
    th += cmd[3] * dt;
  }
  // Place the robot so the tag is exactly at the reference, then offset it
  // (right/forward in the robot frame, yaw) as the initial error.
  void startFromReference(const TagGuidanceConfig& c, double right, double forward, double dyaw) {
    th = phi - c.ref_yaw_rad;
    const Eigen::Vector2d r(std::cos(th), std::sin(th)), f(-std::sin(th), std::cos(th));
    const Eigen::Vector2d p = Eigen::Vector2d(tx, ty) - c.ref_x_m * r - c.ref_z_m * f;
    px = p[0] + right * r[0] + forward * f[0];
    py = p[1] + right * r[1] + forward * f[1];
    th += dyaw;
  }
};

TEST(TagGuidance, WorldModelAgreesWithYawDefinition) {
  // Geometry check independent of the controller: a CCW body yaw of +d lowers
  // the seen tag yaw by d (yaw_seen = phi - th).
  World w;
  w.phi = 0.3;
  const double before = w.seen()[2];
  w.th = 0.1;
  EXPECT_NEAR(w.seen()[2], before - 0.1, 1e-12);
  EXPECT_NEAR(before, 0.3, 1e-12);
}

struct Result {
  bool success = false;
  double seconds = 0.;
  Eigen::Vector3d error;
};

Result runClosedLoop(const TagGuidanceConfig& config, double right, double forward,
                     double dyaw, double yaw_sign = 1., double max_seconds = 60.,
                     bool z_out = false) {
  World w;
  w.startFromReference(config, right, forward, dyaw);
  Rig rig(config);
  rig.z_out = z_out;
  const double dt = 1. / 30.;
  Result result;
  for (int k = 0; k < static_cast<int>(max_seconds / dt); ++k) {
    const Eigen::Vector3d s = w.seen();
    Eigen::Vector4d cmd = rig.step(s[0], s[1], s[2]);
    cmd[3] *= yaw_sign;
    w.apply(cmd, dt);
    if (rig.guidance.state() == TagState::Success) {
      result.success = true;
      break;
    }
    if (rig.guidance.state() == TagState::Failed) break;
  }
  result.seconds = rig.now - 100.;
  const Eigen::Vector3d s = w.seen();
  result.error = Eigen::Vector3d(s[0] - config.ref_x_m, s[1] - config.ref_z_m,
                                 wrapAngle(s[2] - config.ref_yaw_rad));
  return result;
}

TEST(TagGuidance, ClosedLoopConvergesFromSeveralOffsets) {
  const TagGuidanceConfig c = testConfig();
  const double offsets[][3] = {
      {0.05, 0., 0.},     {-0.05, 0., 0.},    {0., 0.05, 0.},      {0., -0.05, 0.},
      {0., 0., 0.1396},   {0., 0., -0.1396},  {0.05, 0.05, 0.1396}, {-0.05, -0.05, -0.1396},
      {0.05, -0.05, -0.1396}, {-0.05, 0.05, 0.1396}};
  for (const auto& o : offsets) {
    for (bool z_out : {false, true}) {
      const Result r = runClosedLoop(c, o[0], o[1], o[2], 1., 60., z_out);
      SCOPED_TRACE(testing::Message() << "offset " << o[0] << " " << o[1] << " " << o[2]
                                      << " z_out " << z_out);
      EXPECT_TRUE(r.success);
      EXPECT_LE(std::abs(r.error[0]), c.position_tolerance_m + 1e-9);
      EXPECT_LE(std::abs(r.error[1]), c.position_tolerance_m + 1e-9);
      EXPECT_LE(std::abs(r.error[2]), c.yaw_tolerance_rad + 1e-9);
      EXPECT_LT(r.seconds, 40.);
    }
  }
}

TEST(TagGuidance, OppositeYawSignDoesNotConverge) {
  const TagGuidanceConfig c = testConfig();
  for (bool z_out : {false, true}) {
    const Result r = runClosedLoop(c, 0., 0., 0.1396, -1., 20., z_out);
    EXPECT_FALSE(r.success);
    EXPECT_GT(std::abs(r.error[2]), 0.5);
  }
}

TEST(TagGuidance, SettleRequiresContinuousToleranceAndStoppedGait) {
  const TagGuidanceConfig c = testConfig();
  Rig rig;
  rig.handoff(c.ref_x_m, c.ref_z_m, c.ref_yaw_rad);
  ASSERT_EQ(rig.guidance.state(), TagState::Guiding);
  // Gait still moving: never succeeds, however long it stays in tolerance.
  for (int i = 0; i < 60; ++i) rig.step(c.ref_x_m, c.ref_z_m, c.ref_yaw_rad, false);
  EXPECT_EQ(rig.guidance.state(), TagState::Guiding);
  // Stopped: success only after tag_settle_s (0.5 s).
  const double t0 = rig.now + 1. / 30.;
  while (rig.guidance.state() == TagState::Guiding && rig.now - t0 < 2.) {
    rig.step(c.ref_x_m, c.ref_z_m, c.ref_yaw_rad, true);
  }
  EXPECT_EQ(rig.guidance.state(), TagState::Success);
  EXPECT_GE(rig.now - t0, 0.5 - 1e-9);
  EXPECT_LE(rig.now - t0, 0.5 + 0.1);
  EXPECT_EQ(rig.step(0.3, 0.3, 0.3), Eigen::Vector4d::Zero());  // terminal: zero
}

TEST(TagGuidance, LeavingToleranceRestartsSettle) {
  const TagGuidanceConfig c = testConfig();
  Rig rig;
  rig.handoff(c.ref_x_m, c.ref_z_m, c.ref_yaw_rad);
  for (int i = 0; i < 12; ++i) rig.step(c.ref_x_m, c.ref_z_m, c.ref_yaw_rad);  // 0.4 s
  EXPECT_EQ(rig.guidance.state(), TagState::Guiding);
  rig.step(c.ref_x_m + 0.02, c.ref_z_m, c.ref_yaw_rad);  // blip out of tolerance
  for (int i = 0; i < 12; ++i) rig.step(c.ref_x_m, c.ref_z_m, c.ref_yaw_rad);
  EXPECT_EQ(rig.guidance.state(), TagState::Guiding);
  for (int i = 0; i < 6; ++i) rig.step(c.ref_x_m, c.ref_z_m, c.ref_yaw_rad);
  EXPECT_EQ(rig.guidance.state(), TagState::Success);
}

TEST(TagGuidance, TagLostAfterHandoffTimesOut) {
  Rig rig;
  rig.handoff(0.05, 0.3, 0.1);
  ASSERT_EQ(rig.guidance.state(), TagState::Guiding);
  const double last = rig.now;
  while (rig.now - last < 0.95) rig.tick();
  EXPECT_EQ(rig.guidance.state(), TagState::Guiding);
  EXPECT_EQ(rig.tick(), Eigen::Vector4d::Zero());  // stale data never drives
  while (rig.now - last < 1.1) rig.tick();
  EXPECT_EQ(rig.guidance.state(), TagState::Failed);
  EXPECT_EQ(rig.guidance.reason(), "tag lost");
}

TEST(TagGuidance, AlignTimeoutWhenNeverConverging) {
  Rig rig;
  rig.handoff(0.05, 0.3, 0.1);
  const double start = rig.now;
  while (rig.guidance.state() == TagState::Guiding && rig.now - start < 70.) {
    rig.step(0.05, 0.3, 0.1);  // fresh but never converging
  }
  EXPECT_EQ(rig.guidance.state(), TagState::Failed);
  EXPECT_EQ(rig.guidance.reason(), "tag alignment timeout");
  EXPECT_NEAR(rig.now - start, 60., 0.1);
}

TEST(TagGuidance, ResumeDiscardsPrePauseDetectionsInCoarse) {
  Rig rig;
  for (int i = 0; i < 3; ++i) rig.step(0., .3, 0.);
  EXPECT_EQ(rig.guidance.confirmations(), 3);
  rig.now += 5.;  // pause
  rig.guidance.resume(rig.now);
  EXPECT_EQ(rig.guidance.confirmations(), 0);
  // A pre-pause stamp delivered late is not usable.
  rig.guidance.cache(observation(0., .3, 0., rig.now - .01 - 5.));
  rig.guidance.update(rig.now, true);
  EXPECT_EQ(rig.guidance.confirmations(), 0);
  for (int i = 0; i < 4; ++i) rig.step(0., .3, 0.);
  EXPECT_FALSE(rig.guidance.handedOff());
  rig.step(0., .3, 0.);
  EXPECT_TRUE(rig.guidance.handedOff());
}

TEST(TagGuidance, ResumeDiscardsPrePauseDetectionsWhileGuiding) {
  const TagGuidanceConfig c = testConfig();
  Rig rig;
  rig.handoff(0.05, 0.3, 0.1);
  for (int i = 0; i < 6; ++i) rig.step(c.ref_x_m, c.ref_z_m, c.ref_yaw_rad);  // 0.2 s settled
  rig.now += 0.5;  // short pause; old sample would be stale anyway
  rig.guidance.resume(rig.now);
  EXPECT_EQ(rig.guidance.state(), TagState::Guiding);
  // The pre-pause sample is gone: no command and no settle progress.
  EXPECT_EQ(rig.tick(), Eigen::Vector4d::Zero());
  // A late pre-pause delivery (stamp before resume) is not accepted either.
  rig.guidance.cache(observation(0.05, 0.3, 0.1, rig.now - 0.1));
  EXPECT_EQ(rig.guidance.update(rig.now, true), Eigen::Vector4d::Zero());
  // Loss clock restarted at resume, not at the last pre-pause sample.
  for (int i = 0; i < 20; ++i) rig.tick();
  EXPECT_EQ(rig.guidance.state(), TagState::Guiding);
  // Fresh post-resume sample drives again.
  const Eigen::Vector4d cmd = rig.step(0.05, 0.3, 0.1);
  EXPECT_GT(cmd[1], 0.);
  // Settle must restart from zero after resume.
  for (int i = 0; i < 12; ++i) rig.step(c.ref_x_m, c.ref_z_m, c.ref_yaw_rad);
  EXPECT_EQ(rig.guidance.state(), TagState::Guiding);
  for (int i = 0; i < 6; ++i) rig.step(c.ref_x_m, c.ref_z_m, c.ref_yaw_rad);
  EXPECT_EQ(rig.guidance.state(), TagState::Success);
}

TEST(TagGuidance, ConfigValidation) {
  TagGuidanceConfig c;
  c.confirm_frames = 0;
  EXPECT_THROW(TagGuidance{c}, std::invalid_argument);
  c = TagGuidanceConfig();
  c.max_yaw_rate = 0.;
  EXPECT_THROW(TagGuidance{c}, std::invalid_argument);
  EXPECT_NO_THROW(TagGuidance{TagGuidanceConfig()});
}

}  // namespace
}  // namespace grasp_hexapod_bt_control
