#include <array>
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
  c.p0_z_m = 0.16;  // z_target = p0 - overshoot (0.01) = 0.15
  c.ref_yaw_rad = 0.05;
  return c;
}

double zTarget(const TagGuidanceConfig& c) { return c.p0_z_m - c.final_overshoot_m; }

// Feeds fresh samples at 30 Hz.  Extra inputs (IMU, feet, gait flags) default to
// "IMU valid, feet at rest, all-stance mask".
struct Rig {
  explicit Rig(TagGuidanceConfig c = testConfig()) : guidance(c) {
    guidance.begin(now);
    for (auto& foot : feet) foot = Eigen::Vector3d(0., 0.1, -0.1);
  }
  TagGuidanceInput input(bool gait_stopped) const {
    TagGuidanceInput in;
    in.gait_started = !gait_stopped;
    in.transfer_active = transfer_active;
    in.stance_mask = stance_mask;
    in.feet_valid = true;
    in.feet_base = feet;
    in.imu_valid = imu_valid;
    in.imu_yaw = imu_yaw;
    return in;
  }
  Eigen::Vector4d step(double x, double z, double yaw, bool gait_stopped = true) {
    now += 1. / 30.;
    guidance.cache(observation(x, z, yaw, now - .01, 0, kFrame, z_out));
    return guidance.update(now, input(gait_stopped));
  }
  Eigen::Vector4d tick(bool gait_stopped = true) {  // control tick, no new sample
    now += 1. / 30.;
    return guidance.update(now, input(gait_stopped));
  }
  void shiftFeet(double dy) {
    for (auto& foot : feet) foot.y() += dy;
  }
  void handoff(double x, double z, double yaw) {
    for (int i = 0; i < 5 && !guidance.handedOff(); ++i) step(x, z, yaw);
  }
  TagGuidance guidance;
  double now = 100.0;
  bool z_out = false;
  bool imu_valid = true;
  double imu_yaw = 0.2;
  bool transfer_active = false;
  std::array<bool, 6> stance_mask{{true, true, true, true, true, true}};
  std::array<Eigen::Vector3d, 6> feet;
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
  EXPECT_EQ(rig.guidance.state(), TagState::Aligning);
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
  Eigen::Vector4d c = g.servo(0.01 + 0.015, 0.15 + 0.015, 0.05 + 0.05, true);
  EXPECT_GT(c[0], 0.);
  EXPECT_GT(c[1], 0.);
  EXPECT_EQ(c[2], 0.);
  EXPECT_GT(c[3], 0.);
  EXPECT_NEAR(c[1], 0.5 * 0.015, 1e-12);
  c = g.servo(0.01 - 0.015, 0.15 - 0.015, 0.05 - 0.05, true);
  EXPECT_LT(c[0], 0.);
  EXPECT_EQ(c[1], 0.);  // never backward: tag nearer than the target
  EXPECT_LT(c[3], 0.);
  // Deadband is kServoDeadbandFraction (0.5) x tolerance on every axis: exact
  // zero inside, a command just outside (still inside the 1x tolerance).
  ASSERT_DOUBLE_EQ(kServoDeadbandFraction, 0.5);
  EXPECT_EQ(g.servo(0.01 + 0.0049, 0.15 - 0.009, 0.05 + 0.0173, true), Eigen::Vector4d::Zero());
  EXPECT_EQ(g.servo(0.01, 0.15 + 0.0049, 0.05, true), Eigen::Vector4d::Zero());  // ez deadband
  EXPECT_NE(g.servo(0.01 + 0.0051, 0.15, 0.05, true)[0], 0.);
  EXPECT_NE(g.servo(0.01, 0.15, 0.05 + 0.0177, true)[3], 0.);
  EXPECT_NE(g.servo(0.01, 0.15 + 0.0051, 0.05, true)[1], 0.);
  // Clamp.
  c = g.servo(5., 5., 1.5, true);
  EXPECT_DOUBLE_EQ(c[0], 0.02);
  EXPECT_DOUBLE_EQ(c[1], 0.02);
  EXPECT_DOUBLE_EQ(c[3], 0.10);
  c = g.servo(-5., -5., -1.5, true);
  EXPECT_DOUBLE_EQ(c[0], -0.02);
  EXPECT_EQ(c[1], 0.);
  EXPECT_DOUBLE_EQ(c[3], -0.10);
  // Forward is gated off without touching lateral / yaw.
  c = g.servo(5., 5., 1.5, false);
  EXPECT_DOUBLE_EQ(c[0], 0.02);
  EXPECT_EQ(c[1], 0.);
  EXPECT_DOUBLE_EQ(c[3], 0.10);
  // Yaw error is wrapped: ref+3.2 rad is measured as ref+3.2-2pi, error -3.08 -> negative.
  c = g.servo(0.01, 0.15, wrapAngle(0.05 + 3.2), true);
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
    const Eigen::Vector2d p = Eigen::Vector2d(tx, ty) - c.ref_x_m * r - zTarget(c) * f;
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
  result.error = Eigen::Vector3d(s[0] - config.ref_x_m, s[1] - zTarget(config),
                                 wrapAngle(s[2] - config.ref_yaw_rad));
  return result;
}

TEST(TagGuidance, ClosedLoopConvergesFromSeveralOffsets) {
  const TagGuidanceConfig c = testConfig();
  // Forward offsets are <= 0 (robot behind the target): the policy never backs up.
  const double offsets[][3] = {
      {0.05, 0., 0.},     {-0.05, 0., 0.},    {0., -0.05, 0.},      {0.05, -0.05, 0.},
      {0., 0., 0.1396},   {0., 0., -0.1396},  {0.05, -0.05, 0.1396}, {-0.05, -0.05, -0.1396},
      {0.05, -0.02, -0.1396}, {-0.05, -0.02, 0.1396}};
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
  const double zt = zTarget(c);
  rig.handoff(c.ref_x_m, zt, c.ref_yaw_rad);
  ASSERT_TRUE(rig.guidance.handedOff());
  // Gait still moving: never succeeds, however long it stays in tolerance.
  for (int i = 0; i < 60; ++i) rig.step(c.ref_x_m, zt, c.ref_yaw_rad, false);
  EXPECT_EQ(rig.guidance.state(), TagState::Advancing);
  // Stopped: success only after tag_settle_s (0.5 s).
  const double t0 = rig.now + 1. / 30.;
  while (rig.guidance.state() == TagState::Advancing && rig.now - t0 < 2.) {
    rig.step(c.ref_x_m, zt, c.ref_yaw_rad, true);
  }
  EXPECT_EQ(rig.guidance.state(), TagState::Success);
  EXPECT_GE(rig.now - t0, 0.5 - 1e-9);
  EXPECT_LE(rig.now - t0, 0.5 + 0.1);
  EXPECT_EQ(rig.step(0.3, 0.3, 0.3), Eigen::Vector4d::Zero());  // terminal: zero
}

TEST(TagGuidance, LeavingToleranceRestartsSettle) {
  const TagGuidanceConfig c = testConfig();
  const double zt = zTarget(c);
  Rig rig;
  rig.handoff(c.ref_x_m, zt, c.ref_yaw_rad);
  for (int i = 0; i < 12; ++i) rig.step(c.ref_x_m, zt, c.ref_yaw_rad);  // 0.4 s
  EXPECT_EQ(rig.guidance.state(), TagState::Advancing);
  rig.step(c.ref_x_m + 0.02, zt, c.ref_yaw_rad);  // blip out of tolerance
  for (int i = 0; i < 12; ++i) rig.step(c.ref_x_m, zt, c.ref_yaw_rad);
  EXPECT_EQ(rig.guidance.state(), TagState::Advancing);
  for (int i = 0; i < 6; ++i) rig.step(c.ref_x_m, zt, c.ref_yaw_rad);
  EXPECT_EQ(rig.guidance.state(), TagState::Success);
}

TEST(TagGuidance, TagLostAfterHandoffTimesOut) {
  Rig rig;
  rig.handoff(0.05, 0.3, 0.1);  // misaligned: stays in Aligning
  ASSERT_EQ(rig.guidance.state(), TagState::Aligning);
  const double last = rig.now;
  while (rig.now - last < 0.95) rig.tick();
  EXPECT_EQ(rig.guidance.state(), TagState::Aligning);
  EXPECT_EQ(rig.tick(), Eigen::Vector4d::Zero());  // stale data never drives
  while (rig.now - last < 1.1) rig.tick();
  EXPECT_EQ(rig.guidance.state(), TagState::Failed);
  EXPECT_EQ(rig.guidance.reason(), "tag lost");
}

TEST(TagGuidance, AlignTimeoutWhenNeverConverging) {
  Rig rig;
  rig.handoff(0.05, 0.3, 0.1);
  const double start = rig.now;
  while (rig.guidance.state() == TagState::Aligning && rig.now - start < 70.) {
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

TEST(TagGuidance, ResumeDiscardsPrePauseDetectionsWhileAdvancing) {
  const TagGuidanceConfig c = testConfig();
  const double zt = zTarget(c);
  Rig rig;
  rig.handoff(c.ref_x_m, zt, c.ref_yaw_rad);
  for (int i = 0; i < 6; ++i) rig.step(c.ref_x_m, zt, c.ref_yaw_rad);  // 0.2 s settled
  rig.now += 0.5;  // short pause; old sample would be stale anyway
  rig.guidance.resume(rig.now);
  EXPECT_EQ(rig.guidance.state(), TagState::Advancing);
  // The pre-pause sample is gone: no command and no settle progress.
  EXPECT_EQ(rig.tick(), Eigen::Vector4d::Zero());
  // A late pre-pause delivery (stamp before resume) is not accepted either.
  rig.guidance.cache(observation(c.ref_x_m, 0.3, c.ref_yaw_rad, rig.now - 0.1));
  EXPECT_EQ(rig.guidance.update(rig.now, true), Eigen::Vector4d::Zero());
  // Fresh post-resume sample drives again.
  const Eigen::Vector4d cmd = rig.step(c.ref_x_m, 0.3, c.ref_yaw_rad);
  EXPECT_GT(cmd[1], 0.);
  // Settle must restart from zero after resume.
  for (int i = 0; i < 12; ++i) rig.step(c.ref_x_m, zt, c.ref_yaw_rad);
  EXPECT_EQ(rig.guidance.state(), TagState::Advancing);
  for (int i = 0; i < 6; ++i) rig.step(c.ref_x_m, zt, c.ref_yaw_rad);
  EXPECT_EQ(rig.guidance.state(), TagState::Success);
}

TEST(TagGuidance, ResumeRestartsLossClockAndDropsLastSample) {
  const TagGuidanceConfig c = testConfig();
  Rig rig;
  rig.handoff(c.ref_x_m, 0.2, c.ref_yaw_rad);
  for (int i = 0; i < 4; ++i) rig.step(c.ref_x_m, 0.2, c.ref_yaw_rad);
  ASSERT_EQ(rig.guidance.state(), TagState::Advancing);
  rig.now += 5.;  // pause
  rig.guidance.resume(rig.now);
  for (int i = 0; i < 8; ++i) rig.tick();  // 0.27 s < handover_loss_s
  EXPECT_EQ(rig.guidance.state(), TagState::Advancing);
  for (int i = 0; i < 4; ++i) rig.tick();
  // The pre-pause sample must not be used for the handover.
  EXPECT_EQ(rig.guidance.state(), TagState::Failed);
  EXPECT_EQ(rig.guidance.reason(), "tag lost before alignment");
}

// ---------------------------------------------------------------------------
// Align gate, advance, handover.
// ---------------------------------------------------------------------------

TEST(TagGuidance, EffectiveStanceRule) {
  const std::array<bool, 6> a{{true, true, false, false, false, true}};
  const std::array<bool, 6> all{{true, true, true, true, true, true}};
  EXPECT_EQ(effectiveStance(true, false, a), a);      // step in progress: raw mask
  EXPECT_EQ(effectiveStance(true, true, a), all);     // transfer: all stance
  EXPECT_EQ(effectiveStance(false, false, a), all);   // gait stopped: raw mask is stale
  EXPECT_EQ(effectiveStance(false, true, a), all);
}

TEST(TagGuidance, AlignGateBlocksForwardUntilInsideTwiceTolerance) {
  const TagGuidanceConfig c = testConfig();
  Rig rig;
  rig.handoff(c.ref_x_m + 0.03, 0.4, c.ref_yaw_rad);  // ex = 3 cm > 2 cm gate
  ASSERT_EQ(rig.guidance.state(), TagState::Aligning);
  for (int i = 0; i < 5; ++i) {
    const Eigen::Vector4d cmd = rig.step(c.ref_x_m + 0.03, 0.4, c.ref_yaw_rad);
    EXPECT_GT(cmd[0], 0.);
    EXPECT_EQ(cmd[1], 0.);
  }
  EXPECT_EQ(rig.guidance.state(), TagState::Aligning);
  // Yaw outside twice the tolerance blocks forward as well.
  Eigen::Vector4d cmd = rig.step(c.ref_x_m, 0.4, c.ref_yaw_rad + 0.08);
  EXPECT_EQ(cmd[1], 0.);
  EXPECT_GT(cmd[3], 0.);
  EXPECT_EQ(rig.guidance.state(), TagState::Aligning);
  // Inside the 2x gate (but outside 1x): forward starts, lateral keeps correcting.
  cmd = rig.step(c.ref_x_m + 0.015, 0.4, c.ref_yaw_rad);
  EXPECT_EQ(rig.guidance.state(), TagState::Advancing);
  EXPECT_GT(cmd[1], 0.);
  EXPECT_GT(cmd[0], 0.);
}

TEST(TagGuidance, AdvancingForwardLawAndRegating) {
  const TagGuidanceConfig c = testConfig();
  const double zt = zTarget(c);
  Rig rig;
  rig.handoff(c.ref_x_m, 0.4, c.ref_yaw_rad);
  Eigen::Vector4d cmd = rig.step(c.ref_x_m, 0.4, c.ref_yaw_rad);
  ASSERT_EQ(rig.guidance.state(), TagState::Advancing);
  EXPECT_DOUBLE_EQ(cmd[1], c.max_forward_speed_m_s);  // clamped
  cmd = rig.step(c.ref_x_m, zt + 0.02, c.ref_yaw_rad);
  EXPECT_NEAR(cmd[1], c.forward_gain * 0.02, 1e-12);
  // Out of the gate: forward back to 0, state stays Advancing, then resumes.
  cmd = rig.step(c.ref_x_m + 0.03, 0.4, c.ref_yaw_rad);
  EXPECT_EQ(cmd[1], 0.);
  EXPECT_GT(cmd[0], 0.);
  EXPECT_EQ(rig.guidance.state(), TagState::Advancing);
  cmd = rig.step(c.ref_x_m, 0.4, c.ref_yaw_rad + 0.08);
  EXPECT_EQ(cmd[1], 0.);
  cmd = rig.step(c.ref_x_m, 0.4, c.ref_yaw_rad);
  EXPECT_GT(cmd[1], 0.);
  // Past the target: never backward.
  cmd = rig.step(c.ref_x_m, zt - 0.05, c.ref_yaw_rad);
  EXPECT_EQ(cmd[1], 0.);
}

// Aligned Advancing run at tag depth z, then no more samples until the state
// leaves Advancing.  Returns the command of the handover tick.
Eigen::Vector4d advanceThenLose(Rig& rig, double x, double z, double yaw) {
  rig.handoff(x, z, yaw);
  for (int i = 0; i < 5; ++i) rig.step(x, z, yaw, false);
  EXPECT_EQ(rig.guidance.state(), TagState::Advancing);
  Eigen::Vector4d cmd = Eigen::Vector4d::Zero();
  const double last = rig.now;
  while (rig.guidance.state() == TagState::Advancing && rig.now - last < 1.) {
    cmd = rig.tick(false);
  }
  return cmd;
}

TEST(TagGuidance, HandoverOnLossUsesLastSampleRemaining) {
  const TagGuidanceConfig c = testConfig();
  Rig rig;
  rig.imu_yaw = 0.7;
  const double t0 = rig.now;
  const Eigen::Vector4d cmd = advanceThenLose(rig, c.ref_x_m + 0.004, 0.20, c.ref_yaw_rad - 0.01);
  ASSERT_EQ(rig.guidance.state(), TagState::Blind);
  const TagHandover& h = rig.guidance.handover();
  EXPECT_TRUE(h.occurred);
  EXPECT_NEAR(h.z_last, 0.20, 1e-12);
  EXPECT_NEAR(h.ex, 0.004, 1e-12);
  EXPECT_NEAR(h.eyaw, -0.01, 1e-12);
  EXPECT_NEAR(h.remaining, 0.20 - 0.15, 1e-12);
  // Tag-aligned heading target: IMU heading at the last sample + eyaw_last.
  EXPECT_NEAR(rig.guidance.yawHold(), 0.7 - 0.01, 1e-12);
  EXPECT_NEAR(h.heading_target_offset, -0.01, 1e-12);
  EXPECT_EQ(cmd[0], 0.);
  EXPECT_DOUBLE_EQ(cmd[1], c.blind_speed_m_s);
  EXPECT_NEAR(cmd[3], c.heading_gain * -0.01, 1e-12);  // steers toward the tag-aligned target
  // Loss is detected only after tag_handover_loss_s without a fresh sample.
  EXPECT_GT(rig.now - t0, 0.3);
}

TEST(TagGuidance, HandoverRefusedWhenLastSampleMisaligned) {
  const TagGuidanceConfig c = testConfig();
  {  // lateral between 1x and 2x tolerance: allowed to advance, not to hand over
    Rig rig;
    advanceThenLose(rig, c.ref_x_m + 0.015, 0.20, c.ref_yaw_rad);
    EXPECT_EQ(rig.guidance.state(), TagState::Failed);
    EXPECT_EQ(rig.guidance.reason(), "tag lost before alignment");
  }
  {  // yaw between 1x and 2x tolerance
    Rig rig;
    advanceThenLose(rig, c.ref_x_m, 0.20, c.ref_yaw_rad + 0.05);
    EXPECT_EQ(rig.guidance.state(), TagState::Failed);
    EXPECT_EQ(rig.guidance.reason(), "tag lost before alignment");
  }
}

TEST(TagGuidance, HandoverFailsWhenTooFarFromP0) {
  const TagGuidanceConfig c = testConfig();
  Rig rig;
  advanceThenLose(rig, c.ref_x_m, 0.50, c.ref_yaw_rad);  // remaining 0.35 > 0.25
  EXPECT_EQ(rig.guidance.state(), TagState::Failed);
  EXPECT_EQ(rig.guidance.reason(), "tag lost too far from P0 (remaining 0.350 m)");
  // Exactly at the limit is allowed.
  Rig ok;
  advanceThenLose(ok, c.ref_x_m, 0.15 + 0.25, c.ref_yaw_rad);
  EXPECT_EQ(ok.guidance.state(), TagState::Blind);
}

TEST(TagGuidance, HandoverWithNothingRemainingGoesStraightToSettle) {
  const TagGuidanceConfig c = testConfig();
  Rig rig;
  rig.imu_valid = false;  // not needed when nothing is left to drive
  const Eigen::Vector4d cmd = advanceThenLose(rig, c.ref_x_m, 0.13, c.ref_yaw_rad);  // z < z_target
  EXPECT_EQ(rig.guidance.state(), TagState::Settling);
  EXPECT_EQ(cmd, Eigen::Vector4d::Zero());
  EXPECT_LE(rig.guidance.handover().remaining, 0.);
  // Gait still running: no success.
  for (int i = 0; i < 30; ++i) EXPECT_EQ(rig.tick(false), Eigen::Vector4d::Zero());
  EXPECT_EQ(rig.guidance.state(), TagState::Settling);
  const double t0 = rig.now + 1. / 30.;
  while (rig.guidance.state() == TagState::Settling && rig.now - t0 < 2.) rig.tick(true);
  EXPECT_EQ(rig.guidance.state(), TagState::Success);
  EXPECT_GE(rig.now - t0, 0.5 - 1e-9);
}

TEST(TagGuidance, MinReliableDepthTriggersHandoverWhileTagStillVisible) {
  TagGuidanceConfig c = testConfig();
  c.min_reliable_z_m = 0.20;
  Rig rig(c);
  rig.handoff(c.ref_x_m, 0.30, c.ref_yaw_rad);
  for (int i = 0; i < 5; ++i) rig.step(c.ref_x_m, 0.30, c.ref_yaw_rad, false);
  EXPECT_EQ(rig.guidance.state(), TagState::Advancing);
  rig.step(c.ref_x_m, 0.19, c.ref_yaw_rad, false);  // fresh, valid, but too close to trust
  ASSERT_EQ(rig.guidance.state(), TagState::Blind);
  EXPECT_NEAR(rig.guidance.handover().remaining, 0.19 - 0.15, 1e-12);
}

TEST(TagGuidance, LossBeforeAdvancingKeepsTagLossTimeout) {
  const TagGuidanceConfig c = testConfig();
  Rig rig;
  rig.handoff(c.ref_x_m + 0.05, 0.3, c.ref_yaw_rad);  // outside the gate: Aligning
  const double last = rig.now;
  while (rig.now - last < 0.9) rig.tick();
  EXPECT_EQ(rig.guidance.state(), TagState::Aligning);  // no 0.3 s handover here
  while (rig.now - last < 1.1) rig.tick();
  EXPECT_EQ(rig.guidance.state(), TagState::Failed);
  EXPECT_EQ(rig.guidance.reason(), "tag lost");
}

TEST(TagGuidance, AlignTimeoutAlsoCoversAdvancing) {
  const TagGuidanceConfig c = testConfig();
  Rig rig;
  rig.handoff(c.ref_x_m, 0.4, c.ref_yaw_rad);
  const double start = rig.now;
  while (rig.guidance.state() != TagState::Failed && rig.now - start < 70.) {
    rig.step(c.ref_x_m, 0.4, c.ref_yaw_rad, false);  // advancing, never arrives
  }
  EXPECT_EQ(rig.guidance.reason(), "tag alignment timeout");
}

// ---------------------------------------------------------------------------
// Blind finish with directly fed odometry / IMU.
// ---------------------------------------------------------------------------

// Enters Blind at remaining 0.05 m (z_last 0.20) with IMU yaw 0.2.
void enterBlind(Rig& rig) {
  const TagGuidanceConfig c = testConfig();
  advanceThenLose(rig, c.ref_x_m, 0.20, c.ref_yaw_rad);
  ASSERT_EQ(rig.guidance.state(), TagState::Blind);
}

TEST(TagGuidance, BlindHeadingHoldSignAndClamp) {
  TagGuidanceConfig c = testConfig();
  c.heading_gain = 5.;
  Rig rig(c);
  enterBlind(rig);
  EXPECT_NEAR(rig.guidance.yawHold(), 0.2, 1e-12);
  // Robot rotated CW (imu yaw decreased): positive (CCW) yaw command.
  rig.imu_yaw = 0.2 - 0.01;
  Eigen::Vector4d cmd = rig.tick(false);
  EXPECT_NEAR(cmd[3], 5. * 0.01, 1e-12);
  EXPECT_NEAR(rig.guidance.headingError(), 0.01, 1e-12);
  rig.imu_yaw = 0.2 + 0.01;
  cmd = rig.tick(false);
  EXPECT_NEAR(cmd[3], -5. * 0.01, 1e-12);
  rig.imu_yaw = 0.2 + 0.05;  // gain 5 * 0.05 = 0.25 -> clamp to 0.10
  cmd = rig.tick(false);
  EXPECT_DOUBLE_EQ(cmd[3], -c.max_yaw_rate);
  EXPECT_EQ(cmd[0], 0.);
  EXPECT_DOUBLE_EQ(cmd[1], c.blind_speed_m_s);
  // Wrap-around: yaw hold near +pi, measurement just across -pi.
  Rig wrap(c);
  wrap.imu_yaw = kPi - 0.005;
  enterBlind(wrap);
  wrap.imu_yaw = -kPi + 0.005;  // rotated CCW by 0.01
  cmd = wrap.tick(false);
  EXPECT_NEAR(cmd[3], -5. * 0.01, 1e-9);
}

TEST(TagGuidance, BlindHeadingErrorFails) {
  const TagGuidanceConfig c = testConfig();
  for (double sign : {1., -1.}) {
    Rig rig;
    enterBlind(rig);
    rig.imu_yaw = 0.2 + sign * 0.08;
    rig.tick(false);
    EXPECT_EQ(rig.guidance.state(), TagState::Blind);
    rig.imu_yaw = 0.2 + sign * (c.max_heading_error_rad + 0.002);
    EXPECT_EQ(rig.tick(false), Eigen::Vector4d::Zero());
    EXPECT_EQ(rig.guidance.state(), TagState::Failed);
    EXPECT_EQ(rig.guidance.reason(), "blind heading error");
  }
}

TEST(TagGuidance, BlindImuInvalidFails) {
  {  // at handover
    Rig rig;
    rig.imu_valid = false;
    advanceThenLose(rig, testConfig().ref_x_m, 0.20, testConfig().ref_yaw_rad);
    EXPECT_EQ(rig.guidance.state(), TagState::Failed);
    EXPECT_EQ(rig.guidance.reason(), "imu unavailable for blind finish");
  }
  {  // during blind
    Rig rig;
    enterBlind(rig);
    rig.imu_valid = false;
    EXPECT_EQ(rig.tick(false), Eigen::Vector4d::Zero());
    EXPECT_EQ(rig.guidance.state(), TagState::Failed);
    EXPECT_EQ(rig.guidance.reason(), "imu unavailable for blind finish");
  }
}

TEST(TagGuidance, OdometryUsesStanceLegsOnly) {
  Rig rig;
  enterBlind(rig);
  const double base = rig.guidance.odometry();
  // Tripod A (0,1,5) stance, B swinging forward 3x faster: odometry follows A.
  rig.stance_mask = {{true, true, false, false, false, true}};
  rig.tick(false);  // baseline for the new mask
  const double start = rig.guidance.odometry();
  EXPECT_NEAR(start - base, 0., 1e-9);
  for (int i = 0; i < 4; ++i) {
    for (int leg : {0, 1, 5}) rig.feet[leg].y() -= 0.0005;
    for (int leg : {2, 3, 4}) rig.feet[leg].y() += 0.0015;
    rig.tick(false);
  }
  EXPECT_NEAR(rig.guidance.odometry() - start, 4 * 0.0005, 1e-12);
  // Frame where the mask flips without a transfer: only legs stance in both
  // frames count (none here), so the swing-leg motion cannot leak in.
  const double before = rig.guidance.odometry();
  rig.stance_mask = {{false, false, true, true, true, false}};
  for (int leg : {0, 1, 5}) rig.feet[leg].y() -= 0.0005;
  for (int leg : {2, 3, 4}) rig.feet[leg].y() += 0.0015;
  rig.tick(false);
  EXPECT_NEAR(rig.guidance.odometry(), before, 1e-12);
  // Transfer frames are all-stance: every leg that moved back counts.
  rig.transfer_active = true;
  rig.tick(false);  // baseline: previous frame was not all-stance for A legs
  const double t0 = rig.guidance.odometry();
  rig.shiftFeet(-0.0004);
  rig.tick(false);
  EXPECT_NEAR(rig.guidance.odometry() - t0, 0.0004, 1e-12);
  // Gait stopped: all six legs are trusted despite the stale raw mask.
  rig.transfer_active = false;
  rig.tick(true);
  const double s0 = rig.guidance.odometry();
  rig.shiftFeet(-0.0004);
  rig.tick(true);
  EXPECT_NEAR(rig.guidance.odometry() - s0, 0.0004, 1e-12);
}

TEST(TagGuidance, BlindStopsAtRemainingThenSettlesAndSucceeds) {
  const TagGuidanceConfig c = testConfig();
  Rig rig;
  enterBlind(rig);
  const double remaining = rig.guidance.blindRemaining();
  EXPECT_NEAR(remaining, 0.05, 1e-12);
  // Feet move exactly as commanded: 0.015 m/s at 30 Hz = 0.5 mm per frame.
  int frames = 0;
  Eigen::Vector4d cmd = Eigen::Vector4d::Zero();
  while (rig.guidance.state() == TagState::Blind && frames++ < 500) {
    rig.shiftFeet(-c.blind_speed_m_s / 30.);
    cmd = rig.tick(false);
  }
  EXPECT_EQ(rig.guidance.state(), TagState::Settling);
  EXPECT_EQ(cmd, Eigen::Vector4d::Zero());
  EXPECT_GE(rig.guidance.odometry(), remaining);
  EXPECT_NEAR(rig.guidance.odometry(), remaining, 0.001);
  EXPECT_NEAR(rig.guidance.commandIntegral(), rig.guidance.odometry(), 0.001);
  // Settling waits for the gait to stop plus tag_settle_s; coasting odometry
  // keeps being tracked and never triggers the mismatch check.
  for (int i = 0; i < 20; ++i) {
    rig.shiftFeet(-0.0002);
    EXPECT_EQ(rig.tick(false), Eigen::Vector4d::Zero());
  }
  EXPECT_EQ(rig.guidance.state(), TagState::Settling);
  const double t0 = rig.now + 1. / 30.;
  while (rig.guidance.state() == TagState::Settling && rig.now - t0 < 2.) rig.tick(true);
  EXPECT_EQ(rig.guidance.state(), TagState::Success);
  EXPECT_GE(rig.now - t0, 0.5 - 1e-9);
}

TEST(TagGuidance, BlindOdometryMismatchFails) {
  const TagGuidanceConfig c = testConfig();
  {  // feet do not follow the command (blocked legs)
    Rig rig;
    enterBlind(rig);
    int frames = 0;
    while (rig.guidance.state() == TagState::Blind && frames++ < 200) rig.tick(false);
    EXPECT_EQ(rig.guidance.state(), TagState::Failed);
    EXPECT_EQ(rig.guidance.reason(), "blind odometry mismatch (blocked or slipping legs?)");
    // Fails at about tol / speed = 2 s of commanded motion.
    EXPECT_NEAR(frames / 30., c.odom_mismatch_m / c.blind_speed_m_s, 0.1);
  }
  {  // feet move much faster than commanded
    Rig rig;
    enterBlind(rig);
    int frames = 0;
    while (rig.guidance.state() == TagState::Blind && frames++ < 200) {
      rig.shiftFeet(-0.0002);
      rig.tick(false);
    }
    EXPECT_EQ(rig.guidance.state(), TagState::Failed);
  }
}

TEST(TagGuidance, BlindTimeoutFails) {
  TagGuidanceConfig c = testConfig();
  c.blind_timeout_s = 1.;
  Rig rig(c);
  enterBlind(rig);
  while (rig.guidance.state() == TagState::Blind && rig.now < 200.) {
    rig.shiftFeet(-c.blind_speed_m_s / 30. * 0.9);  // consistent, but too slow to finish in 1 s
    rig.tick(false);
  }
  EXPECT_EQ(rig.guidance.state(), TagState::Failed);
  EXPECT_EQ(rig.guidance.reason(), "blind finish timeout");
}

TEST(TagGuidance, PauseDuringBlindFreezesAccumulation) {
  const TagGuidanceConfig c = testConfig();
  Rig rig;
  enterBlind(rig);
  for (int i = 0; i < 30; ++i) {
    rig.shiftFeet(-c.blind_speed_m_s / 30.);
    rig.tick(false);
  }
  const double odom = rig.guidance.odometry();
  const double integral = rig.guidance.commandIntegral();
  const double hold = rig.guidance.yawHold();
  ASSERT_GT(odom, 0.01);
  ASSERT_EQ(rig.guidance.state(), TagState::Blind);
  // Pause: 10 s with no control ticks, feet reposition by 5 cm and the IMU
  // yaw changes while held.
  rig.now += 10.;
  rig.shiftFeet(-0.05);
  rig.imu_yaw += 0.03;
  rig.guidance.resume(rig.now);
  EXPECT_DOUBLE_EQ(rig.guidance.odometry(), odom);
  EXPECT_DOUBLE_EQ(rig.guidance.commandIntegral(), integral);
  EXPECT_DOUBLE_EQ(rig.guidance.yawHold(), hold);
  const Eigen::Vector4d cmd = rig.tick(false);
  EXPECT_EQ(rig.guidance.state(), TagState::Blind);
  EXPECT_DOUBLE_EQ(rig.guidance.odometry(), odom);       // new baseline, no jump
  EXPECT_DOUBLE_EQ(rig.guidance.commandIntegral(), integral);  // no 10 s of command
  EXPECT_DOUBLE_EQ(rig.guidance.yawHold(), hold);        // heading hold kept
  EXPECT_NEAR(rig.guidance.headingError(), -0.03, 1e-12);
  EXPECT_GT(cmd[1], 0.);
  // It carries on normally afterwards.
  rig.shiftFeet(-c.blind_speed_m_s / 30.);
  rig.tick(false);
  EXPECT_NEAR(rig.guidance.odometry() - odom, c.blind_speed_m_s / 30., 1e-12);
  EXPECT_NEAR(rig.guidance.commandIntegral() - integral, c.blind_speed_m_s / 30., 1e-12);
}

// ---------------------------------------------------------------------------
// Closed loop: planar robot, camera blind close to P0, leg odometry, IMU.
// ---------------------------------------------------------------------------
struct Sim {
  explicit Sim(const TagGuidanceConfig& c) : config(c), guidance(c) {
    guidance.begin(now);
    for (int leg = 0; leg < 6; ++leg) feet[leg] = Eigen::Vector3d(leg < 3 ? -0.1 : 0.1, 0.02 * leg, -0.1);
  }
  // Put the robot at P0 (tag at the virtual depth p0_z, ex=0, eyaw=0) and then
  // offset it: `back` metres behind P0, `right`, `dyaw`.
  void start(double back, double right, double dyaw) {
    World p0;
    p0.startFromReference(config, 0., 0., 0.);  // z_target; shift to P0 below
    w = p0;
    w.startFromReference(config, right, -config.final_overshoot_m - back, dyaw);
    // startFromReference works at z_target: z_target + overshoot = p0, so the
    // forward offset above measured from z_target places us `back` behind P0.
  }
  void step() {
    now += dt;
    const Eigen::Vector3d s = w.seen();
    if (s[1] > invisible_z) guidance.cache(observation(s[0], s[1], s[2], now - .01));
    TagGuidanceInput in;
    in.gait_started = gait_active;
    in.transfer_active = transfer;
    in.stance_mask = group == 0 ? std::array<bool, 6>{{true, true, false, false, false, true}}
                                : std::array<bool, 6>{{false, false, true, true, true, false}};
    in.feet_valid = true;
    in.feet_base = feet;
    in.imu_valid = imu_ok;
    in.imu_yaw = wrapAngle(imu_sign * (w.th + imu_bias));
    travel_seen = travel;
    cmd = guidance.update(now, in);
    if (!blind_seen && guidance.state() == TagState::Blind) {
      blind_seen = true;
      travel_at_blind = travel_seen;
    }
    // Plant: first-order forward speed lag, slip factor, heading drift.
    const bool nonzero = cmd.norm() > 1e-9;
    if (nonzero) gait_end = now + 0.3;
    const double target_v = body_blocked ? 0. : cmd[1] * slip;
    v += (target_v - v) * dt / tau;
    const double dy = v * dt;
    w.apply(Eigen::Vector4d(body_blocked ? 0. : cmd[0], v, 0., body_blocked ? 0. : cmd[3]), dt);
    if (gait_active && !body_blocked) {
      w.th += (guidance.state() == TagState::Blind ? yaw_drift_blind : yaw_drift_guided) * dt;
    }
    travel += dy;
    const std::array<bool, 6> eff = effectiveStance(in.gait_started, in.transfer_active, in.stance_mask);
    for (int leg = 0; leg < 6; ++leg) {
      if (eff[leg]) {
        if (!stance_feet_pinned) feet[leg].y() -= dy;
      } else {
        feet[leg].y() += 0.002;  // swing legs move forward regardless of the body
      }
    }
    if (nonzero || now < gait_end) {
      if (!gait_active) cycle = 0.;
      gait_active = true;
      cycle += dt;
    } else {
      gait_active = false;
    }
    group = static_cast<int>(cycle / 0.3) % 2;
    transfer = gait_active && std::fmod(cycle, 0.3) > 0.25;
  }
  bool run(double max_s = 150.) {
    const double t0 = now;
    while (now - t0 < max_s) {
      step();
      if (guidance.state() == TagState::Success) return true;
      if (guidance.state() == TagState::Failed) return false;
    }
    return false;
  }
  Eigen::Vector3d error() const {
    const Eigen::Vector3d s = w.seen();
    return Eigen::Vector3d(s[0] - config.ref_x_m, s[1] - zTarget(config),
                           wrapAngle(s[2] - config.ref_yaw_rad));
  }

  TagGuidanceConfig config;
  World w;
  TagGuidance guidance;
  double now = 100., dt = 1. / 30.;
  double invisible_z = 0.18;  // tag is returned only while z is above this
  double slip = 1., tau = 0.08, yaw_drift_guided = 0., yaw_drift_blind = 0., imu_bias = 0.3;
  double imu_sign = 1.;  // mounting: raw IMU yaw = imu_sign * body yaw
  bool imu_ok = true, body_blocked = false, stance_feet_pinned = false;
  Eigen::Vector4d cmd = Eigen::Vector4d::Zero();
  double v = 0., travel = 0., travel_seen = 0., travel_at_blind = 0.;
  bool blind_seen = false;
  bool gait_active = false, transfer = false;
  double gait_end = -1., cycle = 0.;
  int group = 0;
  std::array<Eigen::Vector3d, 6> feet;
};

TagGuidanceConfig simConfig() {
  TagGuidanceConfig c = testConfig();
  c.p0_z_m = 0.12;  // virtual depth at P0; z_target = 0.11; tag invisible below 0.18
  return c;
}

TEST(TagGuidanceSim, GuidedAdvanceThenBlindFinishReachesP0Overshoot) {
  const double offsets[][2] = {{0., 0.}, {0.05, 0.1396}, {-0.05, -0.1396}, {0.05, -0.1396},
                               {-0.05, 0.1396}};
  for (const auto& o : offsets) {
    const TagGuidanceConfig c = simConfig();
    Sim sim(c);
    sim.start(0.5, o[0], o[1]);
    SCOPED_TRACE(testing::Message() << "right " << o[0] << " dyaw " << o[1]);
    ASSERT_TRUE(sim.run()) << sim.guidance.reason();
    EXPECT_TRUE(sim.blind_seen);  // the camera never sees the tag near P0
    const TagHandover& h = sim.guidance.handover();
    EXPECT_TRUE(h.occurred);
    EXPECT_LT(h.remaining, c.max_blind_distance_m);
    EXPECT_GT(h.remaining, 0.);
    const Eigen::Vector3d e = sim.error();
    EXPECT_LT(std::abs(e[1]), 0.01);  // forward error vs P0 + overshoot
    EXPECT_LE(std::abs(e[0]), c.position_tolerance_m + 0.002);
    EXPECT_LE(std::abs(e[2]), c.yaw_tolerance_rad + 0.01);
    // Leg odometry from the synthetic stance feet equals the true travel.
    EXPECT_NEAR(sim.guidance.odometry(), sim.travel_seen - sim.travel_at_blind, 1e-9);
    EXPECT_NEAR(sim.guidance.odometry(), sim.guidance.commandIntegral(), 0.01);
  }
}

TEST(TagGuidanceSim, ForwardSlipStillStopsAtP0ByOdometryUntilMismatch) {
  const TagGuidanceConfig c = simConfig();
  {  // 20% less motion than commanded: odometry still stops at the right place
    Sim sim(c);
    sim.slip = 0.8;
    sim.start(0.5, 0.03, 0.05);
    ASSERT_TRUE(sim.run()) << sim.guidance.reason();
    EXPECT_LT(std::abs(sim.error()[1]), 0.01);
  }
  {  // 50% slip: command integral outruns the legs -> mismatch fail
    Sim sim(c);
    sim.slip = 0.5;
    sim.start(0.5, 0.03, 0.05);
    EXPECT_FALSE(sim.run());
    EXPECT_EQ(sim.guidance.reason(), "blind odometry mismatch (blocked or slipping legs?)");
  }
}

TEST(TagGuidanceSim, HeadingHoldRejectsBlindDriftAndLargeDriftFails) {
  const TagGuidanceConfig c = simConfig();
  {
    Sim sim(c);
    sim.yaw_drift_guided = 0.004;  // small walking drift on the tag-guided leg
    sim.yaw_drift_blind = 0.02;    // rad/s disturbance while blind
    sim.start(0.5, 0.0, 0.0);
    ASSERT_TRUE(sim.run()) << sim.guidance.reason();
    // Blind heading error settles at drift / gain = 0.025 rad (< 0.087), yaw
    // ends within handover tolerance plus that residual.
    EXPECT_LE(std::abs(sim.error()[2]), c.yaw_tolerance_rad + 0.03);
    EXPECT_LT(std::abs(sim.error()[1]), 0.01);
  }
  {
    Sim sim(c);
    sim.yaw_drift_blind = 0.4;  // beyond what the clamped heading hold can reject
    sim.start(0.5, 0.0, 0.0);
    EXPECT_FALSE(sim.run());
    EXPECT_EQ(sim.guidance.reason(), "blind heading error");
  }
}

TEST(TagGuidanceSim, BlindFailuresInClosedLoop) {
  const TagGuidanceConfig c = simConfig();
  {  // IMU never valid -> fails at handover
    Sim sim(c);
    sim.imu_ok = false;
    sim.start(0.5, 0., 0.);
    EXPECT_FALSE(sim.run());
    EXPECT_EQ(sim.guidance.reason(), "imu unavailable for blind finish");
  }
  {  // IMU drops out while blind
    Sim sim(c);
    sim.start(0.5, 0., 0.);
    while (sim.guidance.state() != TagState::Blind && sim.now < 300.) sim.step();
    ASSERT_EQ(sim.guidance.state(), TagState::Blind);
    for (int i = 0; i < 10; ++i) sim.step();
    sim.imu_ok = false;
    EXPECT_FALSE(sim.run());
    EXPECT_EQ(sim.guidance.reason(), "imu unavailable for blind finish");
  }
  {  // swing legs move but the stance feet (and the body) stay put
    Sim sim(c);
    sim.start(0.5, 0., 0.);
    while (sim.guidance.state() != TagState::Blind && sim.now < 300.) sim.step();
    ASSERT_EQ(sim.guidance.state(), TagState::Blind);
    sim.body_blocked = true;
    sim.stance_feet_pinned = true;
    EXPECT_FALSE(sim.run());
    EXPECT_EQ(sim.guidance.reason(), "blind odometry mismatch (blocked or slipping legs?)");
  }
  {  // blind timeout (slow blind speed)
    TagGuidanceConfig slow = c;
    slow.blind_timeout_s = 2.;
    Sim sim(slow);
    sim.start(0.5, 0., 0.);
    EXPECT_FALSE(sim.run());
    EXPECT_EQ(sim.guidance.reason(), "blind finish timeout");
  }
}

TEST(TagGuidanceSim, PauseDuringBlindThenCompletes) {
  const TagGuidanceConfig c = simConfig();
  Sim sim(c);
  sim.start(0.5, 0., 0.);
  while (sim.guidance.state() != TagState::Blind && sim.now < 300.) sim.step();
  for (int i = 0; i < 40; ++i) sim.step();
  ASSERT_EQ(sim.guidance.state(), TagState::Blind);
  const double odom = sim.guidance.odometry();
  const double integral = sim.guidance.commandIntegral();
  const double hold = sim.guidance.yawHold();
  sim.now += 8.;  // paused: no ticks, robot held
  sim.guidance.resume(sim.now);
  sim.step();
  EXPECT_EQ(sim.guidance.state(), TagState::Blind);
  EXPECT_NEAR(sim.guidance.odometry(), odom, 1e-3);  // at most one frame of motion
  EXPECT_NEAR(sim.guidance.commandIntegral(), integral, 1e-3);
  EXPECT_DOUBLE_EQ(sim.guidance.yawHold(), hold);
  ASSERT_TRUE(sim.run()) << sim.guidance.reason();
  EXPECT_LT(std::abs(sim.error()[1]), 0.01);
}

// ---------------------------------------------------------------------------
// Corrections: deadband vs gates, travel since last sample, one-sided success,
// IMU yaw sign.
// ---------------------------------------------------------------------------

TEST(TagGuidanceSim, HandoverSurvivesWalkingYawDriftAndParksInsideTolerance) {
  const double offsets[] = {0., 0.1396, -0.1396};
  for (double drift : {0., 0.005, 0.01, 0.02}) {
    for (double dyaw : offsets) {
      const TagGuidanceConfig c = simConfig();
      Sim sim(c);
      sim.yaw_drift_guided = drift;
      sim.yaw_drift_blind = drift;
      sim.start(0.5, 0.03, dyaw);
      SCOPED_TRACE(testing::Message() << "drift " << drift << " dyaw " << dyaw);
      ASSERT_TRUE(sim.run()) << sim.guidance.reason();
      const TagHandover& h = sim.guidance.handover();
      ASSERT_TRUE(h.occurred);
      if (drift == 0.) {
        EXPECT_LE(std::abs(h.eyaw), 0.6 * c.yaw_tolerance_rad);
        EXPECT_LE(std::abs(h.ex), 0.6 * c.position_tolerance_m);
      }
      EXPECT_LT(std::abs(sim.error()[1]), 0.01);
      EXPECT_LT(std::abs(sim.error()[0]), 0.02);
      // The tag-aligned hold removes the tag yaw residual and the drift during
      // the loss window: only the steady blind error drift/gain is left.
      const double yaw_limit = drift <= 0.01 ? c.yaw_tolerance_rad
                                             : c.yaw_tolerance_rad + drift / c.heading_gain;
      EXPECT_LE(std::abs(sim.error()[2]), yaw_limit);
    }
  }
}

TEST(TagGuidanceSim, FinalForwardErrorWithinThreeMillimetresWithoutSlip) {
  const double offsets[][2] = {{0., 0.}, {0.05, 0.1396}, {-0.05, -0.1396}, {0.05, -0.1396},
                               {-0.05, 0.1396}};
  for (const auto& o : offsets) {
    const TagGuidanceConfig c = simConfig();
    Sim sim(c);
    sim.start(0.5, o[0], o[1]);
    SCOPED_TRACE(testing::Message() << "right " << o[0] << " dyaw " << o[1]);
    ASSERT_TRUE(sim.run()) << sim.guidance.reason();
    const TagHandover& h = sim.guidance.handover();
    EXPECT_GT(h.travelled, 0.);  // the robot kept moving after the last sample
    EXPECT_NEAR(h.remaining, h.z_last - sim.guidance.zTarget() - h.travelled, 1e-12);
    EXPECT_LT(std::abs(sim.error()[1]), 0.003);
  }
}

TEST(TagGuidance, HandoverSubtractsTravelSinceLastValidSample) {
  const TagGuidanceConfig c = testConfig();
  Rig rig;
  rig.handoff(c.ref_x_m, 0.20, c.ref_yaw_rad);
  // The robot is advancing at 0.5 mm/frame (all-stance mask) through the sample
  // phase and keeps moving during the whole 0.3 s loss window.
  for (int i = 0; i < 5; ++i) {
    rig.shiftFeet(-0.0005);
    rig.step(c.ref_x_m, 0.20, c.ref_yaw_rad, false);
  }
  ASSERT_EQ(rig.guidance.state(), TagState::Advancing);
  int frames = 0;
  while (rig.guidance.state() == TagState::Advancing && frames < 30) {
    rig.shiftFeet(-0.0005);
    rig.tick(false);
    ++frames;
  }
  ASSERT_EQ(rig.guidance.state(), TagState::Blind);
  const TagHandover& h = rig.guidance.handover();
  EXPECT_GE(frames, 9);  // 0.3 s at 30 Hz
  // Odometry accumulated continuously since handoff (it is reset only at the
  // blind entry); travelled covers the frames after the last sample.
  EXPECT_NEAR(h.travelled, (frames + 1) * 0.0005, 0.0006);  // incl. the last sample frame
  EXPECT_GT(h.travelled, 0.004);
  EXPECT_NEAR(h.remaining, 0.20 - 0.15 - h.travelled, 1e-12);
  EXPECT_LT(h.remaining, 0.05 - 0.004);
  EXPECT_DOUBLE_EQ(rig.guidance.odometry(), 0.);  // BLIND odometry restarts
  EXPECT_DOUBLE_EQ(rig.guidance.commandIntegral(), 0.);
  // So much travel that nothing is left -> straight to settling.
  Rig done;
  done.handoff(c.ref_x_m, 0.17, c.ref_yaw_rad);
  for (int i = 0; i < 5; ++i) {
    done.shiftFeet(-0.0005);
    done.step(c.ref_x_m, 0.17, c.ref_yaw_rad, false);
  }
  int n = 0;
  while (done.guidance.state() == TagState::Advancing && n++ < 30) {
    done.shiftFeet(-0.003);  // fast: 9 cm/s
    done.tick(false);
  }
  EXPECT_EQ(done.guidance.state(), TagState::Settling);
  EXPECT_LE(done.guidance.handover().remaining, 0.);
}

TEST(TagGuidance, SuccessIsOneSidedInDepthCloserIsAccepted) {
  const TagGuidanceConfig c = testConfig();
  const double zt = zTarget(c);
  Rig rig;
  // Already 3 cm closer than z_target, aligned, tag visible, gait stopped.
  rig.handoff(c.ref_x_m, zt - 0.03, c.ref_yaw_rad);
  Eigen::Vector4d cmd = Eigen::Vector4d::Zero();
  const double t0 = rig.now + 1. / 30.;
  while (rig.guidance.state() != TagState::Success && rig.now - t0 < 2.) {
    cmd = rig.step(c.ref_x_m, zt - 0.03, c.ref_yaw_rad, true);
    EXPECT_EQ(cmd[1], 0.);  // never backward
  }
  EXPECT_EQ(rig.guidance.state(), TagState::Success);
  EXPECT_GE(rig.now - t0, 0.5 - 1e-9);
  // Farther than z_target + tol is still not success; lateral/yaw stay two-sided.
  Rig far;
  far.handoff(c.ref_x_m, zt + 0.02, c.ref_yaw_rad);
  for (int i = 0; i < 40; ++i) far.step(c.ref_x_m, zt + 0.02, c.ref_yaw_rad, true);
  EXPECT_EQ(far.guidance.state(), TagState::Advancing);
  Rig lateral;
  lateral.handoff(c.ref_x_m - 0.015, zt - 0.03, c.ref_yaw_rad);
  for (int i = 0; i < 40; ++i) lateral.step(c.ref_x_m - 0.015, zt - 0.03, c.ref_yaw_rad, true);
  EXPECT_NE(lateral.guidance.state(), TagState::Success);
}

TEST(TagGuidance, ImuYawSignAppliedToHoldAndError) {
  TagGuidanceConfig c = testConfig();
  c.imu_yaw_sign = -1.;
  Rig rig(c);
  rig.imu_yaw = -0.2;  // raw; effective yaw = +0.2
  advanceThenLose(rig, c.ref_x_m, 0.20, c.ref_yaw_rad);
  ASSERT_EQ(rig.guidance.state(), TagState::Blind);
  EXPECT_NEAR(rig.guidance.yawHold(), 0.2, 1e-12);
  // Body turned clockwise by 0.01: effective yaw 0.19, raw -0.19 -> CCW command.
  rig.imu_yaw = -0.19;
  const Eigen::Vector4d cmd = rig.tick(false);
  EXPECT_NEAR(rig.guidance.headingError(), 0.01, 1e-12);
  EXPECT_NEAR(cmd[3], c.heading_gain * 0.01, 1e-12);
}

TEST(TagGuidanceSim, HeadingHoldWorksWithNegatedImuAndSignParam) {
  TagGuidanceConfig c = simConfig();
  {  // mounting flipped, sign param compensates
    c.imu_yaw_sign = -1.;
    Sim sim(c);
    sim.imu_sign = -1.;
    sim.yaw_drift_blind = 0.02;
    sim.start(0.5, 0.0, 0.0);
    ASSERT_TRUE(sim.run()) << sim.guidance.reason();
    EXPECT_LT(std::abs(sim.guidance.headingError()), 0.03);
    EXPECT_LT(std::abs(sim.error()[1]), 0.01);
    EXPECT_LE(std::abs(sim.error()[2]), c.yaw_tolerance_rad + 0.03);
  }
  {  // mounting flipped but sign left at +1: positive feedback is caught
    c.imu_yaw_sign = 1.;
    Sim sim(c);
    sim.imu_sign = -1.;
    sim.yaw_drift_blind = 0.02;
    sim.start(0.5, 0.0, 0.0);
    EXPECT_FALSE(sim.run());
    EXPECT_EQ(sim.guidance.reason(), "blind heading error");
  }
}

TEST(TagGuidance, BlindHeadingTargetIsTagAlignedForBothImuSigns) {
  for (double sign : {1., -1.}) {
    TagGuidanceConfig c = testConfig();
    c.imu_yaw_sign = sign;
    const double eyaw_last = 0.02;  // tag says: still 0.02 rad short of the reference yaw
    Rig rig(c);
    rig.imu_yaw = sign * 0.5;  // raw; effective heading 0.5 at the last sample
    rig.handoff(c.ref_x_m, 0.20, c.ref_yaw_rad + eyaw_last);
    for (int i = 0; i < 5; ++i) rig.step(c.ref_x_m, 0.20, c.ref_yaw_rad + eyaw_last, false);
    ASSERT_EQ(rig.guidance.state(), TagState::Advancing);
    rig.imu_yaw = sign * 0.515;  // rotated CCW by 0.015 during the loss window
    while (rig.guidance.state() == TagState::Advancing) rig.tick(false);
    ASSERT_EQ(rig.guidance.state(), TagState::Blind);
    SCOPED_TRACE(testing::Message() << "sign " << sign);
    // Target = heading at the last sample + eyaw_last (CCW for positive eyaw).
    EXPECT_NEAR(rig.guidance.yawHold(), 0.5 + eyaw_last, 1e-12);
    EXPECT_NEAR(rig.guidance.handover().heading_target_offset, 0.5 + eyaw_last - 0.515, 1e-12);
    EXPECT_NEAR(rig.guidance.handover().eyaw, eyaw_last, 1e-12);
    // 0.005 rad CCW still to go -> positive (CCW) yaw command.
    const Eigen::Vector4d cmd = rig.tick(false);
    EXPECT_NEAR(rig.guidance.headingError(), 0.005, 1e-12);
    EXPECT_NEAR(cmd[3], c.heading_gain * 0.005, 1e-12);
    // Negative eyaw_last gives a target on the other side.
    Rig neg(c);
    neg.imu_yaw = sign * 0.5;
    neg.handoff(c.ref_x_m, 0.20, c.ref_yaw_rad - 0.02);
    for (int i = 0; i < 5; ++i) neg.step(c.ref_x_m, 0.20, c.ref_yaw_rad - 0.02, false);
    while (neg.guidance.state() == TagState::Advancing) neg.tick(false);
    ASSERT_EQ(neg.guidance.state(), TagState::Blind);
    EXPECT_NEAR(neg.guidance.yawHold(), 0.5 - 0.02, 1e-12);
  }
}

TEST(TagGuidance, BlindHeadingTargetFallsBackToCurrentImuWhenSampleHadNone) {
  const TagGuidanceConfig c = testConfig();
  Rig rig;
  rig.imu_valid = false;  // no IMU while the tag was being tracked
  rig.handoff(c.ref_x_m, 0.20, c.ref_yaw_rad + 0.02);
  for (int i = 0; i < 5; ++i) rig.step(c.ref_x_m, 0.20, c.ref_yaw_rad + 0.02, false);
  rig.imu_valid = true;
  rig.imu_yaw = 0.9;
  while (rig.guidance.state() == TagState::Advancing) rig.tick(false);
  ASSERT_EQ(rig.guidance.state(), TagState::Blind);
  EXPECT_NEAR(rig.guidance.yawHold(), 0.9 + 0.02, 1e-12);
  EXPECT_NEAR(rig.guidance.handover().heading_target_offset, 0.02, 1e-12);
}

TEST(TagGuidance, ImuYawSignValidation) {
  for (double bad : {0., 2., 0.5, -2., std::numeric_limits<double>::quiet_NaN()}) {
    TagGuidanceConfig c;
    c.imu_yaw_sign = bad;
    EXPECT_THROW(TagGuidance{c}, std::invalid_argument) << bad;
  }
  for (double good : {1., -1.}) {
    TagGuidanceConfig c;
    c.imu_yaw_sign = good;
    EXPECT_NO_THROW(TagGuidance{c});
  }
}


TEST(TagGuidance, ConfigValidation) {
  TagGuidanceConfig c;
  c.confirm_frames = 0;
  EXPECT_THROW(TagGuidance{c}, std::invalid_argument);
  c = TagGuidanceConfig();
  c.max_yaw_rate = 0.;
  EXPECT_THROW(TagGuidance{c}, std::invalid_argument);
  c = TagGuidanceConfig();
  c.p0_z_m = 0.;
  EXPECT_THROW(TagGuidance{c}, std::invalid_argument);
  c = TagGuidanceConfig();
  c.final_overshoot_m = c.p0_z_m;  // target depth would be 0
  EXPECT_THROW(TagGuidance{c}, std::invalid_argument);
  c = TagGuidanceConfig();
  c.final_overshoot_m = -0.01;
  EXPECT_THROW(TagGuidance{c}, std::invalid_argument);
  c = TagGuidanceConfig();
  c.blind_speed_m_s = 0.;
  EXPECT_THROW(TagGuidance{c}, std::invalid_argument);
  c = TagGuidanceConfig();
  c.min_reliable_z_m = -1.;
  EXPECT_THROW(TagGuidance{c}, std::invalid_argument);
  EXPECT_NO_THROW(TagGuidance{TagGuidanceConfig()});
}

}  // namespace
}  // namespace grasp_hexapod_bt_control
