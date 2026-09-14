#include "grasp_hexapod_control_cpp/ring_detection.h"
#include "grasp_hexapod_control_cpp/ring_dock_mode.h"
#include "grasp_hexapod_control_cpp/ring_controller.h"
#include "grasp_hexapod_control_cpp/config_io.h"
#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
namespace r=grasp_hexapod_ring;
namespace g=grasp_hexapod_control_cpp;
using nlohmann::json;
json fixtures() {std::ifstream input(RING_FIXTURE_JSON);return json::parse(input);}
g::FootPositions feetFromJson(const json& a) {
  g::FootPositions p{};for(int i=0;i<6;++i)for(int j=0;j<3;++j)p[i][j]=a[i][j];return p;
}
TEST(RingNative, DetectionMatchesPythonFixtures) {
  const auto data=fixtures();
  for(const auto& item:data["images"]) {
    const auto image=cv::imread(std::string(RING_FIXTURE_DIR)+"/"+item["file"].get<std::string>(),0);
    ASSERT_FALSE(image.empty());auto result=r::detectRings(image);
    ASSERT_EQ(result.valid,item["valid"].get<bool>())<<item["file"];
    if(result.valid) {
      EXPECT_NEAR(result.center.x,item["center"][0].get<double>(),.05)<<item["file"];
      EXPECT_NEAR(result.center.y,item["center"][1].get<double>(),.05)<<item["file"];
      EXPECT_NEAR(result.diameter,item["diameter"].get<double>(),.1)<<item["file"];
      EXPECT_EQ(result.matched,item["matched"].get<int>())<<item["file"];
    }
  }
}
TEST(RingNative, StateMachineMatchesPythonFrames) {
  const auto data=fixtures();
  for(const auto& scenario:data["scenarios"]) {
    auto feet=feetFromJson(scenario["initial_feet"]);
    r::DockMode mode(.1,[&](const g::JointAngles&,bool){return feet;},
      [](){r::PerceptionResult result;result.reason="lost ring";return result;},[](){},
      scenario["confirmation"].get<bool>(),.05,30,10,.05,.5,.04,.003,{});
    g::JointAngles q{};for(auto& leg:q)leg.setZero();mode.enter(q,nullptr);
    for(const auto& frame:scenario["frames"]) {
      r::DockRobotState state;state.joints=q;
      if(!frame["pose"].is_null()) {
        g::Matrix4d pose=g::Matrix4d::Identity();
        for(int i=0;i<3;++i)pose(i,3)=frame["pose"][i];state.lock_from_pin=pose;
      }
      state.lock_confirmed=frame["confirmed"].get<bool>();
      const auto result=mode.update(state);
      ASSERT_EQ(result.state,frame["state"].get<std::string>()) << result.reason << " progress=" << mode.leg_lift_progress_;
      ASSERT_EQ(bool(result.foot_positions_base),!frame["feet"].is_null());
      ASSERT_EQ(bool(result.joint_positions),!frame["joints"].is_null());
      EXPECT_EQ(result.request_lock,frame["request_lock"].get<bool>());
      if(result.foot_positions_base) {
        const auto expected=feetFromJson(frame["feet"]);
        for(int i=0;i<6;++i)EXPECT_LT(((*result.foot_positions_base)[i]-expected[i]).norm(),1e-10);
        feet=*result.foot_positions_base;
      }
    }
    EXPECT_EQ(mode.state,r::DockMode::kSuccess);
    EXPECT_TRUE(mode.update({}).success);
  }
}
TEST(RingNative, StalledLegLiftCannotReportSuccess) {
  g::FootPositions feet{};for(auto& f:feet)f.setZero();
  r::DockMode mode(.1,[&](const g::JointAngles&,bool){return feet;},[](){return r::PerceptionResult{};},[](){},false,.05,30,10,.05,.5,.04,.003,{});
  mode.active=true;mode.state=r::DockMode::kLegLift;
  r::DockRobotState state;state.joints=feet;
  for(int i=0;i<100;++i)mode.update(state);
  EXPECT_EQ(mode.state,r::DockMode::kLegLift);
  mode.failExecution("test failure");EXPECT_TRUE(mode.update({}).failed);
}
TEST(RingNative, ControllerUsesCurrentPoseAndSharedDls) {
  g::ControlPaths paths{std::string(RING_CONFIG_DIR)+"/climb_compact.json",std::string(RING_CONFIG_DIR)+"/workspace_bounds.csv"};
  r::RingController controller(1./30.,false,false,paths);
  r::DockMode mode(controller.dt,[&](const g::JointAngles& q,bool sync){
    return sync?controller.syncActualFeet(q):controller.kinematic.forwardBase(q);
  },[](){return r::PerceptionResult{};},[](){},false,.05,30,10,.05,.5,.04,.003,{});
  controller.attachDockMode(&mode);auto q=controller.q_init;
  auto initial=controller.kinematic.forwardBase(q);controller.enterDock(q);
  for(int i=0;i<120 && mode.state!=r::DockMode::kWaitingTag;++i) {
    r::DockRobotState state;state.joints=q;q=controller.update(q,Eigen::Vector4d::Zero(),nullptr,&state);
  }
  ASSERT_EQ(mode.state,r::DockMode::kWaitingTag);
  auto actual=controller.kinematic.forwardBase(q);
  for(int i=0;i<6;++i) {
    EXPECT_NEAR(actual[i].x(),initial[i].x(),.0001);
    EXPECT_NEAR(actual[i].y(),initial[i].y(),.0001);
    EXPECT_NEAR(actual[i].z(),initial[i].z()-.04,.001);
  }
  controller.resetToStand(q);EXPECT_FALSE(mode.active);
}

TEST(RingNative, SlowerAlignmentPreservesVerticalRaiseSpeed) {
  g::FootPositions feet{}; for(auto& f:feet) f.setZero();
  r::DockMode mode(.1,[&](const g::JointAngles&,bool){return feet;},
    [](){return r::PerceptionResult{};},[](){},
    false,.05,30,10,.05,.5,.04,.003,{},.02);
  r::DockRobotState state; state.joints=feet;
  mode.active=true; mode.state=r::DockMode::kWaitingTag;
  g::Matrix4d pose=g::Matrix4d::Identity();
  pose(0,3)=.03; pose(1,3)=.04;
  state.lock_from_pin=pose;
  const auto aligned=mode.update(state);
  ASSERT_EQ(aligned.state,r::DockMode::kPrealign);
  ASSERT_TRUE(aligned.foot_positions_base);
  for(const auto& foot:*aligned.foot_positions_base) {
    EXPECT_NEAR(foot.x(),-.0012,1e-12);
    EXPECT_NEAR(foot.y(),-.0016,1e-12);
    EXPECT_NEAR(foot.z(),0.,1e-12);
  }
  mode.state=r::DockMode::kBodyRaise;
  const auto raised=mode.update(state);
  ASSERT_TRUE(raised.foot_positions_base);
  for(const auto& foot:*raised.foot_positions_base)
    EXPECT_NEAR(foot.z(),-.005,1e-12);
}
