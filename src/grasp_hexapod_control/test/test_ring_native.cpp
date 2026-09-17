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
namespace {
struct DockHarness {
  g::ControlPaths paths{std::string(RING_CONFIG_DIR)+"/climb_compact.json",
                       std::string(RING_CONFIG_DIR)+"/workspace_bounds.csv"};
  r::RingController controller{1./30.,false,false,paths};
  double support_feedback_offset = 0.;
  r::DockMode mode;
  g::JointAngles q;
  DockHarness(bool lock=false):mode(controller.dt,[this](const g::JointAngles& q,bool sync){
      auto feet=sync?controller.syncActualFeet(q):controller.kinematic.forwardBase(q);
      if(!sync)for(int leg:{0,1,3,4})feet[leg].x()+=support_feedback_offset;
      return feet;
    },[](){return r::PerceptionResult{};},[](){},lock,.05,30,10,.05,.5,.04,.003,{}) {
    q=controller.q_init;controller.attachDockMode(&mode);controller.enterDock(q);
    mode.descent_distance_=.015;
  }
  g::JointAngles tick(bool follow=true,std::optional<g::Matrix4d> pose=std::nullopt,bool locked=false) {
    r::DockRobotState state;state.joints=q;state.lock_from_pin=pose;state.lock_confirmed=locked;
    auto cmd=controller.update(q,Eigen::Vector4d::Zero(),nullptr,&state);
    if(follow) q=cmd;return cmd;
  }
  bool until(const std::string& state,int budget=1500) {
    for(int i=0;i<budget&&!r::DockMode::isTerminalState(mode.state)&&mode.state!=state;++i)tick();
    return mode.state==state;
  }
};
}

TEST(RingNative, MiddleFirstThenFourLegBodyRaise) {
  DockHarness h;const auto start=h.q;const auto initial=h.controller.kinematic.forwardBase(start);
  ASSERT_TRUE(h.until(r::DockMode::kMiddleLift))<<h.mode.reason;
  bool ankle_started=false;int knee_only_ticks=0,ankle_ticks=0,ticks=0;
  double peak_speed=0.;
  for(int n=0;n<1000&&h.mode.state==r::DockMode::kMiddleLift;++n) {
    auto previous=h.q;h.tick();++ticks;
    double knee_delta=0.,ankle_delta=0.;
    for(int leg:{2,5}) {
      knee_delta=std::max(knee_delta,std::abs(h.q[leg][1]-previous[leg][1]));
      ankle_delta=std::max(ankle_delta,std::abs(h.q[leg][2]-previous[leg][2]));
      EXPECT_LE(std::abs(h.q[leg][1]),100.*M_PI/180.+1e-9);
    }
    if(ankle_started)EXPECT_LT(knee_delta,1e-9);
    if(knee_delta>1e-9 && ankle_delta<1e-9)++knee_only_ticks;
    if(ankle_delta>1e-9){ankle_started=true;++ankle_ticks;}
    for(int leg=0;leg<6;++leg) {
      if(!r::DockMode::isMiddleLeg(leg))EXPECT_LT((h.q[leg]-start[leg]).norm(),1e-12);
      const double speed=(h.q[leg]-previous[leg]).cwiseAbs().maxCoeff()/h.controller.dt*180./M_PI;
      peak_speed=std::max(peak_speed,speed);
      EXPECT_LE(speed,r::DockMode::isMiddleLeg(leg)?30.+1e-6:20.+1e-6);
    }
  }
  EXPECT_GT(knee_only_ticks,0);EXPECT_GT(ankle_ticks,0);
  EXPECT_GT(peak_speed,20.); // 验证实际提速，不只是修改未生效的上限。
  EXPECT_LT(ticks*h.controller.dt,6.); // 标准站姿原中腿轨迹约7.36秒，另有到位时间。
  std::cout<<"Middle motion seconds="<<ticks*h.controller.dt<<" peak deg/s="<<peak_speed<<std::endl;
  ASSERT_EQ(h.mode.state,r::DockMode::kBodyRaise)<<h.mode.reason;
  ASSERT_TRUE(h.mode.middleHold());const auto locked=*h.mode.middleHold();
  auto middle_feet=h.controller.kinematic.forwardBase(h.q);
  for(int leg:{2,5}) {
    EXPECT_NEAR(middle_feet[leg].z()-initial[leg].z(),.1371,.001);
    EXPECT_NEAR(std::abs(h.q[leg][1])*180./M_PI,98.,.01);
    EXPECT_NEAR(std::abs(h.q[leg][2])*180./M_PI,75.,.1);
  }
  ASSERT_TRUE(h.until(r::DockMode::kSearchingTag))<<h.mode.reason;
  const auto feet=h.controller.kinematic.forwardBase(h.q);
  for(int leg=0;leg<6;++leg) {
    if(r::DockMode::isMiddleLeg(leg))EXPECT_LT((h.q[leg]-locked[leg]).norm(),1e-12);
    else EXPECT_NEAR(feet[leg].z()-initial[leg].z(),-.040,.003);
  }
}

TEST(RingNative, CircleSearchKeepsSupportContactsAndMiddlePose) {
  DockHarness h;ASSERT_TRUE(h.until(r::DockMode::kSearchingTag));
  const auto anchor=*h.mode.search_anchor_feet_;
  const auto locked=*h.mode.middleHold();
  Eigen::Vector2d previous=Eigen::Vector2d::Zero();
  double max_error=0.;int circle_ticks=0;
  // 前移至30mm，随后绕行超过一圈，覆盖全部支撑姿态。
  for(int n=0;n<350;++n) {
    h.tick();ASSERT_EQ(h.mode.state,r::DockMode::kSearchingTag)<<h.mode.reason;
    const auto offset=h.mode.search_body_offset_;
    EXPECT_LE((offset-previous).norm(),.020*h.controller.dt+1e-10);
    EXPECT_LE(offset.norm(),.030+1e-10);
    if(h.mode.search_on_circle_) {
      EXPECT_NEAR(offset.norm(),.030,1e-10);++circle_ticks;
    } else {
      EXPECT_NEAR(offset.x(),0.,1e-10);EXPECT_GT(offset.y(),previous.y());
    }
    const auto feet=h.controller.kinematic.forwardBase(h.q);
    for(int leg=0;leg<6;++leg) {
      if(r::DockMode::isMiddleLeg(leg)) {
        EXPECT_LT((h.q[leg]-locked[leg]).norm(),1e-12);
      } else {
        const g::Vector3d shift(offset.x(),offset.y(),0.);
        EXPECT_LT(((*h.mode.last_visual_target)[leg]+shift-anchor[leg]).norm(),1e-10);
        max_error=std::max(max_error,(feet[leg]+shift-anchor[leg]).norm());
      }
    }
    previous=offset;
  }
  EXPECT_GT(circle_ticks,2.*M_PI*.030/.020/h.controller.dt);
  EXPECT_LT(max_error,.003);
  std::cout<<"Circle search maximum support error (mm)="<<max_error*1000.<<std::endl;
}

TEST(RingNative, CircleSearchTransitionsToAlignmentAndResetsOnReentry) {
  DockHarness h;ASSERT_TRUE(h.until(r::DockMode::kSearchingTag));
  for(int n=0;n<80;++n)h.tick();
  const auto before=*h.mode.last_visual_target;
  const auto offset=h.mode.search_body_offset_;
  auto pose=g::Matrix4d::Identity().eval();pose(0,3)=.030;
  h.tick(true,pose);ASSERT_EQ(h.mode.state,r::DockMode::kPrealign)<<h.mode.reason;
  for(int leg:{0,1,3,4})
    EXPECT_LE(((*h.mode.last_visual_target)[leg]-before[leg]).norm(),.050/30.+1e-10);
  for(int n=0;n<3;++n)h.tick(); // 覆盖一次感知刷新；丢失目标沿用原有保持逻辑。
  EXPECT_EQ(h.mode.state,r::DockMode::kWaitingTag);
  EXPECT_LT((h.mode.search_body_offset_-offset).norm(),1e-12);
  h.controller.enterDock(h.q);
  EXPECT_FALSE(h.mode.search_anchor_feet_);
  EXPECT_EQ(h.mode.search_body_offset_.norm(),0.);
  EXPECT_FALSE(h.mode.search_on_circle_);
}

TEST(RingNative, SupportFootOffsetDoesNotGateDockStages) {
  DockHarness h;ASSERT_TRUE(h.until(r::DockMode::kBodyRaise));
  // 模拟超过旧3mm阈值的持续足端反馈偏差，关节仍正常跟踪。
  h.support_feedback_offset=.0037;
  ASSERT_TRUE(h.until(r::DockMode::kSearchingTag))<<h.mode.reason;
  for(int n=0;n<40;++n)h.tick();
  ASSERT_EQ(h.mode.state,r::DockMode::kSearchingTag)<<h.mode.reason;
  auto pose=g::Matrix4d::Identity().eval();pose(0,3)=.030;
  for(int n=0;n<35;++n)h.tick(true,pose);
  ASSERT_EQ(h.mode.state,r::DockMode::kPrealign)<<h.mode.reason;
  pose(0,3)=0.;
  for(int n=0;n<1000&&!r::DockMode::isTerminalState(h.mode.state);++n)h.tick(true,pose);
  EXPECT_EQ(h.mode.state,r::DockMode::kSuccess)<<h.mode.reason;
}

TEST(RingNative, FullDockPreservesMiddleThroughSupportMotion) {
  for(bool lock:{false,true}) {
    DockHarness h(lock);ASSERT_TRUE(h.until(r::DockMode::kSearchingTag))<<h.mode.reason;
    const auto locked=*h.mode.middleHold();const auto before=h.controller.kinematic.forwardBase(h.q);
    auto pose=g::Matrix4d::Identity().eval();pose(0,3)=.010;
    for(int n=0;n<5;++n){h.tick(true,pose);for(int leg:{2,5})EXPECT_LT((h.q[leg]-locked[leg]).norm(),1e-12);}
    // 模拟一帧目标丢失，中腿仍保持同一指令。
    h.tick();for(int leg:{2,5})EXPECT_LT((h.q[leg]-locked[leg]).norm(),1e-12);
    pose(0,3)=0.;bool descent_seen=false;
    for(int n=0;n<1500&&!r::DockMode::isTerminalState(h.mode.state)&&h.mode.state!=r::DockMode::kAligned;++n) {
      h.tick(true,pose);descent_seen|=h.mode.state==r::DockMode::kDescent;
      for(int leg=0;leg<6;++leg) {
        EXPECT_LE(std::abs(h.q[leg][1]),100.*M_PI/180.+1e-9);
        if(r::DockMode::isMiddleLeg(leg))EXPECT_LT((h.q[leg]-locked[leg]).norm(),1e-12);
      }
    }
    ASSERT_TRUE(descent_seen);ASSERT_EQ(h.mode.state,lock?r::DockMode::kAligned:r::DockMode::kSuccess)<<h.mode.reason;
    if(lock){h.tick(true,pose,false);EXPECT_EQ(h.mode.state,r::DockMode::kAligned);h.tick(true,pose,true);}
    ASSERT_EQ(h.mode.state,r::DockMode::kSuccess)<<h.mode.reason;
    auto feet=h.controller.kinematic.forwardBase(h.q);
    for(int leg:{2,5})EXPECT_NEAR(feet[leg].z(),before[leg].z(),1e-9);
    for(int leg:{0,1,3,4})EXPECT_GT(feet[leg].z(),before[leg].z());
    h.tick();for(int leg:{2,5})EXPECT_LT((h.q[leg]-locked[leg]).norm(),1e-12);
  }
}

TEST(RingNative, LargeDescentIsNotRejectedByMiddleClearance) {
  DockHarness h;ASSERT_TRUE(h.until(r::DockMode::kSearchingTag));h.mode.descent_distance_=.180;
  const auto locked=*h.mode.middleHold();auto pose=g::Matrix4d::Identity().eval();
  h.tick(true,pose);
  ASSERT_EQ(h.mode.state,r::DockMode::kPreDescentSettle)<<h.mode.reason;
  for(int n=0;n<30&&h.mode.state!=r::DockMode::kDescent;++n)h.tick(true,pose);
  ASSERT_EQ(h.mode.state,r::DockMode::kDescent)<<h.mode.reason;
  for(int leg:{2,5})EXPECT_LT((h.q[leg]-locked[leg]).norm(),1e-12);
}

TEST(RingNative, MiddleCompletionDoesNotRequireSupportFootArrival) {
  DockHarness h;ASSERT_TRUE(h.until(r::DockMode::kMiddleLift));h.tick();
  // 规划完成后注入支撑足端偏差，不能阻止中腿完成本阶段。
  h.support_feedback_offset=.010;
  ASSERT_TRUE(h.until(r::DockMode::kBodyRaise))<<h.mode.reason;
  EXPECT_TRUE(h.mode.middleHold());
}

TEST(RingNative, MiddleLagDoesNotTriggerRemovedStallProtection) {
  DockHarness h;ASSERT_TRUE(h.until(r::DockMode::kMiddleLift));
  for(int i=0;i<60;++i)h.tick(false); // 静止反馈持续2秒，超过原0.8秒门限。
  ASSERT_EQ(h.mode.state,r::DockMode::kMiddleLift)<<h.mode.reason;
  EXPECT_FALSE(h.mode.middleHold());
  EXPECT_TRUE(h.until(r::DockMode::kSearchingTag))<<h.mode.reason;
}

TEST(RingNative, LockedMiddleOffsetDoesNotFailWhileWaitingForVision) {
  DockHarness h;ASSERT_TRUE(h.until(r::DockMode::kSearchingTag));const auto locked=*h.mode.middleHold();
  h.q[2][1]+=4.*M_PI/180.;
  for(int n=0;n<100;++n) {
    const auto cmd=h.tick(false);
    ASSERT_EQ(h.mode.state,r::DockMode::kSearchingTag)<<h.mode.reason;
    EXPECT_LT((cmd[2]-locked[2]).norm(),1e-12);
  }
}

TEST(RingNative, DescentContinuesWithFrozenFeedbackAndCompletesAfterRecovery) {
  DockHarness h;ASSERT_TRUE(h.until(r::DockMode::kSearchingTag));
  auto pose=g::Matrix4d::Identity().eval();
  for(int n=0;n<100&&h.mode.state!=r::DockMode::kDescent;++n)h.tick(true,pose);
  ASSERT_EQ(h.mode.state,r::DockMode::kDescent)<<h.mode.reason;
  const auto frozen=h.q[1];double previous=h.mode.descent_remaining_;int advanced=0;
  auto command=h.q;
  for(int n=0;n<400;++n) {
    command=h.tick(false,pose);
    ASSERT_NE(h.mode.state,r::DockMode::kFailed)<<h.mode.reason;
    EXPECT_LE(h.mode.descent_remaining_,previous+1e-12);
    if(h.mode.descent_remaining_<previous-1e-12)++advanced;
    previous=h.mode.descent_remaining_;
    h.q=command;h.q[1]=frozen;
  }
  EXPECT_GT(advanced,3);
  EXPECT_EQ(h.mode.state,r::DockMode::kLegLift); // 原有最终到位判断仍生效。
  EXPECT_NEAR(command[1][1],-98.*M_PI/180.,1e-9);
  EXPECT_NEAR(command[1][2],75.*M_PI/180.,1e-9);
  h.q=command;
  EXPECT_TRUE(h.until(r::DockMode::kSuccess))<<h.mode.reason;
}

TEST(RingNative, CompletePlanIsReadyBeforeDescentAndTargetsStayContinuous) {
  for(double distance:{.015,.100,.180}) {
    DockHarness h;ASSERT_TRUE(h.until(r::DockMode::kSearchingTag));
    h.mode.descent_distance_=distance;
    const auto locked=*h.mode.middleHold();const auto command=h.q;
    // 模拟反馈向踝角边界方向偏移，规划入口不能把偏差变成目标跳变。
    for(int leg:{0,1,3,4})h.q[leg][2]+=std::copysign(1.5*M_PI/180.,h.q[leg][2]);
    auto pose=g::Matrix4d::Identity().eval();
    const auto first=h.tick(true,pose);
    ASSERT_EQ(h.mode.state,r::DockMode::kPreDescentSettle)<<h.mode.reason;
    EXPECT_GT(h.mode.descent_duration_,0.);
    for(int leg=0;leg<6;++leg)EXPECT_LT((first[leg]-command[leg]).norm(),1e-12);
    const auto initial=*h.mode.pre_descent_feet_;const double planned_time=h.mode.descent_duration_;
    bool descent=false,fold=false;
    for(int n=0;n<2000&&!r::DockMode::isTerminalState(h.mode.state);++n) {
      const auto previous=h.q;h.tick(); // 规划后不再依赖视觉输入。
      descent|=h.mode.state==r::DockMode::kDescent;
      fold|=h.mode.state==r::DockMode::kLegLift;
      const auto feet=h.controller.kinematic.forwardBase(h.q);
      for(int leg=0;leg<6;++leg) {
        EXPECT_LE((h.q[leg]-previous[leg]).cwiseAbs().maxCoeff(),20.*M_PI/180./30.+1e-8);
        EXPECT_LE(std::abs(h.q[leg][1]),100.*M_PI/180.+1e-9);
        EXPECT_LE(std::abs(h.q[leg][2]),118.*M_PI/180.+1e-9);
        if(r::DockMode::isMiddleLeg(leg))EXPECT_LT((h.q[leg]-locked[leg]).norm(),1e-12);
        else if(!fold)EXPECT_LE((feet[leg]-initial[leg]).head<2>().norm(),.030+1e-6);
      }
      EXPECT_DOUBLE_EQ(h.mode.descent_duration_,planned_time);
    }
    EXPECT_TRUE(fold);if(distance<.05)EXPECT_TRUE(descent);
    EXPECT_EQ(h.mode.state,r::DockMode::kSuccess)<<h.mode.reason;
    std::cout<<"Preplanned requested="<<distance*1000.<<"mm controlled="
      <<(distance-h.mode.descent_remaining_)*1000.<<"mm duration="<<planned_time<<"s ankles=";
    for(int leg:{0,1,3,4})std::cout<<h.q[leg][2]*180./M_PI<<" ";std::cout<<std::endl;
  }
}

TEST(RingNative, RecordedSupportEntryHasCompletePlan) {
  DockHarness h;ASSERT_TRUE(h.until(r::DockMode::kSearchingTag));
  // 9月14日记录按腿名映射，避免数组顺序(lb,lf,lm,rb,rf,rm)混淆。
  const char* names[]={"lf","rf","lb","rb"};
  const double values[4][3]={{-3.12,-8.16,79.92},{5.76,9.36,-77.52},
                           {-6.,-12.96,72.48},{2.16,11.04,-70.8}};
  for(int i=0;i<4;++i)for(int j=0;j<3;++j)h.q[g::legIndexFromName(names[i])][j]=values[i][j]*M_PI/180.;
  h.mode.acceptCommand(h.q,h.q);h.mode.descent_distance_=.100;
  auto pose=g::Matrix4d::Identity().eval();h.tick(true,pose);
  ASSERT_EQ(h.mode.state,r::DockMode::kPreDescentSettle)<<h.mode.reason;
  for(int n=0;n<2000&&!r::DockMode::isTerminalState(h.mode.state);++n)h.tick();
  EXPECT_EQ(h.mode.state,r::DockMode::kSuccess)<<h.mode.reason;
}

TEST(RingNative, RecordedSeptember15PreservesDescentBeforeFolding) {
  // 两轮下降前的指令角；数组顺序为lb/lf/lm/rb/rf/rm。
  const double entries[2][6][3]={
    {{-6.55685414,-11.0555264,62.03926966},{-15.84987341,-3.93676312,80.32566212},
     {-.24,-44.38864985,75.},{8.24279986,11.34445224,-62.96655307},
     {10.16784129,5.82987039,-81.43511297},{-.24,44.3858416,-75.}},
    {{-8.34059146,-11.10675158,63.31222047},{-12.98151379,-1.53591276,81.53410021},
     {-.24,-43.88850714,75.},{6.73751773,11.0948737,-62.23246299},
     {12.39203706,5.46158818,-80.29210627},{0.,43.66322232,-75.}}};
  for(int round=0;round<2;++round) {
    DockHarness h;ASSERT_TRUE(h.until(r::DockMode::kSearchingTag));
    // 中腿沿用已锁定姿态；支撑腿使用实测指令作为规划入口。
    for(int leg:{0,1,3,4})for(int j=0;j<3;++j)h.q[leg][j]=entries[round][leg][j]*M_PI/180.;
    const auto locked=*h.mode.middleHold();
    h.mode.acceptCommand(h.q,h.q);
    h.mode.descent_distance_=round==0?.1715:.1718;
    h.tick(true,g::Matrix4d::Identity().eval());
    ASSERT_EQ(h.mode.state,r::DockMode::kPreDescentSettle)<<h.mode.reason;
    const auto initial=*h.mode.pre_descent_feet_;
    bool descending=false,folding=false,ankle_started=false;
    int knee_only_ticks=0, ankle_ticks=0;
    for(int n=0;n<2000&&!r::DockMode::isTerminalState(h.mode.state);++n) {
      const auto previous=h.q;const auto previous_state=h.mode.state;h.tick();
      if(previous_state==r::DockMode::kLegLift && h.mode.state==r::DockMode::kLegLift) {
        double knee_delta=0.,ankle_delta=0.;
        for(int leg:{0,1,3,4}) {
          knee_delta=std::max(knee_delta,std::abs(h.q[leg][1]-previous[leg][1]));
          ankle_delta=std::max(ankle_delta,std::abs(h.q[leg][2]-previous[leg][2]));
        }
        if(ankle_started)EXPECT_LT(knee_delta,1e-9);
        if(knee_delta>1e-9 && ankle_delta<1e-9)++knee_only_ticks;
        if(ankle_delta>1e-9){ankle_started=true;++ankle_ticks;}
      }
      if(h.mode.state==r::DockMode::kDescent){EXPECT_FALSE(folding);descending=true;}
      if(h.mode.state==r::DockMode::kLegLift){EXPECT_TRUE(descending);folding=true;}
      const auto feet=h.controller.kinematic.forwardBase(h.q);
      for(int leg=0;leg<6;++leg) {
        EXPECT_LE((h.q[leg]-previous[leg]).cwiseAbs().maxCoeff(),20.*M_PI/180./30.+1e-8);
        EXPECT_LE(std::abs(h.q[leg][1]),98.*M_PI/180.+1e-9);
        if(r::DockMode::isMiddleLeg(leg))EXPECT_LT((h.q[leg]-locked[leg]).norm(),1e-12);
        else if(!folding)EXPECT_LE((feet[leg]-initial[leg]).head<2>().norm(),.030+1e-6);
      }
    }
    ASSERT_EQ(h.mode.state,r::DockMode::kSuccess)<<h.mode.reason;
    EXPECT_TRUE(folding);
    EXPECT_GT(knee_only_ticks,0);
    EXPECT_GT(ankle_ticks,0);
    const auto feet=h.controller.kinematic.forwardBase(h.q);
    const double descent=h.mode.descent_distance_.value()-h.mode.descent_remaining_;
    // 下降优先：不能为了踝75deg/高抬腿，再退回混合评分选出的20~22mm。
    EXPECT_GE(descent, (round==0?.058:.052)-1e-8);
    std::cout<<"September15 round="<<round+1<<" descent="<<descent*1000.<<"mm";
    for(int leg:{0,1,3,4}) {
      const double lift=feet[leg].z()-initial[leg].z()-descent;
      EXPECT_GE(lift,.005-1e-8);
      EXPECT_NEAR(std::abs(h.q[leg][1])*180./M_PI,98.,.01);
      EXPECT_NEAR(std::abs(h.q[leg][2])*180./M_PI,75.,.01);
      std::cout<<" "<<g::kLegNames[leg]<<":ankle="<<h.q[leg][2]*180./M_PI<<",lift="<<lift*1000.;
    }
    std::cout<<std::endl;
  }
}

TEST(RingNative, NearBoundaryFeedbackDoesNotBecomeANewCommand) {
  DockHarness h;ASSERT_TRUE(h.until(r::DockMode::kSearchingTag));
  const char* names[]={"lf","rf","lb","rb"};
  const double values[4][3]={{-6.24,-70.35,115.},{4.56,70.59,-114.69},
                           {-4.8,-67.23,106.54},{2.88,66.99,-107.74}};
  for(int i=0;i<4;++i)for(int j=0;j<3;++j)h.q[g::legIndexFromName(names[i])][j]=values[i][j]*M_PI/180.;
  const auto command=h.mode.acceptCommand(h.q,h.q);
  ASSERT_NE(h.mode.state,r::DockMode::kFailed)<<h.mode.reason;
  const int lf=g::legIndexFromName("lf");h.q[lf][2]=117.12*M_PI/180.;
  h.mode.descent_distance_=.030;
  const auto next=h.tick(true,g::Matrix4d::Identity().eval());
  ASSERT_EQ(h.mode.state,r::DockMode::kPreDescentSettle)<<h.mode.reason;
  EXPECT_NEAR(next[lf][2]*180./M_PI,115.,1e-8);
  for(int leg=0;leg<6;++leg)EXPECT_LT((next[leg]-command[leg]).norm(),1e-12);
  for(int n=0;n<1000&&!r::DockMode::isTerminalState(h.mode.state);++n) {
    const auto previous=h.q;h.tick();
    for(int leg=0;leg<6;++leg)EXPECT_LE((h.q[leg]-previous[leg]).cwiseAbs().maxCoeff(),20.*M_PI/180./30.+1e-8);
  }
  EXPECT_EQ(h.mode.state,r::DockMode::kSuccess)<<h.mode.reason;
}

TEST(RingNative, OutputGuardCatchesDlsAndDirectTargetsAndResetClearsLock) {
  DockHarness h;ASSERT_TRUE(h.until(r::DockMode::kSearchingTag));
  auto bad=h.q;bad[1][1]=-101.*M_PI/180.;const auto actual=h.q;
  auto cmd=h.mode.acceptCommand(bad,h.q);ASSERT_EQ(h.mode.state,r::DockMode::kFailed);
  for(int i=0;i<6;++i)EXPECT_LT((cmd[i]-actual[i]).norm(),1e-12);
  h.controller.resetToStand(h.q);EXPECT_FALSE(h.mode.middleHold());EXPECT_FALSE(h.mode.active);
  for(int i=0;i<300&&h.controller.reset_active;++i)h.tick();
  ASSERT_FALSE(h.controller.reset_active);
  h.controller.enterDock(h.q);EXPECT_FALSE(h.mode.middleHold());
  EXPECT_TRUE(h.until(r::DockMode::kSearchingTag))<<h.mode.reason;
}

TEST(RingNative, KneeHardLimitIsOneHundredDegrees) {
  DockHarness h;ASSERT_TRUE(h.until(r::DockMode::kSearchingTag));
  auto target=h.q;
  target[1][1]=-100.*M_PI/180.;target[1][2]=75.*M_PI/180.;
  const auto command=h.mode.acceptCommand(target,h.q);
  ASSERT_NE(h.mode.state,r::DockMode::kFailed)<<h.mode.reason;
  EXPECT_NEAR(command[1][1],target[1][1],1e-12);
  h.q=command;target[1][1]=-100.1*M_PI/180.;
  h.mode.acceptCommand(target,h.q);
  EXPECT_EQ(h.mode.state,r::DockMode::kFailed);
}

TEST(RingNative, MissingFeedbackCancelsLastCommand) {
  DockHarness h;ASSERT_TRUE(h.until(r::DockMode::kMiddleLift));h.tick(false);
  auto result=h.mode.update({});ASSERT_TRUE(result.failed);ASSERT_TRUE(result.joint_positions);
  for(int leg=0;leg<6;++leg)EXPECT_LT(((*result.joint_positions)[leg]-h.q[leg]).norm(),1e-12);
}

TEST(RingNative, SlowerAlignmentOnlyMovesSupportFeet) {
  DockHarness h;ASSERT_TRUE(h.until(r::DockMode::kSearchingTag));
  h.mode.alignment_speed_m_s_=.020;
  const auto start=*h.mode.last_visual_target;
  auto pose=g::Matrix4d::Identity().eval();pose(0,3)=.030;pose(1,3)=.040;
  h.tick(true,pose);ASSERT_EQ(h.mode.state,r::DockMode::kPrealign)<<h.mode.reason;
  const auto target=*h.mode.last_visual_target;
  for(int leg=0;leg<6;++leg) {
    if(r::DockMode::isMiddleLeg(leg))EXPECT_LT((target[leg]-start[leg]).norm(),1e-12);
    else {
      EXPECT_NEAR(target[leg].x()-start[leg].x(),-.020/30.*.6,1e-10);
      EXPECT_NEAR(target[leg].y()-start[leg].y(),-.020/30.*.8,1e-10);
      EXPECT_NEAR(target[leg].z(),start[leg].z(),1e-12);
    }
  }
}

TEST(RingNative, SuccessHoldKeepsTargetDespiteMiddleOffsetWithoutDockRobotState) {
  DockHarness h;ASSERT_TRUE(h.until(r::DockMode::kSearchingTag));auto pose=g::Matrix4d::Identity().eval();
  for(int i=0;i<1000&&!r::DockMode::isTerminalState(h.mode.state);++i)h.tick(true,pose);
  ASSERT_EQ(h.mode.state,r::DockMode::kSuccess)<<h.mode.reason;
  h.q[2][1]+=4.*M_PI/180.;
  for(int i=0;i<100&&h.mode.state==r::DockMode::kSuccess;++i)
    h.controller.update(h.q,Eigen::Vector4d::Zero());
  EXPECT_EQ(h.mode.state,r::DockMode::kSuccess);
}

TEST(RingNative, OutOfRangeEntryFailsBeforeAnyMiddleMotion) {
  DockHarness h;h.q[1][1]=-102.*M_PI/180.;const auto actual=h.q;
  auto cmd=h.tick(false);EXPECT_EQ(h.mode.state,r::DockMode::kFailed);EXPECT_FALSE(h.mode.middleHold());
  for(int i=0;i<6;++i)EXPECT_LT((cmd[i]-actual[i]).norm(),1e-12);
}
