#pragma once
#include "grasp_hexapod_control_cpp/control.h"
#include "grasp_hexapod_control_cpp/ring_dock_mode.h"
namespace grasp_hexapod_ring {
// Reuse the existing C++ kinematics, walking, climbing and DLS implementation.
// The original base-class AprilTag DockMode pointer remains null.
class RingController : public grasp_hexapod_control_cpp::GraspController {
 public:
  using GraspController::GraspController;
  DockMode* ring_mode = nullptr;
  void attachDockMode(DockMode* mode) {
    if (!mode) throw ConfigError("ring dock mode is required");
    ring_mode=mode;
  }
  void enterDock(const JointAngles& q) {
    if (!ring_mode) throw ConfigError("ring dock mode is not attached");
    for (const auto& leg:q) if (!leg.allFinite()) throw ConfigError("q_cur must be finite");
    abortClimb(); syncActualFeet(q); setMode(kDock); ring_mode->enter(q,nullptr);
  }
  void exitDock(const JointAngles& q) {
    if(ring_mode && ring_mode->active) ring_mode->exit();
    GraspController::exitDock(q);
  }
  void resetToStand(const JointAngles& q) {
    exitDock(q); GraspController::resetToStand(q);
  }
  void enterClimb(const JointAngles& q,
      const std::optional<ClimbCompactConfig>& config=std::nullopt,
      int start=0,std::optional<int> end=std::nullopt,bool hardware=false) {
    exitDock(q); GraspController::enterClimb(q,config,start,end,hardware);
  }
  JointAngles update(const JointAngles& q,const Eigen::Vector4d& command,
      const NavigationState* navigation=nullptr,const DockRobotState* state=nullptr) {
    if(mode!=kDock || mission.state!=MissionStateMachine::kIdle)
      return GraspController::update(q,command,navigation,nullptr);
    if(!ring_mode) throw ConfigError("ring dock mode is not attached");
    const auto result=ring_mode->update(state?*state:DockRobotState{});
    if(result.joint_positions) {
      for(const auto& leg:*result.joint_positions) if(!leg.allFinite()) {
        ring_mode->failExecution("dock joint target is non-finite"); return q;
      }
      q_des=*result.joint_positions; return q_des;
    }
    if(!result.foot_positions_base) {q_des=q; return q_des;}
    for(const auto& foot:*result.foot_positions_base) if(!foot.allFinite()) {
      ring_mode->failExecution("dock foot target is non-finite");return q;
    }
    foot_desired_base=*result.foot_positions_base;
    return calJointPoses(q);
  }
};
}
