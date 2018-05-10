#ifndef ARM_SPY_H
#define ARM_SPY_H

#include "../src/arm.h"

class ArmSpy: public Arm {
public:
  ArmSpy(Gripper& gripper, Logger& logger, const std::string& group, const std::string& gripperGroup,
         const PickNPlacerParams& params) : Arm(gripper, logger, group, gripperGroup, params) {}

  void Initialize() {
    InitializeCalled = true;
    Arm::Initialize();
  }

  bool DoPick(double x, double y) {
    DoPickCalled = true;
    return Arm::DoPick(x, y);
  }

  bool DoPlace() {
    DoPlaceCalled = true;
    return Arm::DoPlace();
  }

  void DoMoveVertical() {
    DoMoveVerticalCalled = true;
    Arm::DoMoveVertical();
  }

  void SetSupportSurfaceName(const std::string& surfaceName) {
    SetSupportSurfaceNameCalled = true;
    Arm::SetSupportSurfaceName(surfaceName);
  }

  bool InitializeCalled;
  bool DoPickCalled;
  bool DoPlaceCalled;
  bool DoMoveVerticalCalled;
  bool SetSupportSurfaceNameCalled;
  bool DoPickPrepareCalled;
  bool DoOpenGripperCalled;
  bool DoApproachCalled;
  bool DoGraspCalled;
  bool DoRetreatCalled;
  bool DoPlacePrepareCalled;
  bool DoPlaceApproachCalled;
  bool DoReleaseCalled;
  bool DoRestCalled;
protected:
  bool DoPickPrepare(geometry_msgs::PoseStamped& pose, double x, double y) {
    DoPickPrepareCalled = true;
    return Arm::DoPickPrepare(pose, x, y);
  }

  bool DoOpenGripper(control_msgs::GripperCommandGoal& goal) {
    DoOpenGripperCalled = true;
    return Arm::DoOpenGripper(goal);
  }

  bool DoApproach(geometry_msgs::PoseStamped& pose) {
    DoApproachCalled = true;
    return Arm::DoApproach(pose);
  }

  bool DoGrasp(control_msgs::GripperCommandGoal& goal) {
    DoGraspCalled = true;
    return Arm::DoGrasp(goal);
  }

  bool DoRetreat(geometry_msgs::PoseStamped& pose) {
    DoRetreatCalled = true;
    return Arm::DoRetreat(pose);
  }

  bool DoPlacePrepare(geometry_msgs::PoseStamped& pose) {
    DoPlacePrepareCalled = true;
    return Arm::DoPlacePrepare(pose);
  }

  bool DoPlaceApproach(geometry_msgs::PoseStamped& pose) {
    DoPlaceApproachCalled = true;
    return Arm::DoPlaceApproach(pose);
  }

  bool DoRelease(control_msgs::GripperCommandGoal& goal) {
    DoReleaseCalled = true;
    return Arm::DoRelease(goal);
  }

  void DoRest(control_msgs::GripperCommandGoal& goal) {
    DoRestCalled = true;
    return Arm::DoRest(goal);
  }
};

#endif // ARM_SPY_H
