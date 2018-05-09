#ifndef ARM_SPY_H
#define ARM_SPY_H

#include "../src/arm.h"

class ArmSpy : public Arm {
public:
  ArmSpy(Gripper& gripper, const std::string& group, const std::string& gripperGroup,
         const PickNPlacerParams& params) : Arm(gripper, group, gripperGroup, params) {}

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
};

#endif // ARM_SPY_H
