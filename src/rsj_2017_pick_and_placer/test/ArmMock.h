#ifndef ARM_MOCK_H
#define ARM_MOCK_H

class ArmMock : public Arm {
public:
  ArmMock(Gripper& gripper, const std::string& group, 
          const std::string& gripperGroup, const PickNPlacerParams& params)
    : arm_(group), gripper_(gripper), gripper_group_(gripperGroup), 
      params_(params), Arm(gripper, group, gripperGroup, params) {}
  void Initialize() {InitializeCalled = true;}
  bool DoPick(double x, double y) {}
  bool DoPlace() {}
  void DoMoveVertical() {}
  void SetSupportSurfaceName(const std::string& surfaceName) {}
  bool InitializeCalled;
private:
  moveit::planning_interface::MoveGroupInterface arm_;
  moveit::planning_interface::MoveGroupInterface gripper_group_;
  Gripper& gripper_;
  const PickNPlacerParams& params_;
};

#endif // ARM_MOCK_H
