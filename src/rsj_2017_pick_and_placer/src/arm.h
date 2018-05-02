#ifndef RSJ_2017_PICK_AND_PLACER_ARM_H
#define RSJ_2017_PICK_AND_PLACER_ARM_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <control_msgs/GripperCommandAction.h>

#include "pick_and_placer_params.h"
#include "gripper.h"

class Arm {
public:
  Arm(Gripper& gripper, const std::string& group, const std::string& gripperGroup, const PickNPlacerParams& params);
  void Initialize();
  bool DoPick(double x, double y);
  bool DoPlace();
  void DoMoveVertical();
  void SetSupportSurfaceName(const std::string& surfaceName);
private:
  moveit::planning_interface::MoveGroupInterface arm_;
  // Planning interface for the gripper (used for planning scene purposes here)
  moveit::planning_interface::MoveGroupInterface gripper_group_;
  Gripper& gripper_;
  const PickNPlacerParams& params_;
  bool DoPickPrepare(geometry_msgs::PoseStamped& pose, double x, double y);
  bool DoOpenGripper(control_msgs::GripperCommandGoal& goal);
  bool DoApproach(geometry_msgs::PoseStamped& pose);
  bool DoGrasp(control_msgs::GripperCommandGoal& goal);
  bool DoPlacePrepare(geometry_msgs::PoseStamped &pose);
  bool DoPlaceApproach(geometry_msgs::PoseStamped &pose);
  bool DoRelease(control_msgs::GripperCommandGoal& goal);
  bool DoRetreat(geometry_msgs::PoseStamped& pose);
  void DoRest(control_msgs::GripperCommandGoal &goal);
  void DoCloseGripper(control_msgs::GripperCommandGoal &goal);
};

#endif //RSJ_2017_PICK_AND_PLACER_ARM_H
