#ifndef RSJ_2017_PICK_AND_PLACER_ARM_H
#define RSJ_2017_PICK_AND_PLACER_ARM_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <control_msgs/GripperCommandAction.h>

#include "pick_and_placer_params.h"
#include "gripper.h"
#include "logger.h"

class Arm {
public:
  Arm(Gripper& gripper, Logger& logger_, const std::string& group, const std::string& gripperGroup,
      const PickNPlacerParams& params);
  virtual void Initialize();
  virtual bool DoPick(double x, double y);
  virtual bool DoPlace();
  virtual void DoMoveVertical();
  virtual void SetSupportSurfaceName(const std::string& surfaceName);
  virtual ~Arm();
protected:
  moveit::planning_interface::MoveGroupInterface arm_;
  moveit::planning_interface::MoveGroupInterface gripper_group_;
  Gripper& gripper_;
  Logger& logger_;
  const PickNPlacerParams& params_;
  virtual bool DoPickPrepare(geometry_msgs::PoseStamped& pose, double x, double y);
  virtual bool DoOpenGripper(control_msgs::GripperCommandGoal& goal);
  virtual bool DoApproach(geometry_msgs::PoseStamped& pose);
  virtual bool DoGrasp(control_msgs::GripperCommandGoal& goal);
  virtual bool DoPlacePrepare(geometry_msgs::PoseStamped &pose);
  virtual bool DoPlaceApproach(geometry_msgs::PoseStamped &pose);
  virtual bool DoRelease(control_msgs::GripperCommandGoal& goal);
  virtual bool DoRetreat(geometry_msgs::PoseStamped& pose);
  virtual void DoRest(control_msgs::GripperCommandGoal &goal);
  virtual void DoCloseGripper(control_msgs::GripperCommandGoal &goal);
};

#endif //RSJ_2017_PICK_AND_PLACER_ARM_H
