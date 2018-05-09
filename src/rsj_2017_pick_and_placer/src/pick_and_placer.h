#ifndef RSJ_2017_PICK_AND_PLACER_PICK_AND_PLACER_H
#define RSJ_2017_PICK_AND_PLACER_PICK_AND_PLACER_H

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose2D.h>
#include "pick_and_placer_params.h"
#include "arm.h"
#include "gripper.h"
#include "logger.h"

class PickNPlacer {
 public:
  PickNPlacer(Arm& arm, Logger& logger);
  void DoPickAndPlace(geometry_msgs::Pose2D::ConstPtr const& msg);
  void SetupPlanningScene();
  void AddBoxToScene(geometry_msgs::Pose2D::ConstPtr const& msg);
  void RemoveBoxFromScene();
 private:
  Arm& arm_;
  Logger& logger_;
  moveit::planning_interface::PlanningSceneInterface scene_;
};

#endif
