#ifndef RSJ_2017_PICK_AND_PLACER_PLANNING_SCENE_H
#define RSJ_2017_PICK_AND_PLACER_PLANNING_SCENE_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "arm.h"
#include "logger.h"

class PlanningScene {
public:
  PlanningScene(Arm& arm, Logger& logger);
  void Initialize();
  void AddBox(geometry_msgs::Pose2D::ConstPtr const& msg);
  void RemoveBox();

protected:
  void AddTable();
  moveit::planning_interface::PlanningSceneInterface scene_;
  Arm& arm_;
  Logger& logger_;
};

#endif //RSJ_2017_PICK_AND_PLACER_PLANNING_SCENE_H
