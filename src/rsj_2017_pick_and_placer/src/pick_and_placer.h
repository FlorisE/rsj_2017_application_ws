#ifndef RSJ_2017_PICK_AND_PLACER_PICK_AND_PLACER_H
#define RSJ_2017_PICK_AND_PLACER_PICK_AND_PLACER_H

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose2D.h>
#include "pick_and_placer_params.h"
#include "arm.h"
#include "gripper.h"
#include "logger.h"
#include "planning_scene.h"

class PickNPlacer {
public:
  PickNPlacer(Arm& arm, Logger& logger, PlanningScene& scene);
  void DoPickAndPlace(double x, double y);
  void SetupPlanningScene();
  int sleepTime = 1; // public so that for example tests can skip sleeping
private:
  Arm& arm_;
  Logger& logger_;
  PlanningScene& scene_;
};

#endif
