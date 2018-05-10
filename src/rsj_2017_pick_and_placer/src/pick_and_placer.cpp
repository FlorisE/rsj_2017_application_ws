// Copyright 2017 Geoffrey Biggs (geoffrey.biggs@aist.go.jp)

#include <ros/ros.h>
#include <string>
#include <vector>

#include "pick_and_placer.h"

PickNPlacer::PickNPlacer(Arm& arm, Logger& logger, PlanningScene& scene)
    : arm_(arm), logger_(logger), scene_(scene) {
  // Specify end-effector positions in the configured task frame
  arm_.Initialize();

  // Initialise the planning scene with known objects
  SetupPlanningScene();

  // Start by moving to the vertical pose
  arm_.DoMoveVertical();
}

void PickNPlacer::DoPickAndPlace(double x, double y) {
  // Add the newly-detected object
  scene_.AddBox(x, y);
  // Sleep a little to let the messages flow and be processed
  ros::Duration(sleepTime).sleep();

  // Do the pick-and-place
  if (arm_.DoPick(x, y)) {
    arm_.DoPlace();
  }
  // Remove the object now that we don't care about it any more
  scene_.RemoveBox();
}

void PickNPlacer::SetupPlanningScene() {
  logger_.INFO("Setting up planning scene");
  scene_.Initialize();
}
