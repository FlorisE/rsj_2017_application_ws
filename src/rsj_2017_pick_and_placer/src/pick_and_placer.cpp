// Copyright 2017 Geoffrey Biggs (geoffrey.biggs@aist.go.jp)

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

void PickNPlacer::DoPickAndPlace(geometry_msgs::Pose2D::ConstPtr const& msg) {
  // Add the newly-detected object
  scene_.AddBox(msg);
  // Do the pick-and-place
  if (arm_.DoPick(msg->x, msg->y)) {
    arm_.DoPlace();
  }
  // Remove the object now that we don't care about it any more
  scene_.RemoveBox();
}

void PickNPlacer::SetupPlanningScene() {
  logger_.INFO("Setting up planning scene");
  scene_.Initialize();
}
