// Copyright 2017 Geoffrey Biggs (geoffrey.biggs@aist.go.jp)

#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

#include <string>
#include <vector>

#include "pick_and_placer_params.h"
#include "arm.h"
#include "gripper.h"


// Class to provide the node's behaviour and store its state between callbacks
class PickNPlacer {
 public:
  explicit PickNPlacer(Gripper& gripper, const PickNPlacerParams& params)
      : arm_(gripper, "arm", "gripper", params) {
    // Specify end-effector positions in the configured task frame
    //arm_.Initialize();

    // Initialise the planning scene with known objects
    //SetupPlanningScene();

    // Start by moving to the vertical pose
    //arm_.DoMoveVertical();
  }

  void DoPickAndPlace(geometry_msgs::Pose2D::ConstPtr const& msg) {
    // Add the newly-detected object
    AddBoxToScene(msg);
    // Do the pick-and-place
    if (arm_.DoPick(msg->x, msg->y)) {
      arm_.DoPlace();
    }
    // Remove the object now that we don't care about it any more
    RemoveBoxFromScene();
  }

  void SetupPlanningScene() {
    //ROS_INFO("Setting up planning scene");
    // Clear the planning scene
    std::vector<std::string> objs;
    for (auto o: scene_.getObjects()) {
      objs.push_back(o.first);
    }
    for (auto o: scene_.getAttachedObjects()) {
      objs.push_back(o.first);
    }
    scene_.removeCollisionObjects(objs);

    // Add a table to the planning scene (the surface on which objects will be)
    moveit_msgs::CollisionObject table;
    table.header.frame_id = "base_link";
    table.id = "table";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1;
    primitive.dimensions[1] = 1;
    primitive.dimensions[2] = 0.1;
    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = -0.05;
    pose.orientation.w = 1;
    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(pose);
    table.operation = table.ADD;
    std_msgs::ColorRGBA colour;
    colour.b = 0.5;
    colour.a = 1;
    scene_.applyCollisionObject(table, colour);

    // Let the planner know that this is the surface supporting things we will
    // be picking and placing, so collisions are allowed
    arm_.SetSupportSurfaceName("table");
  }

  void AddBoxToScene(geometry_msgs::Pose2D::ConstPtr const& msg) {
    //ROS_INFO("Adding box to planning scene at %f, %f", msg->x, msg->y);
    // Add a box to the scene to represent the object to be picked
    moveit_msgs::CollisionObject sponge;
    sponge.header.frame_id = "base_link";
    sponge.id = "sponge";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.04;
    primitive.dimensions[1] = 0.04;
    primitive.dimensions[2] = 0.031;
    geometry_msgs::Pose pose;
    pose.position.x = msg->x;
    pose.position.y = msg->y;
    pose.position.z = 0.016;
    pose.orientation.w = 1;
    sponge.primitives.push_back(primitive);
    sponge.primitive_poses.push_back(pose);
    sponge.operation = sponge.ADD;
    scene_.applyCollisionObject(sponge);
    // Sleep a little to let the messages flow and be processed
    ros::Duration(1).sleep();
  }

  void RemoveBoxFromScene() {
    //ROS_INFO("Removing box from planning scene");
    // Remove the box from the scene
    std::vector<std::string> objs;
    objs.push_back("sponge");
    scene_.removeCollisionObjects(objs);
  }

private:
  Arm arm_;
  // Object to manage the planning scene
  moveit::planning_interface::PlanningSceneInterface scene_;
  // Topic to receive object positions
  // Variables to hold configured parameters
};
