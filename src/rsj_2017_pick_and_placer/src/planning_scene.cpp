#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <std_msgs/ColorRGBA.h>

#include "planning_scene.h"

PlanningScene::PlanningScene(Arm& arm, Logger& logger) 
  : arm_(arm), logger_(logger) {}

void PlanningScene::Initialize() {
    logger_.INFO("Setting up planning scene");
    // Clear the planning scene
    std::vector<std::string> objs;
    for (auto o: scene_.getObjects()) {
        objs.push_back(o.first);
    }
    for (auto o: scene_.getAttachedObjects()) {
        objs.push_back(o.first);
    }
    scene_.removeCollisionObjects(objs);

    AddTable();

    // Let the planner know that this is the surface supporting things we will
    // be picking and placing, so collisions are allowed
    arm_.SetSupportSurfaceName("table");
}

void PlanningScene::AddTable() {
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
}

void PlanningScene::AddBox(double x, double y) {
    logger_.INFO("Adding box to planning scene at %f, %f", x, y);
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
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.016;
    pose.orientation.w = 1;
    sponge.primitives.push_back(primitive);
    sponge.primitive_poses.push_back(pose);
    sponge.operation = sponge.ADD;
    scene_.applyCollisionObject(sponge);
}

void PlanningScene::RemoveBox() {
    logger_.INFO("Removing box from planning scene");
    // Remove the box from the scene
    std::vector<std::string> objs;
    objs.push_back("sponge");
    scene_.removeCollisionObjects(objs);
}
