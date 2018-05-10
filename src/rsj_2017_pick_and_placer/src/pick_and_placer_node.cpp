#include <ros/ros.h>

#include "gripper.h"
#include "arm.h"
#include "pick_and_placer.cpp"

ros::Subscriber sub_;

class PickAndPlacerNode {
public:
  PickAndPlacerNode(PickNPlacer& pnp) : pnp_(pnp) {
  }

  void DoPickAndPlace(const geometry_msgs::Pose2D::ConstPtr &msg) {
    pnp_.DoPickAndPlace(msg->x, msg->y);
  }

private:
  PickNPlacer& pnp_;
};

void GetROSParams(PickNPlacerParams& params) {
  // Get the value for the configurable values from the parameter server, and
  // set sensible defaults for those values not specified on the parameter
  // server
  ros::param::param<float>(
    "~place_x",
    params.place_x_,
    0.1);
  ros::param::param<float>("~place_y", params.place_y_, -0.2);
  ros::param::param<std::__cxx11::string>(
    "~task_frame",
    params.scene_task_frame_,
    "base_link");
  ros::param::param<float>("~pick_prepare_z", params.pick_prepare_z_, 0.1);
  ros::param::param<float>("~pick_z", params.pick_z_, 0.05);
  ros::param::param<float>("~place_prepare_z", params.place_prepare_z_, 0.1);
  ros::param::param<float>("~place_z", params.place_z_, 0.05);
  ros::param::param<float>("~gripper_open", params.gripper_open_, 0.1);
  ros::param::param<float>("~gripper_close", params.gripper_close_, 0.015);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pickandplacer");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh;
  // Create an instance of the class that implements the node's behaviour
  PickNPlacerParams params;
  GetROSParams(params);

  Gripper gripper("/crane_plus_gripper/gripper_command", "true");
  Logger logger;
  Arm arm(gripper, logger, "arm", "gripper", params);
  PlanningScene scene(arm, logger);

  PickNPlacer pnp(arm, logger, scene);
  auto node = PickAndPlacerNode(pnp);

  // Subscribe to the "/block" topic to receive object positions; excecute
  // DoPickAndPlace() when one is received
  sub_ = nh.subscribe("/block", 1, &PickAndPlacerNode::DoPickAndPlace, &node);

  // Wait until the node is shut down
  ros::waitForShutdown();

  ros::shutdown();
  return 0;
}
