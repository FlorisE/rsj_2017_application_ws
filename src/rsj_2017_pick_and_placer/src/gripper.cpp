#include "gripper.h"

bool Gripper::waitForServer() { return gripper_.waitForServer(); }
bool Gripper::waitForResult(const ros::Duration & timeout) { return gripper_.waitForResult(timeout); }
void Gripper::sendGoal(const control_msgs::GripperCommandGoal& goal) { gripper_.sendGoal(goal); }
