#ifndef RSJ_2017_PICK_AND_PLACER_GRIPPER_H
#define RSJ_2017_PICK_AND_PLACER_GRIPPER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>

class Gripper {
public:
  Gripper(const std::string &name, bool spinThread) : gripper_(name, spinThread) {};
  virtual bool waitForServer();
  virtual bool waitForResult(const ros::Duration & timeout = ros::Duration(0, 0));
  virtual void sendGoal(const control_msgs::GripperCommandGoal& goal);
private:
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_;
};

#endif //RSJ_2017_PICK_AND_PLACER_GRIPPER_H
