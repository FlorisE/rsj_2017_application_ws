#ifndef GRIPPER_SPY_H
#define GRIPPER_SPY_H

#include "SimpleActionClientMock.h"
#include "../src/gripper.h"

class GripperSpy: public Gripper {
public:
  GripperSpy(): Gripper("arm", true) {}
  GripperSpy(const std::string &name, bool spinThread): Gripper(name, spinThread) {}
  bool waitForServer() { 
    WaitForServerCalled = true;
    return true; 
  }
  bool waitForResult(const ros::Duration & timeout) {
    WaitForResultCalled = true;
    return true;
  }
  void sendGoal(const control_msgs::GripperCommandGoal& goal) {
    SendGoalCalled = true;
  }
  bool WaitForServerCalled;
  bool WaitForResultCalled;
  bool SendGoalCalled;
};

#endif // GRIPPER_SPY_H
