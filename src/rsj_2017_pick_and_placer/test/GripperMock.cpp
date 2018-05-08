#include "../src/gripper.h"

class GripperMock : public Gripper {
public:
  GripperMock(const std::string &name, bool spinThread) : Gripper(name, spinThread) {}
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
}
