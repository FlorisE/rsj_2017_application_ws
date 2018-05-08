//#ifndef ROSCONSOLE_ROSCONSOLE_H
//#define ROSCONSOLE_ROSCONSOLE_H
//
//#include <string>
//
//void ROS_INFO(const std::string& msg) {};
//void ROS_WARN(const std::string& msg) {};
//void ROS_FATAL(const std::string& msg) {};
// 
//#endif
//
#include <ros/ros.h>
#include <gmock/gmock.h>
#include <geometry_msgs/Pose2D.h>
#include <memory>

#include "GripperMock.h"


#include "../src/pick_and_placer.cpp"

#include "SimpleActionClientMock.h"
#include "ArmMock.h"

class PickAndPlacer : public ::testing::Test {
public:
  PickAndPlacer() 
    : params_(PickNPlacerParams()), gripper_(GripperMock("arm", true)), 
      arm_(gripper_, "arm", "gripper", params_), pnp_(PickNPlacer(arm_)) {}

  virtual void SetUp() {
  }

  PickNPlacerParams params_;
  PickNPlacer pnp_;
  ArmMock arm_;
  GripperMock gripper_;
};

TEST_F(PickAndPlacer, DISABLED_InitializesArm) {
  //ASSERT_TRUE(arm_->InitializeCalled);
}
