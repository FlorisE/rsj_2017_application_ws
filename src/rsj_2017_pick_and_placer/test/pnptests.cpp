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

// mock simpleactionclient
#ifndef ACTIONLIB__CLIENT__SIMPLE_ACTION_CLIENT_H_
#define ACTIONLIB__CLIENT__SIMPLE_ACTION_CLIENT_H_

namespace actionlib {
  template<class ActionSpec> class SimpleActionClient {
  public:
    SimpleActionClient(std::string& a, bool& b) {}
  };
}

#endif

#include "../src/pick_and_placer.cpp"

/* Mocks */
// gripper
Gripper::Gripper(std::string name, bool spinThread) : gripper_(name, spinThread) {}
bool Gripper::waitForServer() { return true; }
bool Gripper::waitForResult(const ros::Duration & timeout) { return true; }
void Gripper::sendGoal(const control_msgs::GripperCommandGoal& goal) { }

// arm
Arm::Arm(Gripper& gripper, const std::string& group, const std::string& gripperGroup, const PickNPlacerParams& params)
  : arm_(group), gripper_(gripper), gripper_group_(gripperGroup), params_(params) {}
void Arm::Initialize() {}
bool Arm::DoPick(double x, double y) {}
bool Arm::DoPlace() {}
void Arm::DoMoveVertical() {}
void Arm::SetSupportSurfaceName(const std::string& surfaceName) {}

namespace moveit {
  namespace planning_interface {
    MoveGroupInterface::MoveGroupInterface(const std::string& group, const boost::shared_ptr<tf::Transformer>& tf, const ros::WallDuration& wait_for_servers) {}
    MoveGroupInterface::~MoveGroupInterface() {}
    PlanningSceneInterface::PlanningSceneInterface() {}
  }
}

/* end mocks */

class PickAndPlacer : public testing::Test {
public:
  PickNPlacer *pnp_;

  virtual void SetUp() {
    PickNPlacerParams params;
    Gripper gripper("arm", true);
    pnp_ = new PickNPlacer(gripper, params);
  }
  virtual void TearDown() {}
};

TEST_F(PickAndPlacer, Picks) {

}

TEST_F(PickAndPlacer, Places) {

}

TEST_F(PickAndPlacer, PicksAndPlaces) {

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
