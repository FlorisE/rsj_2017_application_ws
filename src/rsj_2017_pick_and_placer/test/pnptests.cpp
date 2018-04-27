#include <ros/ros.h>
#include <gtest/gtest.h>
#include <geometry_msgs/Pose2D.h>

#include "../src/pick_and_placer.cpp"

ros::NodeHandle *nh_;

class PickAndPlacer : testing::Test {
public:
  PickNPlacer *pnp_;
  virtual void SetUp() {
    pnp_ = new PickNPlacer(*nh_);
  }
};

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "pickandplacer");
  nh_ = new ros::NodeHandle();
  return RUN_ALL_TESTS();
}
