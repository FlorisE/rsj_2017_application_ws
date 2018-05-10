#include <geometry_msgs/Pose2D.h>
#include "GripperSpy.h"
#include "ArmSpy.h"
#include "PlanningSceneSpy.h"
#include "../src/pick_and_placer.h"

class PickAndPlacer : public ::testing::Test {
public:
  PickAndPlacer() 
    : arm_(gripper_, logger_, "arm", "gripper", params_), scene_(arm_, logger_),
      pnp_(arm_, logger_, scene_) {
    pnp_.sleepTime = 0;
  }

  GripperSpy gripper_;
  Logger logger_;
  PickNPlacerParams params_;
  ArmSpy arm_;
  PlanningSceneSpy scene_;
  PickNPlacer pnp_;
  void DoPickAndPlace() {
    double x;
    double y;
    ros::Time::init(); // needed for "sleeping" after a box has been added to the scene
    pnp_.DoPickAndPlace(x, y);
  }
};

TEST_F(PickAndPlacer, InitializesArm) {
  ASSERT_TRUE(arm_.InitializeCalled);
  ASSERT_TRUE(scene_.InitializeCalled);
  ASSERT_TRUE(arm_.DoMoveVerticalCalled);
}

TEST_F(PickAndPlacer, DoesPickAndPlace) {
  DoPickAndPlace();
  ASSERT_TRUE(scene_.AddBoxCalled);
  ASSERT_TRUE(arm_.DoPickCalled);
  ASSERT_TRUE(arm_.DoPlaceCalled);
  ASSERT_TRUE(scene_.RemoveBoxCalled);
}
