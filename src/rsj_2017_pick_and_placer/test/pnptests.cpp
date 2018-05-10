#include "GripperMock.h"
#include "ArmMock.h"
#include "PlanningSceneMock.h"
#include "../src/pick_and_placer.h"

class PickAndPlacer : public ::testing::Test {
public:
  PickAndPlacer() 
    : arm_(gripper_, logger_, "arm", "gripper", params_), scene_(arm_, logger_),
      pnp_(arm_, logger_, scene_) {}

  GripperMock gripper_;
  Logger logger_;
  PickNPlacerParams params_;
  ArmMock arm_;
  PlanningSceneMock scene_;
  PickNPlacer pnp_;
};

TEST_F(PickAndPlacer, InitializesArm) {
  ASSERT_TRUE(arm_.InitializeCalled);
  ASSERT_TRUE(scene_.InitializeCalled);
  ASSERT_TRUE(arm_.DoMoveVerticalCalled);
}
