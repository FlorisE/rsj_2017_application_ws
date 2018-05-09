#include "GripperMock.h"
#include "ArmSpy.h"
#include "../src/pick_and_placer.h"

class PickAndPlacer : public ::testing::Test {
public:
  PickAndPlacer() 
    : arm_(gripper_, logger_, "arm", "gripper", params_), pnp_(arm_, logger_) {}

  GripperMock gripper_;
  Logger logger_;
  PickNPlacerParams params_;
  ArmSpy arm_;
  PickNPlacer pnp_;
};

TEST_F(PickAndPlacer, InitializesArm) {
  ASSERT_TRUE(arm_.InitializeCalled);
}
