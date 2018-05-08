#include "../src/pick_and_placer_params.h"
#include "GripperMock.h"

class ArmTest : public ::testing::Test {
public:
  ArmTest() : arm_(gripper_, "arm", "gripper", params_) {}
  PickNPlacerParams params_;
  GripperMock gripper_;
  Arm arm_;
};

TEST_F(ArmTest, SetsPoseReferenceFrame) {
  
}
