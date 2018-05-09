#include "../src/pick_and_placer_params.h"
#include "GripperMock.h"
#include "MoveItMock.cpp"
#include "ArmSpy.h"

class ArmTest : public ::testing::Test {
public:
  ArmTest() : arm_(gripper_, "arm", "gripper", params_) {}
  PickNPlacerParams params_;
  GripperMock gripper_;
  ArmSpy arm_;
  bool DoPick() {
    double x;
    double y;
    arm_.Initialize();
    arm_.DoPick(x, y);
  }
};

TEST_F(ArmTest, InitializeCallsGripperWaitForServer) {
  arm_.Initialize();

  ASSERT_TRUE(gripper_.WaitForServerCalled);
}

TEST_F(ArmTest, DoPickCallsDoPickPrepare) {
  DoPick();

  ASSERT_TRUE(arm_.DoPickPrepareCalled);
}

TEST_F(ArmTest, DoPickCallsDoOpenGripper) {
  DoPick();

  ASSERT_TRUE(arm_.DoOpenGripperCalled);
}
  
TEST_F(ArmTest, DoPickCallsDoApproach) {
  DoPick();

  ASSERT_TRUE(arm_.DoApproachCalled);
}

TEST_F(ArmTest, DoPickCallsDoGrasp) {
  DoPick();

  ASSERT_TRUE(arm_.DoGraspCalled);
}

TEST_F(ArmTest, DoPickCallsDoRetreat) {
  DoPick();

  ASSERT_TRUE(arm_.DoRetreatCalled);
}

TEST_F(ArmTest, DoPickSucceeds) {
  ASSERT_TRUE(DoPick());
}
