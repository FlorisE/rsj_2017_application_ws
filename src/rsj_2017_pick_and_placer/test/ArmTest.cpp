#include "../src/pick_and_placer_params.h"
#include "GripperSpy.h"
#include "MoveItMock.cpp"
#include "ArmSpy.h"

class ArmTest: public ::testing::Test {
public:
  ArmTest() : arm_(gripper_, logger_, "arm", "gripper", params_) {}
  PickNPlacerParams params_;
  GripperSpy gripper_;
  Logger logger_;
  ArmSpy arm_;
  bool DoPick() {
    double x;
    double y;
    arm_.Initialize();
    return arm_.DoPick(x, y);
  }
  bool DoPlace() {
    arm_.Initialize();
    return arm_.DoPlace();
  }
};

TEST_F(ArmTest, InitializeCallsGripperWaitForServer) {
  arm_.Initialize();

  ASSERT_TRUE(gripper_.WaitForServerCalled);
}

TEST_F(ArmTest, DoPickSucceeds) {
  bool picked = DoPick();

  ASSERT_TRUE(arm_.DoPickPrepareCalled);
  ASSERT_TRUE(arm_.DoOpenGripperCalled);
  ASSERT_TRUE(arm_.DoApproachCalled);
  ASSERT_TRUE(arm_.DoGraspCalled);
  ASSERT_TRUE(arm_.DoRetreatCalled);
  ASSERT_TRUE(picked);
}

TEST_F(ArmTest, DoPlaceSucceeds) {
  bool placed = DoPlace();

  ASSERT_TRUE(arm_.DoPlacePrepareCalled);
  ASSERT_TRUE(arm_.DoPlaceApproachCalled);
  ASSERT_TRUE(arm_.DoReleaseCalled);
  ASSERT_TRUE(arm_.DoRetreatCalled);
  ASSERT_TRUE(arm_.DoRestCalled);
  ASSERT_TRUE(placed);
}

