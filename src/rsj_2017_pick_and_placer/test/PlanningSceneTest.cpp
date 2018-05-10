#include "../src/pick_and_placer_params.h"
#include "../src/logger.h"
#include "GripperSpy.h"
#include "ArmSpy.h"
#include "PlanningSceneSpy.h"

class PlanningSceneTest: public ::testing::Test {
public:
  PlanningSceneTest(): arm_(gripper_, logger_, "arm", "gripper", params_), scene_(arm_, logger_) {}
  PickNPlacerParams params_;
  GripperSpy gripper_;
  Logger logger_;
  ArmSpy arm_;
  PlanningSceneSpy scene_;
};

TEST_F(PlanningSceneTest, Initializes) {
  scene_.Initialize();
  ASSERT_TRUE(scene_.AddTableCalled);
  ASSERT_TRUE(arm_.SetSupportSurfaceNameCalled);
}
