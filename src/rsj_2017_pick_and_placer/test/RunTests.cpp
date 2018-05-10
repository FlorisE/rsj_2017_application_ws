#include <gmock/gmock.h>

#include "PickNPlacerTest.cpp"
#include "GripperTest.cpp"
#include "ArmTest.cpp"
#include "PlanningSceneTest.cpp"

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
