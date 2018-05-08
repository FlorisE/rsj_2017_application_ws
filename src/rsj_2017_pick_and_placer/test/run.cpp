#include <gmock/gmock.h>

#include "pnptests.cpp"
#include "GripperTest.cpp"
#include "ArmTest.cpp"

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
