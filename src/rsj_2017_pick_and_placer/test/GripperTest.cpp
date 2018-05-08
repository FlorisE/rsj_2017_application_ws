class GripperTest : public ::testing::Test {
public:
  GripperTest() : gripper_("arm", true) {}
  Gripper gripper_;
};

TEST_F(GripperTest, DISABLED_ForwardsWaitForServer) {} 
TEST_F(GripperTest, DISABLED_ForwardsWaitForResult) {} 
TEST_F(GripperTest, DISABLED_ForwardsSendGoal) {} 
