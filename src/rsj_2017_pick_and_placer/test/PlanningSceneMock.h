#include "../src/planning_scene.h"

class PlanningSceneMock: public PlanningScene {
public:
  PlanningSceneMock(Arm& arm, Logger& logger): PlanningScene(arm, logger) {}
  void Initialize() {
    InitializeCalled = true;
    PlanningScene::Initialize();
  }

  void AddBox(geometry_msgs::Pose2D::ConstPtr const& msg) {
    AddBoxCalled = true;
    PlanningScene::AddBox(msg);
  }

  void RemoveBox() {
    RemoveBoxCalled = true;
    PlanningScene::RemoveBox();
  }

  bool InitializeCalled;
  bool AddBoxCalled;
  bool RemoveBoxCalled;
  bool AddTableCalled;
private:
  void AddTable() {
    AddTableCalled = true;
    PlanningScene::AddTable();
  }
};
