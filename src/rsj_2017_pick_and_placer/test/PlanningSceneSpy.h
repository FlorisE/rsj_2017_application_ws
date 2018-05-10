#include "../src/planning_scene.h"

#ifndef PLANNING_SCENE_SPY_H
#define PLANNING_SCENE_SPY_H

class PlanningSceneSpy: public PlanningScene {
public:
  PlanningSceneSpy(Arm& arm, Logger& logger): PlanningScene(arm, logger) {}
  void Initialize() {
    InitializeCalled = true;
    PlanningScene::Initialize();
  }

  void AddBox(double x, double y) {
    AddBoxCalled = true;
    PlanningScene::AddBox(x, y);
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

#endif // PLANNING_SCENE_SPY_H
