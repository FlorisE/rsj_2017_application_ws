#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace moveit {
  namespace planning_interface {
    MoveGroupInterface::MoveGroupInterface(const std::string& group, 
                                           const boost::shared_ptr<tf::Transformer>& tf, 
                                           const ros::WallDuration& wait_for_servers) {}
    void MoveGroupInterface::setPoseReferenceFrame(const std::string& surfaceName) {}
    MoveGroupInterface::~MoveGroupInterface() {}
    PlanningSceneInterface::PlanningSceneInterface() {}
  }
}
