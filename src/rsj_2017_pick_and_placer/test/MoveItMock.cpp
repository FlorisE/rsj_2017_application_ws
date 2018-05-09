#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

const std::vector<std::string> linkNames;
const std::map<std::string, moveit_msgs::CollisionObject> collisionObjects;
const std::map<std::string, moveit_msgs::AttachedCollisionObject> attachedObjects;

typedef std_msgs::ColorRGBA ColorRGBA;

namespace moveit {
  namespace planning_interface {
    /* MoveGroupInterface */
    MoveGroupInterface::MoveGroupInterface(const std::string& group, 
                                           const boost::shared_ptr<tf::Transformer>& tf, 
                                           const ros::WallDuration& wait_for_servers) {}
    void MoveGroupInterface::setPoseReferenceFrame(const std::string& surfaceName) {}
    MoveGroupInterface::~MoveGroupInterface() {}
    bool MoveGroupInterface::setPoseTarget(const geometry_msgs::PoseStamped& target, 
                                           const std::string& end_effector_link) {
      return true;
    }
    bool MoveGroupInterface::setPoseTargets(const std::vector<geometry_msgs::PoseStamped>& target, 
                                            const std::string& end_effector_link) {
      return true;
    }
    MoveItErrorCode MoveGroupInterface::move() {
      return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::SUCCESS);
    }
    const std::vector<std::string>& MoveGroupInterface::getLinkNames() {
      return ::linkNames;
    }
    bool MoveGroupInterface::attachObject(const std::string& object, const std::string& link, 
                                          const std::vector<std::string>& touch_links) {
      return true;
    }
    bool MoveGroupInterface::detachObject(const std::string& name) { return true; }
    bool MoveGroupInterface::setNamedTarget(const std::string& name) { return true; }
    void MoveGroupInterface::setSupportSurfaceName(const std::string& name) {}
    /* /MoveGroupInterface */

    /* PlanningSceneInterface */
    PlanningSceneInterface::PlanningSceneInterface() {}
    PlanningSceneInterface::~PlanningSceneInterface() {}

    std::map<std::string, moveit_msgs::CollisionObject> 
    PlanningSceneInterface::getObjects(const std::vector<std::string>&) {
      return ::collisionObjects;
    }

    bool PlanningSceneInterface::applyCollisionObject(const moveit_msgs::CollisionObject& obj) {
      return true;
    }

    bool PlanningSceneInterface::applyCollisionObject(const moveit_msgs::CollisionObject& obj,
                                                      const std_msgs::ColorRGBA& colour) { 
      return true;
    }

    std::map<std::string, moveit_msgs::AttachedCollisionObject>
    PlanningSceneInterface::getAttachedObjects(const std::vector<std::string>& object_ids) {
      return ::attachedObjects;
    }
    void PlanningSceneInterface::removeCollisionObjects(const std::vector<std::string>& object_ids) const {}
    /* /PlanningSceneInterface */
  }
}
