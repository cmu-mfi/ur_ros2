#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager
{
  // --- MoveIt - Setup --- ///
  void UrRobotManager::moveit_setup() {
    // moveit setup
    moveit::planning_interface::MoveGroupInterface::Options options(moveit_planning_group_, "robot_description", "/"+ns_);
    moveit_move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), options);
    moveit_planning_scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    // default settings
    moveit_move_group_->setPlanningTime(5.0);
    moveit_move_group_->setNumPlanningAttempts(10);
    moveit_move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
    moveit_move_group_->setPlannerId("PTP");
  }
}  // namespace ur_robot_manager
