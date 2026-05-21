#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager
{
  void UrRobotManager::handle_home_service(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
  {
    (void)request;
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
    std::vector<double> home_positions = {0.0, -M_PI/2, M_PI/2, -M_PI/2, -M_PI/2, 0.0}; 
    move_group_->setJointValueTarget(home_positions);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.1);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "[Home Service] Planning failed. Cannot move home.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "[Home Service] Plan successful. Executing movement...");
    auto exec_status = move_group_->execute(my_plan);
    if (exec_status == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "[Home Service] Successfully homed.");
        response->success = true;
        response->message = "Robot successfully homed.";
    } else {
        RCLCPP_ERROR(this->get_logger(), "[Home Service] Execution failed.");
        response->success = false;
        response->message = "Execution failed or was cancelled.";
    }
  }
}  // namespace ur_robot_managr
