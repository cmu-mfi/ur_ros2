#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager
{
  void UrRobotManager::park_service_callback(
      const std::shared_ptr<Park::Request> request,
      std::shared_ptr<Park::Response> response) 
  {
    double velocity_scaling = std::max(0.01, std::min(abs(request->speed), 1.0));
    double acceleration_scaling = velocity_scaling;
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
    std::vector<double> park_positions = {0.0, -M_PI/2, M_PI/2, -M_PI/2, -M_PI/2, 0.0}; 
    move_group_->setJointValueTarget(park_positions);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "[Park Service] Planning failed. Cannot park.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "[Park Service] Plan successful. Executing movement...");
    auto exec_status = move_group_->execute(my_plan);
    if (exec_status != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "[Park Service] Execution failed.");
        response->success = false;
        response->message = "Execution failed or was cancelled.";
    }
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
    park_positions = {M_PI/2, -2.0, 2.6, -2.6, -M_PI/2, 0.0}; 
    move_group_->setJointValueTarget(park_positions);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);
    success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "[Park Service] Planning failed. Cannot park.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "[Park Service] Plan successful. Executing movement...");
    exec_status = move_group_->execute(my_plan);
    if (exec_status == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "[Park Service] Successfully parked.");
        response->success = true;
        response->message = "Robot successfully parked.";
    } else {
        RCLCPP_ERROR(this->get_logger(), "[Park Service] Execution failed.");
        response->success = false;
        response->message = "Execution failed or was cancelled.";
    }
  }
}  // namespace ur_robot_managr
