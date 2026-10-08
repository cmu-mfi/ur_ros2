#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager
{
  // --- Park Service - Setup --- ///
  void UrRobotManager::park_service_setup() {
    // Park Service
    park_service_ = this->create_service<Park>(
        "park",
        std::bind(&UrRobotManager::park_service_callback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
        service_cb_group_
        );
  }

  // --- Park Service - Service Callback --- ///
  void UrRobotManager::park_service_callback(
      const std::shared_ptr<Park::Request> request,
      std::shared_ptr<Park::Response> response) 
  {
    // configure home moveit move
    double velocity_scaling = std::max(0.01, std::min(abs(request->speed), 1.0));
    double acceleration_scaling = velocity_scaling;
    moveit_move_group_->clearPoseTargets();
    moveit_move_group_->clearPathConstraints();
    std::vector<double> park_positions = {0.0, -M_PI/2, M_PI/2, -M_PI/2, -M_PI/2, 0.0}; 
    moveit_move_group_->setJointValueTarget(park_positions);
    moveit_move_group_->setMaxVelocityScalingFactor(velocity_scaling);
    moveit_move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // plan moveit move
    bool success = (moveit_move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "[Park Service] Planning failed. Cannot park.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "[Park Service] Plan successful. Executing movement...");
    // execute moveit move
    auto exec_status = moveit_move_group_->execute(my_plan);
    if (exec_status != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "[Park Service] Execution failed.");
      response->success = false;
      response->message = "Execution failed or was cancelled.";
    }
    // configure park moveit move
    moveit_move_group_->clearPoseTargets();
    moveit_move_group_->clearPathConstraints();
    park_positions = {M_PI/2, -2.0, 2.6, -2.6, -M_PI/2, 0.0}; 
    moveit_move_group_->setJointValueTarget(park_positions);
    moveit_move_group_->setMaxVelocityScalingFactor(velocity_scaling);
    moveit_move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);
    // plan moveit move
    success = (moveit_move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "[Park Service] Planning failed. Cannot park.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "[Park Service] Plan successful. Executing movement...");
    // execute moveit move
    exec_status = moveit_move_group_->execute(my_plan);
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
