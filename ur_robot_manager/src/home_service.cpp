#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager {
  // --- Home Service - Setup --- ///
  void UrRobotManager::home_service_setup() {
    // Home Service
    home_service_ = this->create_service<Home>(
        "home",
        std::bind(&UrRobotManager::home_service_callback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
        service_cb_group_
        );
  }

  // --- Home Service - Service Callback --- ///
  void UrRobotManager::home_service_callback(
      const std::shared_ptr<Home::Request> request,
      std::shared_ptr<Home::Response> response) 
  {
    // Make sure Robot is IDLE
    {
      std::lock_guard<std::mutex> lock(control_mode_mutex_);
      if (control_mode_ != ControlMode::IDLE) {
        RCLCPP_ERROR(this->get_logger(),
            "[Home Service] Robot is already being controlled (current mode: %s).",
            controlModeToString(control_mode_));
        response->success = false;
        response->message = "Robot is already being controlled.";
        return;
      }
      control_mode_ = ControlMode::HOME;
    }
    ModeReset mode_reset = {control_mode_mutex_, control_mode_};
    // configure moveit move
    moveit_move_group_->setStartStateToCurrentState();
    moveit_move_group_->clearPoseTargets();
    moveit_move_group_->clearPathConstraints();
    moveit_move_group_->setPlannerId("PTP");
    // position
    std::vector<double> home_positions = {0.0, -M_PI/2, M_PI/2, -M_PI/2, -M_PI/2, 0.0}; 
    if (!moveit_move_group_->setJointValueTarget(home_positions)) {
      RCLCPP_ERROR(this->get_logger(), "[Home Service] Target joint values are out of bounds.");
      response->success = false;
      response->message = "Target joint values out of bounds.";
      return;
    }
    // velocity and acceleration
    double velocity_scaling = std::max(0.01, std::min(abs(request->speed), 1.0));
    double acceleration_scaling = velocity_scaling;
    moveit_move_group_->setMaxVelocityScalingFactor(velocity_scaling);
    moveit_move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);
    // plan moveit move
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (moveit_move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "[Home Service] Planning failed. Cannot move home.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "[Home Service] Plan successful. Executing movement...");
    // execute moveit move
    auto exec_status = moveit_move_group_->execute(my_plan);
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

}  // namespace ur_robot_manager
