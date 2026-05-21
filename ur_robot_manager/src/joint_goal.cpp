#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager
{
  // --- Joint Goal - Handle Goal --- ///
  rclcpp_action::GoalResponse UrRobotManager::joint_goal_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const JointGoal::Goal> goal) {
    (void)uuid;
    // Check for positions size
    if (goal->positions.size() != 6) {
      RCLCPP_ERROR(this->get_logger(), "[JointGoal] Invalid joint count! Expected 6, got %zu. Rejecting goal.", goal->positions.size());
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "[JointGoal] Accepted Goal.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // --- Joint Goal - Handle Cancel --- ///
  rclcpp_action::CancelResponse UrRobotManager::joint_goal_handle_cancel(const std::shared_ptr<JointGoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "[JointGoal] Received request to cancel goal");
    (void)goal_handle;
    if (move_group_) {
      move_group_->stop(); // Stops the current trajectory execution
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // --- Joint Goal - Handle Accepted --- ///
  void UrRobotManager::joint_goal_handle_accepted(const std::shared_ptr<JointGoalHandle> goal_handle) {
    std::thread{std::bind(&UrRobotManager::joint_goal_handle_execution, this, std::placeholders::_1), goal_handle}.detach();
  }

  // --- Joint Goal - Handle Execution --- ///
  void UrRobotManager::joint_goal_handle_execution(const std::shared_ptr<JointGoalHandle> goal_handle) {
    auto result = std::make_shared<JointGoal::Result>();
    auto goal = goal_handle->get_goal();
    // Clear previous states
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
    move_group_->clearTrajectoryConstraints();
    // Set Joint Targets
    bool within_bounds = move_group_->setJointValueTarget(goal->positions);
    if (!within_bounds) {
      RCLCPP_WARN(this->get_logger(), "[JointGoal] Target joint position(s) were outside of limits, but we will plan and clamp to the limits.");
    }
    // Set settings
    double velocity_scaling = std::max(0.01, std::min(goal->velocity_scaling, 1.0));
    double acceleration_scaling = std::max(0.05, std::min(goal->acceleration_scaling, 1.0));
    move_group_->setMaxVelocityScalingFactor(velocity_scaling);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);
    move_group_->setPlannerId("PTP");
    move_group_->setGoalJointTolerance(0.001);

    // Create Plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "[JointGoal] Planning failed");
      result->success = false;
      result->error_code = 1;
      result->message = "Planning failed";
      goal_handle->abort(result);
      return;
    }

    // Send initial feedback
    auto feedback = std::make_shared<JointGoal::Feedback>();
    feedback->progress = 0.0f;
    goal_handle->publish_feedback(feedback);

    RCLCPP_INFO(this->get_logger(), "[JointGoal] Planning successful. Executing at %d%% speed...", (int)(velocity_scaling * 100));

    // --- Preparing for Feedback ---
    std::vector<double> start_positions;
    move_group_->getCurrentState()->copyJointGroupPositions(planning_group_, start_positions);
    double total_distance = calculate_joint_distance(start_positions, goal->positions);

    // Run the execution in an asynchronous task
    auto exec_future = std::async(std::launch::async, [this, &my_plan]() {
        return move_group_->execute(my_plan);
        });

    // --- Feedback Polling Loop ---
    rclcpp::Rate loop_rate(20); 

    // Loop until the execute task is finished
    while (exec_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      // 1. Check if a cancellation request came in
      if (goal_handle->is_canceling()) {
        RCLCPP_WARN(this->get_logger(), "[JointGoal] Goal canceled during execution!");
        result->success = false;
        result->error_code = 3;
        result->message = "Execution canceled";
        goal_handle->canceled(result);
        return; 
      }

      // 2. Get current joint positions
      std::vector<double> current_positions;
      move_group_->getCurrentState()->copyJointGroupPositions(planning_group_, current_positions);

      // 3. Calculate progress percentage
      if (total_distance > 0.0001) {
        double current_distance_to_goal = calculate_joint_distance(current_positions, goal->positions);
        double progress = (1.0 - (current_distance_to_goal / total_distance)) * 100.0;
        feedback->progress = std::max(0.0f, std::min(100.0f, static_cast<float>(progress)));
      } else {
        feedback->progress = 100.0f;
      }

      // 4. Publish feedback
      goal_handle->publish_feedback(feedback);

      // 5. Sleep to maintain 10Hz
      loop_rate.sleep();
    }

    // Get the final result of the execution from the future
    auto exec_status = exec_future.get();

    if (exec_status == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "[JointGoal] Execution completed successfully.");

      // Final 100% feedback update
      feedback->progress = 100.0f;
      goal_handle->publish_feedback(feedback);

      result->success = true;
      result->error_code = 0;
      result->message = "Execution successful";
      goal_handle->succeed(result);
    } else {
      RCLCPP_ERROR(this->get_logger(), "[JointGoal] Execution failed!");
      result->success = false;
      result->error_code = 2;
      result->message = "Execution failed";
      goal_handle->abort(result);
    }
  }

  // --- Helper Function - Calculate Joint Distance --- ///
  double UrRobotManager::calculate_joint_distance(const std::vector<double>& current, const std::vector<double>& target) {
    double distance = 0.0;
    for (size_t i = 0; i < current.size() && i < target.size(); ++i) {
      distance += std::pow(current[i] - target[i], 2);
    }
    return std::sqrt(distance);
  }
}  // namespace ur_robot_manager
