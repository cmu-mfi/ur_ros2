#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager
{
  // --- Pose Goal - Handle Goal --- ///
  rclcpp_action::GoalResponse UrRobotManager::pose_goal_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PoseGoal::Goal> goal) {
    (void)uuid;
    // Check for valid method
    if (goal->method != "PTP" && goal->method != "LIN") {
      RCLCPP_ERROR(this->get_logger(), "[PoseGoal] Invalid method! Can be LIN or PTP, got %s", goal->method.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    // Check for a valid quaternion (prevent uninitialized pose math errors)
    auto orientation = goal->target_pose.orientation;
    double q_norm_sq = (orientation.w * orientation.w) + (orientation.x * orientation.x) + (orientation.y * orientation.y) + (orientation.z * orientation.z);
    if (q_norm_sq < 0.99 || q_norm_sq > 1.01) { 
      RCLCPP_ERROR(this->get_logger(), "[PoseGoal] Invalid quaternion! Orientation magnitude is %f (must be 1.0). Rejecting goal.", std::sqrt(q_norm_sq));
      return rclcpp_action::GoalResponse::REJECT;
    }
    // Check for valid speed and acceleration scaling (> 0.0)
    if (goal->velocity_scaling <= 0.0 || goal->acceleration_scaling <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "[PoseGoal] Velocity and acceleration scaling must be greater than 0.0. Rejecting goal.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->velocity_scaling > 1.0 || goal->acceleration_scaling > 1.0) {
      RCLCPP_ERROR(this->get_logger(), "[PoseGoal] Velocity and acceleration scaling can't be more than 1.0. Rejecting goal.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "[PoseGoal] Accepted Goal.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // --- Pose Goal - Handle Cancel --- ///
  rclcpp_action::CancelResponse UrRobotManager::pose_goal_handle_cancel(const std::shared_ptr<PoseGoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "[PoseGoal] Received request to cancel goal");
    (void)goal_handle;
    if (move_group_) {
      move_group_->stop(); // Stops the current trajectory execution
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // --- Pose Goal - Handle Accepted --- ///
  void UrRobotManager::pose_goal_handle_accepted(const std::shared_ptr<PoseGoalHandle> goal_handle) {
    std::thread{std::bind(&UrRobotManager::pose_goal_handle_execution, this, std::placeholders::_1), goal_handle}.detach();
  }

  // --- Pose Goal - Handle Execution --- ///
  void UrRobotManager::pose_goal_handle_execution(const std::shared_ptr<PoseGoalHandle> goal_handle) {
    auto result = std::make_shared<PoseGoal::Result>();
    auto goal = goal_handle->get_goal();

    // Determine the actual target frame and tool0 frame
    std::string tool0_frame = move_group_->getEndEffectorLink();
    std::string target_frame = goal->target_id.empty() ? tool0_frame : goal->target_id;
    std::string reference_frame = goal->frame_id.empty() ? move_group_->getPlanningFrame() : goal->frame_id;
    geometry_msgs::msg::Pose goal_pose = goal->target_pose;
    // If target_frame isnt tool0:
    if (target_frame != tool0_frame) {
      // Check wheter the target_frame is a child of tool0:
      if (!is_frame_tool0_child(target_frame)) {
        RCLCPP_ERROR(this->get_logger(), "[PoseGoal] TF Verification Failed: Target Frame '%s' is not a Child of '%s'", target_frame.c_str(), tool0_frame.c_str());
        result->success = false;
        result->error_code = 1;
        result->message = "Target Frame '" + target_frame + "' is not a Child of '" + tool0_frame + "'";
        goal_handle->abort(result);
        return;
      }
      // Calculate tool0 pose from target pose
      try {
        geometry_msgs::msg::TransformStamped tool0_in_target_msg = 
          tf_buffer_->lookupTransform(target_frame, tool0_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5));
        tf2::Transform target_pose;
        tf2::fromMsg(goal->target_pose, target_pose);
        tf2::Transform tf_tool0_in_target;
        tf2::fromMsg(tool0_in_target_msg.transform, tf_tool0_in_target);
        tf2::Transform tf_tool0_in_frame = target_pose * tf_tool0_in_target;
        tf2::toMsg(tf_tool0_in_frame, goal_pose);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "[PoseGoal] TF Verification Failed: Could not transform %s to %s: %s", 
            tool0_frame.c_str(), target_frame.c_str(), ex.what());
        result->success = false;
        result->error_code = 1;
        result->message = "Invalid TF frames provided";
        goal_handle->abort(result);
        return;
      }
    }
    // Clear previous states
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
    move_group_->clearTrajectoryConstraints();

    // Plan
    move_group_->setPoseReferenceFrame(reference_frame);
    move_group_->setPoseTarget(goal_pose, tool0_frame);
    move_group_->setMaxVelocityScalingFactor(goal->velocity_scaling);
    move_group_->setMaxAccelerationScalingFactor(goal->acceleration_scaling);
    move_group_->setPlannerId(goal->method);
    geometry_msgs::msg::PoseStamped start_pose = move_group_->getCurrentPose(tool0_frame);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "[PoseGoal] Planning failed!");
      result->success = false;
      result->error_code = 1; // planning_failed
      result->message = "Planning failed";
      goal_handle->abort(result);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "[PoseGoal] Planning successful. Executing at %d%% speed...", (int)(goal->velocity_scaling * 100));

    // Execution
    auto exec_future = std::async(std::launch::async, [this, &my_plan]() {
        return move_group_->execute(my_plan);
        });

    // Feedback 
    double total_distance = calculate_cartesian_distance(start_pose.pose.position, goal_pose.position);
    rclcpp::Rate loop_rate(20);
    auto feedback = std::make_shared<PoseGoal::Feedback>();
    while (exec_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      // Check for cancellation
      if (goal_handle->is_canceling()) {
        RCLCPP_WARN(this->get_logger(), "[PoseGoal] Goal canceled during execution!");
        result->success = false;
        result->error_code = 3;
        result->message = "Execution canceled by user";
        goal_handle->canceled(result);
        return;
      }
      // Calculate progress
      geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose(tool0_frame);
      if (total_distance > 0.001) {
        double current_distance = calculate_cartesian_distance(current_pose.pose.position, goal_pose.position);
        double progress = (1.0 - (current_distance / total_distance)) * 100.0;
        feedback->progress = std::max(0.0f, std::min(100.0f, static_cast<float>(progress)));
      } else {
        // purely rotational
        feedback->progress = 50.0f;
      }
      // Publish and sleep
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }

    // Finish Execution
    auto exec_status = exec_future.get();
    if (exec_status == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "[PoseGoal] Execution completed successfully.");
      feedback->progress = 100.0f;
      goal_handle->publish_feedback(feedback);
      result->success = true;
      result->error_code = 0;
      result->message = "Execution successful";
      goal_handle->succeed(result);
    } else {
      RCLCPP_ERROR(this->get_logger(), "[PoseGoal] Execution failed!");
      result->success = false;
      result->error_code = 2;
      result->message = "Execution failed";
      goal_handle->abort(result);
    }
  }

  // --- Helper Function - Calculate Cartesion Distance --- ///
  double UrRobotManager::calculate_cartesian_distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
      return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
    };
  // --- Helper Function - Check if transform is a child of tool0 --- ///
  bool UrRobotManager::is_frame_tool0_child(const std::string& target_frame) {
    const std::string tool0_frame = move_group_->getEndEffectorLink();
    if (target_frame == tool0_frame) {
      return true;
    }
    std::string current_frame = target_frame;
    std::string next_parent;
    tf2::TimePoint time = tf2::TimePointZero; 
    while (tf_buffer_->_getParent(current_frame, time, next_parent)) {
      if (next_parent == tool0_frame) {
        return true;
      }
      current_frame = next_parent;
    }
    return false;
  }
}  // namespace ur_robot_manager
