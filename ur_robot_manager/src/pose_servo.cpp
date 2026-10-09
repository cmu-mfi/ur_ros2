#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager {

void UrRobotManager::pose_servo_subscription_callback_(const PoseServo::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(pose_servo_mutex_);
  // Transform target pose to tool0 frame
  std::string reference_frame = msg->frame_id.empty() ? moveit_move_group_->getPlanningFrame() : msg->frame_id;
  geometry_msgs::msg::Pose goal_pose;

  try {
    goal_pose = transform_pose_to_tool0_frame(msg->target_pose, msg->target_id);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "[PoseServo] TF Verification Failed: %s", ex.what());
    return;
  }

  pose_servo_target_pose_cmd_.frame_id = reference_frame;
  tf2::fromMsg(goal_pose, pose_servo_target_pose_cmd_.pose);
  pose_servo_last_msg_time_ = this->now();
  pose_servo_active_ = true;
}

void UrRobotManager::pose_servo_loop_callback_() {
    if (!pose_servo_active_) return;
    auto now = this->now();
    // Watchdog timeout (e.g., 200ms without receiving a new target)
    if ((now - pose_servo_last_msg_time_).seconds() > 0.2) {
        pose_servo_active_ = false;
        pose_servo_joint_cmd_rolling_window_.clear();
        RCLCPP_WARN(this->get_logger(), "[Servo] Stream timed out. Halting.");
        return;
    }
    std::lock_guard<std::mutex> lock(pose_servo_mutex_);
    // Fetch the freshest robot state from the external planning scene
    auto robot_state = servo_planning_scene_monitor_->getStateMonitor()->getCurrentState();
    if (!robot_state) return;
    // Calculate inverse kinematics for the next step towards the streamed target pose
    moveit_servo::KinematicState joint_state = servo_->getNextJointState(robot_state, pose_servo_target_pose_cmd_);
    if (servo_->getStatus() != moveit_servo::StatusCode::INVALID) {
        // Append the new state to the rolling window to smooth the trajectory
        moveit_servo::updateSlidingWindow(joint_state, pose_servo_joint_cmd_rolling_window_, servo_params_.max_expected_latency, now);
        if (auto msg = moveit_servo::composeTrajectoryMessage(servo_params_, pose_servo_joint_cmd_rolling_window_)) {
            servo_trajectory_publisher_->publish(msg.value());
        }
        // Apply the newly calculated state back to the local robot_state pointer to maintain continuity for the next iteration step.
        if (!pose_servo_joint_cmd_rolling_window_.empty()) {
            const auto* jmg = robot_state->getJointModelGroup(servo_params_.move_group_name);
            robot_state->setJointGroupPositions(jmg, pose_servo_joint_cmd_rolling_window_.back().positions);
            robot_state->setJointGroupVelocities(jmg, pose_servo_joint_cmd_rolling_window_.back().velocities);
        }
    }
}

}  // namespace ur_robot_manager
