#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager {

void UrRobotManager::twist_servo_subscription_callback_(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(twist_servo_mutex_);
    std::string reference_frame = msg->header.frame_id.empty() ? moveit_move_group_->getPlanningFrame() : msg->header.frame_id;
    twist_servo_target_cmd_.frame_id = reference_frame;
    twist_servo_target_cmd_.velocities(0) = msg->twist.linear.x;
    twist_servo_target_cmd_.velocities(1) = msg->twist.linear.y;
    twist_servo_target_cmd_.velocities(2) = msg->twist.linear.z;
    twist_servo_target_cmd_.velocities(3) = msg->twist.angular.x;
    twist_servo_target_cmd_.velocities(4) = msg->twist.angular.y;
    twist_servo_target_cmd_.velocities(5) = msg->twist.angular.z;
    twist_servo_last_msg_time_ = this->now();
    twist_servo_active_ = true;
}

void UrRobotManager::twist_servo_loop_callback_()
{
    if (!twist_servo_active_) return;
    auto now = this->now();
    // Watchdog timeout (e.g., 200ms without receiving a new target)
    if ((now - twist_servo_last_msg_time_).seconds() > 0.2) {
        twist_servo_active_ = false;
        twist_servo_joint_cmd_rolling_window_.clear();
        RCLCPP_WARN(this->get_logger(), "[TwistServo] Stream timed out. Halting.");
        return;
    }
    std::lock_guard<std::mutex> lock(twist_servo_mutex_);
    // Fetch the freshest robot state from the external planning scene
    auto robot_state = servo_planning_scene_monitor_->getStateMonitor()->getCurrentState();
    if (!robot_state) return;
    servo_->setCommandType(moveit_servo::CommandType::TWIST);
    // Calculate inverse kinematics/joint velocities for the next step towards the streamed target twist
    moveit_servo::KinematicState joint_state = servo_->getNextJointState(robot_state, twist_servo_target_cmd_);
    if (servo_->getStatus() != moveit_servo::StatusCode::INVALID) {
        // Append the new state to the rolling window to smooth the trajectory
        moveit_servo::updateSlidingWindow(joint_state, twist_servo_joint_cmd_rolling_window_, servo_params_.max_expected_latency, now);
        if (auto msg = moveit_servo::composeTrajectoryMessage(servo_params_, twist_servo_joint_cmd_rolling_window_)) {
            servo_trajectory_publisher_->publish(msg.value());
        }
        // Apply the newly calculated state back to the local robot_state pointer to maintain continuity
        if (!twist_servo_joint_cmd_rolling_window_.empty()) {
            const auto* jmg = robot_state->getJointModelGroup(servo_params_.move_group_name);
            robot_state->setJointGroupPositions(jmg, twist_servo_joint_cmd_rolling_window_.back().positions);
            robot_state->setJointGroupVelocities(jmg, twist_servo_joint_cmd_rolling_window_.back().velocities);
        }
    }
}

}  // namespace ur_robot_manager
