#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager {

void UrRobotManager::setup_servo() {
  // 1. Initialize Servo Parameters and Planning Scene Monitor
  servo_param_listener_ = std::make_shared<servo::ParamListener>(shared_from_this(), "moveit_servo");
  servo_params_ = servo_param_listener_->get_params();
  // This automatically hooks into the externally maintained planning scene via ROS 2 topics
  planning_scene_monitor_ = moveit_servo::createPlanningSceneMonitor(shared_from_this(), servo_params_);
  // 2. Initialize the Servo object
  servo_ = std::make_unique<moveit_servo::Servo>(shared_from_this(), servo_param_listener_, planning_scene_monitor_);
  servo_->setCommandType(moveit_servo::CommandType::POSE);
  // 3. Setup Publisher & Subscriber
  trajectory_cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      servo_params_.command_out_topic, rclcpp::SystemDefaultsQoS());
  servo_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose_servo", rclcpp::SystemDefaultsQoS(),
      std::bind(&UrRobotManager::servo_target_callback, this, std::placeholders::_1));
  // 4. Setup 500Hz Timer for Real-Time Execution
  servo_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  servo_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(servo_params_.publish_period),
      std::bind(&UrRobotManager::servo_loop_callback, this),
      servo_cb_group_);
}

void UrRobotManager::servo_target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(servo_mutex_);
    // Transform from any tool frame to tool0 target pose
    // std::string tool0_frame = move_group_->getEndEffectorLink();
    // std::string target_frame = msg->child_frame_id.empty() ? tool0_frame : msg->child_frame_id;
    // if (msg->header.frame_id.empty()) {
    //   RCLCPP_ERROR(this->get_logger(), "[PoseServo] TF Verification Failed: frame_id can't be empty");
    // }
    // std::string reference_frame = msg->header.frame_id;
    // geometry_msgs::msg::Transform goal_transform = msg->transform;
    // // If target_frame isnt tool0:
    // if (target_frame != tool0_frame) {
    //   // Check wheter the target_frame is a child of tool0:
    //   if (!is_frame_tool0_child(target_frame)) {
    //     RCLCPP_ERROR(this->get_logger(), "[PoseServo] TF Verification Failed: Target Frame '%s' is not a Child of '%s'", target_frame.c_str(), tool0_frame.c_str());
    //     return;
    //   }
    //   // Calculate tool0 pose from target pose
    //   try {
    //     geometry_msgs::msg::TransformStamped tool0_in_target_msg = 
    //       tf_buffer_->lookupTransform(target_frame, tool0_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5));
    //     tf2::Transform target_pose;
    //     tf2::fromMsg(msg->transform, target_pose);
    //     tf2::Transform tf_tool0_in_target;
    //     tf2::fromMsg(tool0_in_target_msg.transform, tf_tool0_in_target);
    //     tf2::Transform tf_tool0_in_frame = target_pose * tf_tool0_in_target;
    //     tf2::toMsg(tf_tool0_in_frame, goal_transform);
    //   } catch (const tf2::TransformException & ex) {
    //     RCLCPP_ERROR(this->get_logger(), "[PoseServo] TF Verification Failed: Could not transform %s to %s: %s", 
    //         tool0_frame.c_str(), target_frame.c_str(), ex.what());
    //     return;
    //   }
    // }
    // servo_target_pose_cmd_.pose = tf2::transformToEigen(goal_transform);
    servo_target_pose_cmd_.frame_id = msg->header.frame_id;
    tf2::fromMsg(msg->pose, servo_target_pose_cmd_.pose);
    last_servo_msg_time_ = this->now();
    servo_active_ = true;
}

void UrRobotManager::servo_loop_callback() {
    if (!servo_active_) return;
    auto now = this->now();
    // Watchdog timeout (e.g., 200ms without receiving a new target)
    if ((now - last_servo_msg_time_).seconds() > 0.2) {
        servo_active_ = false;
        joint_cmd_rolling_window_.clear();
        RCLCPP_WARN(this->get_logger(), "[Servo] Stream timed out. Halting.");
        return;
    }
    std::lock_guard<std::mutex> lock(servo_mutex_);
    auto servo_params = servo_param_listener_->get_params();
    // Fetch the freshest robot state from the external planning scene
    auto robot_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
    if (!robot_state) return;
    // Calculate inverse kinematics for the next step towards the streamed target pose
    moveit_servo::KinematicState joint_state = servo_->getNextJointState(robot_state, servo_target_pose_cmd_);
    if (servo_->getStatus() != moveit_servo::StatusCode::INVALID) {
        // Append the new state to the rolling window to smooth the trajectory
        moveit_servo::updateSlidingWindow(joint_state, joint_cmd_rolling_window_, servo_params.max_expected_latency, now);
        if (auto msg = moveit_servo::composeTrajectoryMessage(servo_params, joint_cmd_rolling_window_)) {
            trajectory_cmd_pub_->publish(msg.value());
        }
        // Apply the newly calculated state back to the local robot_state pointer to maintain continuity for the next iteration step.
        if (!joint_cmd_rolling_window_.empty()) {
            const auto* jmg = robot_state->getJointModelGroup(servo_params.move_group_name);
            robot_state->setJointGroupPositions(jmg, joint_cmd_rolling_window_.back().positions);
            robot_state->setJointGroupVelocities(jmg, joint_cmd_rolling_window_.back().velocities);
        }
    }
}

}  // namespace ur_robot_manager
