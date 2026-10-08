#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager {

  // --- Servo - Setup --- ///
void UrRobotManager::servo_setup() {
  // 1. Initialize Servo Parameters and Planning Scene Monitor
  servo_param_listener_ = std::make_shared<servo::ParamListener>(shared_from_this(), "moveit_servo");
  servo_params_ = servo_param_listener_->get_params();
  // This automatically hooks into the externally maintained planning scene via ROS 2 topics
  servo_planning_scene_monitor_ = moveit_servo::createPlanningSceneMonitor(shared_from_this(), servo_params_);
  // 2. Initialize the Servo object
  servo_ = std::make_unique<moveit_servo::Servo>(shared_from_this(), servo_param_listener_, servo_planning_scene_monitor_);
  servo_->setCommandType(moveit_servo::CommandType::POSE);
  // 3. Setup Publisher & Subscriber
  servo_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      servo_params_.command_out_topic, rclcpp::SystemDefaultsQoS());
  pose_servo_subscriber_ = this->create_subscription<PoseServo>(
      "pose_servo", rclcpp::SystemDefaultsQoS(),
      std::bind(&UrRobotManager::pose_servo_subscription_callback_, this, std::placeholders::_1));
  // 4. Setup 500Hz Timer for Real-Time Execution
  pose_servo_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(servo_params_.publish_period),
      std::bind(&UrRobotManager::pose_servo_loop_callback_, this),
      servo_cb_group_);
}


}  // namespace ur_robot_manager
