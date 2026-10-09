#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager
{
  // --- Admittance Pose Servo - Setup --- ///
  void UrRobotManager::admittance_pose_servo_setup()
  {
    // Start Admittance Pose Servo Service
    start_admittance_pose_servo_service_ = this->create_service<StartAdmittancePoseServo>(
        "start_admittance_pose_servo",
        std::bind(&UrRobotManager::start_admittance_pose_servo_service_callback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
        service_cb_group_
        );
    // Stop Admittance Pose Servo Service
    stop_admittance_pose_servo_service_ = this->create_service<Trigger>(
        "stop_admittance_pose_servo",
        std::bind(&UrRobotManager::stop_admittance_pose_servo_service_callback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
        service_cb_group_
        );
    // Admittance Loop Timer
    admittance_pose_servo_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(servo_params_.publish_period),
        std::bind(&UrRobotManager::admittance_pose_servo_loop_callback_, this),
        servo_cb_group_
        );
    RCLCPP_INFO(this->get_logger(), "[Admittance Pose Servo] Initialized services and loop timer.");
  }

  // --- Start Admittance Pose Servo - Callback --- ///
  void UrRobotManager::start_admittance_pose_servo_service_callback(
      const std::shared_ptr<StartAdmittancePoseServo::Request> request,
      std::shared_ptr<StartAdmittancePoseServo::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "[Admittance Pose Servo] Received request to start admittance controller.");
    if (admittance_pose_servo_active_) {
      response->success = false;
      response->message = "Admittance Pose Servo already running!";

    } else {
      // Copy parameters into configuration struct
      admittance_pose_servo_config_.target_wrench = request->target_wrench;
      for (size_t i = 0; i < 6; ++i) {
        admittance_pose_servo_config_.mass[i] = request->mass[i];
        admittance_pose_servo_config_.damping[i] = request->damping[i];
        admittance_pose_servo_config_.stiffness[i] = request->stiffness[i];
        admittance_pose_servo_config_.selection_vector[i] = request->selection_vector[i];
        admittance_pose_servo_config_.max_velocity[i] = request->max_velocity[i];
        admittance_pose_servo_config_.max_acceleration[i] = request->max_acceleration[i];
      }
      admittance_pose_servo_previous_time_ = this->now();
      admittance_pose_servo_active_ = true;
      response->success = true;
      response->message = "Admittance pose servo successfully started.";
    }
  }

  // --- Stop Admittance Pose Servo - Callback --- ///
  void UrRobotManager::stop_admittance_pose_servo_service_callback(
      const std::shared_ptr<Trigger::Request> request,
      std::shared_ptr<Trigger::Response> response)
  {
    (void)request;
      RCLCPP_INFO(this->get_logger(), "[Admittance Pose Servo] Received request to stop admittance controller.");
    if (admittance_pose_servo_active_) {
      admittance_pose_servo_active_ = false;
      response->success = true;
      response->message = "Admittance pose servo successfully stopped.";
    } else {
      response->success = false;
      response->message = "Admittance Pose Servo wasn't running!";
    }
  }

  // --- Admittance Pose Servo - Loop Callback --- ///
  void UrRobotManager::admittance_pose_servo_loop_callback_()
  {
    if (!admittance_pose_servo_active_) {
      return;
    }
    // Delta time calculation
    rclcpp::Time now = this->now();
    double dt = (now - admittance_pose_servo_previous_time_).seconds();
    admittance_pose_servo_previous_time_ = now;
    if (dt <= 0.0 || dt > 0.5) {
      RCLCPP_WARN(this->get_logger(), "[Admittance Pose Servo] Time between loop to small or too big!");
      return;
    }
    // Gather current state variables (Pose, Wrench)
    geometry_msgs::msg::PoseStamped current_pose;
    geometry_msgs::msg::PoseStamped previous_pose_1;
    geometry_msgs::msg::PoseStamped previous_pose_2;
    {
      std::lock_guard<std::mutex> lock(ee_state_mutex_);
      current_pose = ee_current_pose_;
      previous_pose_1 = ee_previous_pose_1;
      previous_pose_2 = ee_previous_pose_2;
    }
    geometry_msgs::msg::WrenchStamped current_wrench;
    geometry_msgs::msg::WrenchStamped previous_wrench;
    {
      std::lock_guard<std::mutex> lock(wrench_mutex_);
      current_wrench = current_wrench_;
      previous_wrench = previous_wrench_;
    }

    // Admittance Control Math Algorithm
  }

}  // namespace ur_robot_manager
