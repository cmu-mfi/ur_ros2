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
    // Admittance Pose Servo Subscriber
    admittance_pose_servo_subscriber_ = this->create_subscription<PoseServo>(
        "admittance_pose_servo",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&UrRobotManager::admittance_pose_servo_subscription_callback_, this, std::placeholders::_1)
        );
    // Admittance Loop Timer
    admittance_pose_servo_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(servo_params_.publish_period),
        std::bind(&UrRobotManager::admittance_pose_servo_loop_callback_, this),
        servo_cb_group_
        );
    RCLCPP_INFO(this->get_logger(), "[Admittance Pose Servo] Initialized services, subscriber and loop timer.");
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
        admittance_pose_servo_config_.selection_vector[i] = request->selection_vector[i];
        admittance_pose_servo_config_.mass[i] = request->mass[i];
        admittance_pose_servo_config_.damping[i] = request->damping[i];
        admittance_pose_servo_config_.stiffness[i] = request->stiffness[i];
        admittance_pose_servo_config_.max_velocity[i] = request->max_velocity[i];
      }
      {
        std::lock_guard<std::mutex> ee_lock(ee_state_mutex_);
        std::lock_guard<std::mutex> admittance_lock(admittance_pose_servo_mutex_);
        admittance_pose_servo_desired_pose_ = ee_current_pose_.pose;
      }
      admittance_pose_servo_previous_time_ = this->now();
      // Reset filter history states for a clean start
      admittance_x_k_1_.fill(0.0);
      admittance_x_k_2_.fill(0.0);
      admittance_f_k_1_.fill(0.0);
      admittance_f_k_2_.fill(0.0);
      // Tare the force sensor by calculating the mean offset over 50 samples
      RCLCPP_INFO(this->get_logger(), "[Admittance Pose Servo] Taring force/torque sensor...");
      std::array<double, 6> offset_sum = {0.0};
      const int sample_count = 50;
      rclcpp::Rate tare_rate(500); // Sample at 500 Hz for 0.1 seconds overall
      for (int i = 0; i < sample_count; ++i) {
        geometry_msgs::msg::WrenchStamped temp_wrench;
        {
          std::lock_guard<std::mutex> lock(wrench_mutex_);
          temp_wrench = current_wrench_;
        }
        offset_sum[0] += temp_wrench.wrench.force.y;
        offset_sum[1] += temp_wrench.wrench.force.x;
        offset_sum[2] += temp_wrench.wrench.force.z;
        offset_sum[3] += temp_wrench.wrench.torque.y;
        offset_sum[4] += temp_wrench.wrench.torque.x;
        offset_sum[5] += temp_wrench.wrench.torque.z;
        
        tare_rate.sleep();
      }
      for (size_t i = 0; i < 6; ++i) {
        admittance_wrench_offset_[i] = offset_sum[i] / sample_count;
      }
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

  // --- Admittance Pose Servo - Subscription Callback --- ///
  void UrRobotManager::admittance_pose_servo_subscription_callback_(const PoseServo::SharedPtr msg)
  {
    std::string reference_frame = msg->frame_id.empty() ? (moveit_move_group_ ? moveit_move_group_->getPlanningFrame() : tf_prefix_ + "base_link") : msg->frame_id;
    geometry_msgs::msg::Pose tool0_pose;
    try {
      tool0_pose = transform_pose_to_tool0_frame(msg->target_pose, msg->target_id);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "[Admittance Pose Servo] TF Verification Failed (tool0): %s", ex.what());
      return;
    }

    geometry_msgs::msg::Pose base_link_tool0_pose;
    try {
      base_link_tool0_pose = transform_pose_to_base_link_frame(tool0_pose, reference_frame);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "[Admittance Pose Servo] TF Verification Failed (base_link): %s", ex.what());
      return;
    }

    std::lock_guard<std::mutex> lock(admittance_pose_servo_mutex_);
    admittance_pose_servo_desired_pose_ = base_link_tool0_pose;
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
    {
      std::lock_guard<std::mutex> lock(ee_state_mutex_);
      current_pose = ee_current_pose_;
    }
    geometry_msgs::msg::WrenchStamped current_wrench;
    {
      std::lock_guard<std::mutex> lock(wrench_mutex_);
      current_wrench = current_wrench_;
    }
    geometry_msgs::msg::Pose desired_pose;
    {
      std::lock_guard<std::mutex> lock(admittance_pose_servo_mutex_);
      desired_pose = admittance_pose_servo_desired_pose_;
    }
    // Admittance Control Math Algorithm
    std::array<double, 6> f_reading = {
      -(current_wrench.wrench.force.y - admittance_wrench_offset_[0]),
      -(current_wrench.wrench.force.x - admittance_wrench_offset_[1]),
      -(current_wrench.wrench.force.z - admittance_wrench_offset_[2]),
      -(current_wrench.wrench.torque.y - admittance_wrench_offset_[3]),
      -(current_wrench.wrench.torque.x - admittance_wrench_offset_[4]),
      -(current_wrench.wrench.torque.z - admittance_wrench_offset_[5])
    };
    std::array<double, 6> f_contact = {
      admittance_pose_servo_config_.target_wrench.force.x,
      admittance_pose_servo_config_.target_wrench.force.y,
      admittance_pose_servo_config_.target_wrench.force.z,
      admittance_pose_servo_config_.target_wrench.torque.x,
      admittance_pose_servo_config_.target_wrench.torque.y,
      admittance_pose_servo_config_.target_wrench.torque.z
    };
    std::array<double, 6> x_k_0 = {0.0};
    // Loop over all dimensions
    for (size_t i = 0; i < 6; ++i) {
      // Check selection vector
      if (!admittance_pose_servo_config_.selection_vector[i]) {
        x_k_0[i] = 0.0;
        admittance_f_k_1_[i] = 0.0;
        admittance_f_k_2_[i] = 0.0;
        admittance_x_k_1_[i] = 0.0;
        admittance_x_k_2_[i] = 0.0;
        continue;
      }
      // Compute Force Error (Deadband to respect target_wrench)
      double f_k_0 = 0.0;
      f_k_0 = f_reading[i] - f_contact[i];
      // Extract constants for readability
      double M = admittance_pose_servo_config_.mass[i];
      double dt2 = dt * dt;
      // Calculate adaptive damping and stiffness
      double B = admittance_pose_servo_config_.damping[i];
      double K = admittance_pose_servo_config_.stiffness[i];
      // Compute Discrete Transfer Function Output
      double num = dt2 * (f_k_0 + 2.0 * admittance_f_k_1_[i] + admittance_f_k_2_[i])
                   - (2.0 * K * dt2 - 8.0 * M) * admittance_x_k_1_[i]
                   - (4.0 * M - 2.0 * B * dt + K * dt2) * admittance_x_k_2_[i];
      double den = 4.0 * M + 2.0 * B * dt + K * dt2;
      x_k_0[i] = num / den;
      // Calculate previous and current unconstrained velocities
      double current_vel = (x_k_0[i] - admittance_x_k_1_[i]) / dt;
      //  Velocity constraint
      if (std::abs(current_vel) > admittance_pose_servo_config_.max_velocity[i] && admittance_pose_servo_config_.max_velocity[i] != 0.0) {
        current_vel = std::copysign(admittance_pose_servo_config_.max_velocity[i], current_vel);
      }
      // Recompute the actual constrained displacement
      x_k_0[i] = admittance_x_k_1_[i] + current_vel * dt;
      // Shift history for the next iteration
      admittance_x_k_2_[i] = admittance_x_k_1_[i];
      admittance_x_k_1_[i] = x_k_0[i];
      admittance_f_k_2_[i] = admittance_f_k_1_[i];
      admittance_f_k_1_[i] = f_k_0;
    }
    geometry_msgs::msg::Pose commanded_pose = desired_pose;
    // Apply linear deviation directly
    commanded_pose.position.x += x_k_0[0];
    commanded_pose.position.y += x_k_0[1];
    commanded_pose.position.z += x_k_0[2];
    // Apply angular deviation using TF2 Quaternions
    tf2::Quaternion q_desired, q_deviation;
    tf2::fromMsg(desired_pose.orientation, q_desired);
    q_deviation.setRPY(x_k_0[3], x_k_0[4], x_k_0[5]);
    tf2::Quaternion q_commanded = q_deviation * q_desired;
    q_commanded.normalize();
    commanded_pose.orientation = tf2::toMsg(q_commanded);
    // Pass desired pose directly to pose_servo subscriber internally
    auto pose_servo_msg = std::make_shared<PoseServo>();
    pose_servo_msg->frame_id = tf_prefix_ + "base_link";
    pose_servo_msg->target_id = tf_prefix_ + "tool0";
    pose_servo_msg->target_pose = commanded_pose;
    pose_servo_subscription_callback_(pose_servo_msg);
  }

  // --- Sigmoid Helper Function ///
  double UrRobotManager::sigmoid(double x, double x_start, double x_end, double y_start, double y_end, double k) {
    if (x_start == x_end) {
      return (x < x_start) ? y_start : y_end;
    }
    // Normalize x into normalized progress t in range [0, 1]
    double t = (x - x_start) / (x_end - x_start);
    // Map progress [0, 1] to symmetric logistic range [-k, +k]
    double z = (t - 0.5) * (2.0 * k);
    // Compute standard logistic sigmoid: 1 / (1 + e^-z)
    double sig = 1.0 / (1.0 + std::exp(-z));
    // Map output range from [0, 1] to [y_start, y_end]
    return y_start + (y_end - y_start) * sig;
  }

}  // namespace ur_robot_manager
