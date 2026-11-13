#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager {
  namespace {
    std::array<double, 6> wrench_components(const geometry_msgs::msg::Wrench & wrench) {
      return {wrench.force.x, wrench.force.y, wrench.force.z,
              wrench.torque.x, wrench.torque.y, wrench.torque.z};
    }

    geometry_msgs::msg::Pose apply_admittance_displacement(
        const geometry_msgs::msg::Pose & desired_pose,
        const std::array<double, 6> & displacement,
        const tf2::Quaternion & base_ft_rotation)
    {
      auto commanded_pose = desired_pose;
      const auto linear_displacement = tf2::quatRotate(
          base_ft_rotation, tf2::Vector3(displacement[0], displacement[1], displacement[2]));
      // Reverse the applied admittance displacement when using tool0 axes.
      commanded_pose.position.x -= linear_displacement.x();
      commanded_pose.position.y -= linear_displacement.y();
      commanded_pose.position.z -= linear_displacement.z();

      // Apply the exact inverse rotation about tool0's origin and moving axes.
      tf2::Quaternion desired_rotation, angular_displacement;
      tf2::fromMsg(desired_pose.orientation, desired_rotation);
      angular_displacement.setRPY(displacement[3], displacement[4], displacement[5]);
      auto commanded_rotation = base_ft_rotation * angular_displacement.inverse() *
          base_ft_rotation.inverse() * desired_rotation;
      commanded_rotation.normalize();
      commanded_pose.orientation = tf2::toMsg(commanded_rotation);
      return commanded_pose;
    }
  }  // namespace

  // --- Admittance Pose Servo - Setup --- ///
  void UrRobotManager::admittance_pose_servo_setup() {
    start_admittance_pose_servo_service_ = this->create_service<StartAdmittancePoseServo>(
        "start_admittance_pose_servo",
        std::bind(&UrRobotManager::start_admittance_pose_servo_service_callback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
        service_cb_group_
        );
    stop_admittance_pose_servo_service_ = this->create_service<Trigger>(
        "stop_admittance_pose_servo",
        std::bind(&UrRobotManager::stop_admittance_pose_servo_service_callback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
        service_cb_group_
        );
    admittance_pose_servo_subscriber_ = this->create_subscription<PoseServo>(
        "admittance_pose_servo",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&UrRobotManager::admittance_pose_servo_subscription_callback_, this, std::placeholders::_1)
        );
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
    if (!is_robot_ready()) {
      response->success = false;
      response->message = "Robot is not ready.";
      return;
    }
    RCLCPP_INFO(this->get_logger(), "[Admittance Pose Servo] Received request to start admittance controller.");
    if (admittance_pose_servo_active_) {
      response->success = false;
      response->message = "Admittance Pose Servo already running!";
      return;
    }
    {
      std::lock_guard<std::mutex> lock(control_mode_mutex_);
      if (control_mode_ != ControlMode::IDLE) {
        RCLCPP_ERROR(this->get_logger(),
            "[Admittance Pose Servo] Robot is already being controlled (current mode: %s).",
            controlModeToString(control_mode_));
        response->success = false;
        response->message = "Robot is already being controlled.";
        return;
      }
      control_mode_ = ControlMode::ADMITTANCE_POSE_SERVO;
    }

    initialize_admittance_pose_servo(*request);
    try {
      // Require the moving FT frame before enabling admittance.
      tf_buffer_->lookupTransform(tf_prefix_ + "base_link", tf_prefix_ + "tool0", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      std::lock_guard<std::mutex> lock(control_mode_mutex_);
      if (control_mode_ == ControlMode::ADMITTANCE_POSE_SERVO) {
        control_mode_ = ControlMode::IDLE;
      }
      response->success = false;
      response->message = std::string("Could not initialize admittance in FT frame: ") + ex.what();
      return;
    }

    tare_admittance_sensor();
    admittance_pose_servo_active_ = true;
    response->success = true;
    response->message = "Admittance pose servo successfully started.";
  }

  void UrRobotManager::initialize_admittance_pose_servo(const StartAdmittancePoseServo::Request & request) {
    admittance_pose_servo_config_ = {
      request.target_wrench, request.selection_vector, request.mass,
      request.damping, request.stiffness, request.max_velocity
    };
    {
      std::lock_guard<std::mutex> ee_lock(ee_state_mutex_);
      std::lock_guard<std::mutex> admittance_lock(admittance_pose_servo_mutex_);
      admittance_pose_servo_desired_pose_ = ee_current_pose_.pose;
    }
    admittance_pose_servo_previous_time_ = this->now();
    admittance_axis_states_ = {};
  }

  std::array<double, 6> UrRobotManager::read_admittance_sensor_wrench() {
    std::lock_guard<std::mutex> lock(wrench_mutex_);
    return wrench_components(current_wrench_.wrench);
  }

  void UrRobotManager::tare_admittance_sensor() {
    RCLCPP_INFO(this->get_logger(), "[Admittance Pose Servo] Taring force/torque sensor...");
    constexpr int sample_count = 50;
    rclcpp::Rate tare_rate(500);  // 50 samples at 500 Hz span 0.1 seconds.
    std::array<double, 6> offset_sum{};
    // The published sensor wrench already uses FT-frame axes.
    for (int sample = 0; sample < sample_count; ++sample) {
      const auto sensor_wrench = read_admittance_sensor_wrench();
      for (size_t axis = 0; axis < offset_sum.size(); ++axis) {
        offset_sum[axis] += sensor_wrench[axis];
      }
      tare_rate.sleep();
    }
    for (size_t axis = 0; axis < offset_sum.size(); ++axis) {
      admittance_wrench_offset_[axis] = offset_sum[axis] / sample_count;
    }
  }

  // --- Stop Admittance Pose Servo - Callback --- ///
  void UrRobotManager::stop_admittance_pose_servo_service_callback(
      const std::shared_ptr<Trigger::Request>,
      std::shared_ptr<Trigger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "[Admittance Pose Servo] Received request to stop admittance controller.");
    if (!admittance_pose_servo_active_) {
      response->success = false;
      response->message = "Admittance Pose Servo wasn't running!";
      return;
    }
    admittance_pose_servo_active_ = false;
    {
      std::lock_guard<std::mutex> lock(control_mode_mutex_);
      if (control_mode_ == ControlMode::ADMITTANCE_POSE_SERVO) {
        control_mode_ = ControlMode::IDLE;
      }
    }
    response->success = true;
    response->message = "Admittance pose servo successfully stopped.";
  }

  // --- Admittance Pose Servo - Subscription Callback --- ///
  void UrRobotManager::admittance_pose_servo_subscription_callback_(const PoseServo::SharedPtr msg)
  {
    if (!is_robot_ready()) return;
    if (!admittance_pose_servo_active_) {
      return;
    }
    const std::string reference_frame = msg->frame_id.empty() ?
        (moveit_move_group_ ? moveit_move_group_->getPlanningFrame() : tf_prefix_ + "base_link") :
        msg->frame_id;
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
    const auto now = this->now();
    const double dt = (now - admittance_pose_servo_previous_time_).seconds();
    admittance_pose_servo_previous_time_ = now;
    if (dt <= 0.0 || dt > 0.5) {
      RCLCPP_WARN(this->get_logger(), "[Admittance Pose Servo] Loop interval must be in (0, 0.5] seconds.");
      return;
    }
    const auto sensor_wrench = read_admittance_sensor_wrench();
    geometry_msgs::msg::Pose desired_pose;
    {
      std::lock_guard<std::mutex> lock(admittance_pose_servo_mutex_);
      desired_pose = admittance_pose_servo_desired_pose_;
    }
    tf2::Quaternion base_ft_rotation;
    try {
      const auto base_ft = tf_buffer_->lookupTransform(
          tf_prefix_ + "base_link", tf_prefix_ + "tool0", tf2::TimePointZero);
      tf2::fromMsg(base_ft.transform.rotation, base_ft_rotation);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "[Admittance Pose Servo] FT transform failed: %s", ex.what());
      return;
    }

    const auto displacement = calculate_admittance_displacement(sensor_wrench, dt);
    auto pose_servo_msg = std::make_shared<PoseServo>();
    pose_servo_msg->frame_id = tf_prefix_ + "base_link";
    pose_servo_msg->target_id = tf_prefix_ + "tool0";
    pose_servo_msg->target_pose = apply_admittance_displacement(desired_pose, displacement, base_ft_rotation);
    pose_servo_subscription_callback_(pose_servo_msg);
  }

  std::array<double, 6> UrRobotManager::calculate_admittance_displacement(
      const std::array<double, 6> & sensor_wrench, double dt)
  {
    const auto & config = admittance_pose_servo_config_;
    const auto target_wrench = wrench_components(config.target_wrench);
    const double dt_squared = dt * dt;
    std::array<double, 6> displacement{};

    // Wrench errors, filter history and velocity limits all use moving FT-frame axes.
    for (size_t axis = 0; axis < displacement.size(); ++axis) {
      auto & state = admittance_axis_states_[axis];
      if (!config.selection_vector[axis]) {
        state = {};
        continue;
      }
      const double contact_wrench = -(sensor_wrench[axis] - admittance_wrench_offset_[axis]);
      const double wrench_error = contact_wrench - target_wrench[axis];
      const double mass = config.mass[axis];
      const double damping = config.damping[axis];
      const double stiffness = config.stiffness[axis];

      // Bilinear discretization of M*x'' + B*x' + K*x = wrench_error.
      // State holds the two preceding samples (k-1 and k-2).
      const double numerator =
          dt_squared * (wrench_error + 2.0 * state.wrench_error + state.previous_wrench_error)
          - (2.0 * stiffness * dt_squared - 8.0 * mass) * state.displacement
          - (4.0 * mass - 2.0 * damping * dt + stiffness * dt_squared) * state.previous_displacement;
      const double denominator = 4.0 * mass + 2.0 * damping * dt + stiffness * dt_squared;
      const double unconstrained_displacement = numerator / denominator;

      double velocity = (unconstrained_displacement - state.displacement) / dt;
      const double max_velocity = config.max_velocity[axis];
      if (max_velocity != 0.0 && std::abs(velocity) > max_velocity) {
        velocity = std::copysign(max_velocity, velocity);
      }
      displacement[axis] = state.displacement + velocity * dt;

      state.previous_displacement = state.displacement;
      state.displacement = displacement[axis];
      state.previous_wrench_error = state.wrench_error;
      state.wrench_error = wrench_error;
    }
    return displacement;
  }

}  // namespace ur_robot_manager
