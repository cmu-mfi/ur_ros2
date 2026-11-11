#include "ur_robot_manager/ur_robot_manager.hpp"
#include <algorithm>

namespace ur_robot_manager {

  // --- Readiness - Setup --- ///
  void UrRobotManager::readiness_setup() {
    this->get_parameter_or<std::string>("hardware_component_name", hardware_component_name_, "ur20");
    this->get_parameter_or<std::vector<std::string>>(
        "required_controllers", required_controllers_,
        {"joint_state_broadcaster", "joint_trajectory_controller", "io_and_status_controller", "force_torque_sensor_broadcaster"});
    readiness_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = readiness_cb_group_;
    auto status_qos = rclcpp::QoS(1).reliable().transient_local();
    controller_activity_subscriber_ = this->create_subscription<ControllerManagerActivity>(
        "controller_manager/activity", status_qos,
        std::bind(&UrRobotManager::readiness_activity_callback_, this, std::placeholders::_1), options);
    robot_program_running_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "io_and_status_controller/robot_program_running", status_qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          robot_program_running_ = msg->data;
          readiness_update();
        }, options);
    robot_mode_subscriber_ = this->create_subscription<ur_dashboard_msgs::msg::RobotMode>(
        "io_and_status_controller/robot_mode", status_qos,
        [this](const ur_dashboard_msgs::msg::RobotMode::SharedPtr msg) {
          robot_mode_ = msg->mode;
          readiness_update();
        }, options);
    safety_mode_subscriber_ = this->create_subscription<ur_dashboard_msgs::msg::SafetyMode>(
        "io_and_status_controller/safety_mode", status_qos,
        [this](const ur_dashboard_msgs::msg::SafetyMode::SharedPtr msg) {
          safety_mode_ = msg->mode;
          readiness_update();
        }, options);
  }

  // --- Readiness - Activity Callback --- ///
  void UrRobotManager::readiness_activity_callback_(const ControllerManagerActivity::SharedPtr msg) {
    hardware_ready_ = false;
    for (const auto& component : msg->hardware_components) {
      if (component.name == hardware_component_name_) {
        hardware_ready_ = component.state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
        break;
      }
    }
    controllers_ready_ = !required_controllers_.empty();
    for (const auto& name : required_controllers_) {
      auto controller = std::find_if(msg->controllers.begin(), msg->controllers.end(),
          [&name](const auto& state) { return state.name == name; });
      if (controller == msg->controllers.end() ||
          controller->state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        controllers_ready_ = false;
        break;
      }
    }
    readiness_update();
  }

  // --- Readiness - Update --- ///
  void UrRobotManager::readiness_update() {
    std::string reason;
    if (!hardware_ready_) {
      reason = "Robot hardware is not active or its state is unavailable.";
    } else if (!controllers_ready_) {
      reason = "Required controllers are not active or their state is unavailable.";
    } else if (!robot_program_running_) {
      reason = "UR External Control program is not running.";
    } else if (robot_mode_ != ur_dashboard_msgs::msg::RobotMode::RUNNING) {
      reason = "UR robot mode is not RUNNING.";
    } else if (safety_mode_ != ur_dashboard_msgs::msg::SafetyMode::NORMAL &&
               safety_mode_ != ur_dashboard_msgs::msg::SafetyMode::REDUCED) {
      reason = "UR safety mode does not permit motion.";
    }
    bool readiness_lost;
    {
      std::lock_guard<std::mutex> lock(control_mode_mutex_);
      bool ready = reason.empty();
      readiness_lost = control_mode_ != ControlMode::NOT_READY && !ready;
      if (control_mode_ == ControlMode::NOT_READY && ready) {
        control_mode_ = ControlMode::IDLE;
      }
    }
    if (readiness_lost) {
      RCLCPP_FATAL(this->get_logger(), "[Readiness] %s Exiting robot manager.", reason.c_str());
      rclcpp::shutdown(this->get_node_base_interface()->get_context(), "Robot readiness lost");
      return;
    }
    if (reason != readiness_reason_) {
      if (reason.empty()) {
        RCLCPP_INFO(this->get_logger(), "[Readiness] Robot is ready.");
      } else {
        RCLCPP_WARN(this->get_logger(), "[Readiness] %s", reason.c_str());
      }
      readiness_reason_ = reason;
    }
  }

  // --- Readiness - Command Check --- ///
  bool UrRobotManager::is_robot_ready() {
    std::lock_guard<std::mutex> lock(control_mode_mutex_);
    return control_mode_ != ControlMode::NOT_READY;
  }

}  // namespace ur_robot_manager
