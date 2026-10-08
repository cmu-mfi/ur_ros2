#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager
{
  // --- Wrench Publisher - Setup --- ///
  void UrRobotManager::wrench_publisher_setup() {
    // UR Wrench Subscription
    ur_wrench_subscriber_ = this->create_subscription<WrenchStamped>(
        "force_torque_sensor_broadcaster/wrench_filtered", 
        rclcpp::QoS(10), 
        std::bind(&UrRobotManager::ur_wrench_subscription_callback_, this, std::placeholders::_1)
        );
    // Wrench Publisher
    wrench_publisher_ = this->create_publisher<WrenchStamped>("wrench", rclcpp::QoS(10));
  }

  // --- Wrench Publisher - Subscription Callback --- ///
  void UrRobotManager::ur_wrench_subscription_callback_(const WrenchStamped::SharedPtr msg) 
  {
    wrench_publisher_->publish(*msg);
  }
}  // namespace ur_robot_manager
