#include <cmath>
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
    const auto & f = msg->wrench.force;
    const auto & t = msg->wrench.torque;
    // Check if any force or torque component is NaN
    bool has_nan = std::isnan(f.x) || std::isnan(f.y) || std::isnan(f.z) ||
                   std::isnan(t.x) || std::isnan(t.y) || std::isnan(t.z);
    if (has_nan) {
      // Skip saving (and publishing) invalid data
      return; 
    }
    {
      std::lock_guard<std::mutex> lock(wrench_mutex_);
      current_wrench_ = *msg;
    }
    wrench_publisher_->publish(*msg);
  }
}  // namespace ur_robot_manager
