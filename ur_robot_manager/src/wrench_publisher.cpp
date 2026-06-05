#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager
{
  void UrRobotManager::ur_wrench_subscription_callback_(const WrenchStamped::SharedPtr msg) 
  {
    wrench_publisher_->publish(*msg);
  }
}  // namespace ur_robot_manager
