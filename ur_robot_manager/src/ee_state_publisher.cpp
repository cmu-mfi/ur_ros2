#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>

#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager
{

class EndEffectorStatePublisher : public UrRobotManager
{
public:
  EndEffectorStatePublisher()
  : UrRobotManager()
  {
  }
};

}  // namespace ur_robot_manager

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ur_robot_manager::EndEffectorStatePublisher>();

    node->setup();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}