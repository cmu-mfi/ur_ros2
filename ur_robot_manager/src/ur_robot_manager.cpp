#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager
{
  UrRobotManager::UrRobotManager() : Node("ur_robot_manager", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
    ns_ = this->get_parameter("ns").as_string();
    tf_prefix_ = this->get_parameter("tf_prefix").as_string();
    moveit_planning_group_ = tf_prefix_ + "manipulator"; 
    RCLCPP_INFO(this->get_logger(), "Initilizing Robot Manager with namespace: /%s and planning group: %s", ns_.c_str(), moveit_planning_group_.c_str());
  }

  // --- Setup --- ///
  void UrRobotManager::setup() {
    // ROS2 Setup
    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    servo_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // TF setup
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock()); 
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // Setup Moveit
    moveit_setup();
    // Setup Servo
    servo_setup();
    // Joint Goal Setup
    joint_goal_setup();
    // Pose Goal Setup
    pose_goal_setup();
    // Home Service Setup
    home_service_setup();
    // Park Service Setup
    park_service_setup();
    // Set Payload Service Setup
    set_payload_service_setup();
    // Set IO Service Setup
    set_io_service_setup();
    // Wrench Publisher Setup
    wrench_publisher_setup();
    // EE State Publisher Setup
    ee_state_publisher_setup();

    RCLCPP_INFO(this->get_logger(), "Robot Manager is ready!");
  }

}  // namespace ur_robot_manager

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ur_robot_manager::UrRobotManager>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread setup_thread([&node]() {
        node->setup();
    });

    executor.spin();

    rclcpp::shutdown();
    
    if (setup_thread.joinable()) {
        setup_thread.join();
    }
    
    return 0;
}
