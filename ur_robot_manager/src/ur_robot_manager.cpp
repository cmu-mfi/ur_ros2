#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager
{
  UrRobotManager::UrRobotManager() : Node("ur_robot_manager", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
    ns_ = this->get_parameter("ns").as_string();
    tf_prefix_ = this->get_parameter("tf_prefix").as_string();
    moveit_planning_group_ = tf_prefix_ + "manipulator"; 
    RCLCPP_INFO(this->get_logger(), "Initilizing UR Robot Manager with namespace: /%s and planning group: %s", ns_.c_str(), moveit_planning_group_.c_str());
  }

  // --- Setup --- ///
  void UrRobotManager::setup() {
    RCLCPP_INFO(this->get_logger(), "Starting UrRobotManager setup...");

    // ROS2 Setup
    RCLCPP_INFO(this->get_logger(), "[Setup] Configuring callback groups...");
    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    servo_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    ee_state_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // TF setup
    RCLCPP_INFO(this->get_logger(), "[Setup] Initializing TF buffer and listener...");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock()); 
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Setup Moveit
    RCLCPP_INFO(this->get_logger(), "[Setup] Setting up MoveIt...");
    moveit_setup();

    // Setup Servo
    RCLCPP_INFO(this->get_logger(), "[Setup] Setting up MoveIt Servo...");
    servo_setup();

    // Joint Goal Setup
    RCLCPP_INFO(this->get_logger(), "[Setup] Setting up Joint Goal action server...");
    joint_goal_setup();

    // Pose Goal Setup
    RCLCPP_INFO(this->get_logger(), "[Setup] Setting up Pose Goal action server...");
    pose_goal_setup();

    // Home Service Setup
    RCLCPP_INFO(this->get_logger(), "[Setup] Setting up Home service...");
    home_service_setup();

    // Park Service Setup
    RCLCPP_INFO(this->get_logger(), "[Setup] Setting up Park service...");
    park_service_setup();

    // Set Payload Service Setup
    RCLCPP_INFO(this->get_logger(), "[Setup] Setting up Set Payload service...");
    set_payload_service_setup();

    // Set IO Service Setup
    RCLCPP_INFO(this->get_logger(), "[Setup] Setting up Set IO service...");
    set_io_service_setup();

    // Wrench Publisher Setup
    RCLCPP_INFO(this->get_logger(), "[Setup] Setting up Wrench publisher...");
    wrench_publisher_setup();

    // EE State Publisher Setup
    RCLCPP_INFO(this->get_logger(), "[Setup] Setting up EE State publisher...");
    ee_state_publisher_setup();

    // Admittance Pose Servo Setup
    RCLCPP_INFO(this->get_logger(), "[Setup] Setting up Admittance Pose Servo...");
    admittance_pose_servo_setup();

    RCLCPP_INFO(this->get_logger(), "UR Robot Manager is ready!");
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
