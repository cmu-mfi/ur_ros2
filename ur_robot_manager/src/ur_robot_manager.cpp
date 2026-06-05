#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager
{
  UrRobotManager::UrRobotManager() : Node("ur_robot_manager", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
    ns_ = this->get_parameter("ns").as_string();
    tf_prefix_ = this->get_parameter("tf_prefix").as_string();
    planning_group_ = tf_prefix_ + "manipulator"; 
    RCLCPP_INFO(this->get_logger(), "Initilizing Robot Manager with namespace: /%s and planning group: %s", ns_.c_str(), planning_group_.c_str());
  }

  // --- Setup --- ///
  void UrRobotManager::setup() {
    moveit::planning_interface::MoveGroupInterface::Options options(planning_group_, "robot_description", "/"+ns_);
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), options);
    planning_scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();

    // default settings
    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_->setPlannerId("PTP");

    // TF setup
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock()); 
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // JointGoal Action Server
    joint_goal_action_server_ = rclcpp_action::create_server<JointGoal>(
        this,
        "joint_goal",
        std::bind(&UrRobotManager::joint_goal_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&UrRobotManager::joint_goal_handle_cancel, this, std::placeholders::_1),
        std::bind(&UrRobotManager::joint_goal_handle_accepted, this, std::placeholders::_1)
        );
    // PoseGoal Action Server
    pose_goal_action_server_ = rclcpp_action::create_server<PoseGoal>(
        this,
        "pose_goal",
        std::bind(&UrRobotManager::pose_goal_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&UrRobotManager::pose_goal_handle_cancel, this, std::placeholders::_1),
        std::bind(&UrRobotManager::pose_goal_handle_accepted, this, std::placeholders::_1)
        );
    // Home Service
    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    home_service_ = this->create_service<Home>(
        "home",
        std::bind(&UrRobotManager::home_service_callback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
        service_cb_group_
        );
    // Set Payload Service
    set_payload_service_ = this->create_service<SetPayload>(
        "set_payload",
        std::bind(&UrRobotManager::set_payload_service_callback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
        service_cb_group_
        );
    ur_set_payload_client_ = this->create_client<UrSetPayload>("io_and_status_controller/set_payload");
    while (!ur_set_payload_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for service: io_and_status_controller/set_payload");
    }

    // --- FT Publisher ---
    ur_wrench_subscriber_ = this->create_subscription<WrenchStamped>(
        "force_torque_sensor_broadcaster/wrench", 
        rclcpp::QoS(10), 
        std::bind(&UrRobotManager::ur_wrench_subscription_callback_, this, std::placeholders::_1)
        );
    wrench_publisher_ = this->create_publisher<WrenchStamped>("wrench", rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(), "Robot Manager is ready!");
  }

}  // namespace ur_robot_manager

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ur_robot_manager::UrRobotManager>();

    node->setup();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

