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
    home_service_ = this->create_service<std_srvs::srv::Trigger>(
        "home",
        std::bind(&UrRobotManager::handle_home_service, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
        service_cb_group_
        );

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

