#include "ur_robot_manager/ur_robot_manager.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ur_robot_manager {
UrRobotManager::UrRobotManager()
    : Node(
          "ur_robot_manager",
          rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
              true)) {
  ns_ = this->get_parameter("ns").as_string();
  tf_prefix_ = this->get_parameter("tf_prefix").as_string();
  planning_group_ = tf_prefix_ + "manipulator";
  RCLCPP_INFO(
      this->get_logger(),
      "Initilizing Robot Manager with namespace: /%s and planning group: %s",
      ns_.c_str(), planning_group_.c_str());
}

// --- Setup --- ///
void UrRobotManager::setup() {
  moveit::planning_interface::MoveGroupInterface::Options options(
      planning_group_, "robot_description", "/" + ns_);
  move_group_ =
      std::make_unique<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), options);
  planning_scene_interface_ =
      std::make_unique<moveit::planning_interface::PlanningSceneInterface>();

  // default settings
  move_group_->setPlanningTime(5.0);
  move_group_->setNumPlanningAttempts(10);
  move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group_->setPlannerId("PTP");

  // TF setup
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // EE State Publisher setup
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("ee_pose", 10);
  twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("ee_twist", 10);
  accel_pub_ = create_publisher<geometry_msgs::msg::AccelStamped>("ee_accel", 10);

  joint_model_group_ = move_group_->getRobotModel()->getJointModelGroup("manipulator");
  ee_link_ = move_group_->getEndEffectorLink();

  if (ee_link_.empty()) {
    const auto &links = joint_model_group_->getLinkModelNames();
    ee_link_ = links.back();
    RCLCPP_WARN(get_logger(), "No EE link configured. Using last link: %s",
                ee_link_.c_str());
  }

  timer_ = create_wall_timer(std::chrono::milliseconds(20),
                             std::bind(&UrRobotManager::publishState, this));

  RCLCPP_INFO(get_logger(), "Publishing EE state for link: %s", ee_link_.c_str());

  // JointGoal Action Server
  joint_goal_action_server_ = rclcpp_action::create_server<JointGoal>(
      this, "joint_goal",
      std::bind(&UrRobotManager::joint_goal_handle_goal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&UrRobotManager::joint_goal_handle_cancel, this,
                std::placeholders::_1),
      std::bind(&UrRobotManager::joint_goal_handle_accepted, this,
                std::placeholders::_1));
  // PoseGoal Action Server
  pose_goal_action_server_ = rclcpp_action::create_server<PoseGoal>(
      this, "pose_goal",
      std::bind(&UrRobotManager::pose_goal_handle_goal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&UrRobotManager::pose_goal_handle_cancel, this,
                std::placeholders::_1),
      std::bind(&UrRobotManager::pose_goal_handle_accepted, this,
                std::placeholders::_1));
  // Home Service
  service_cb_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  home_service_ = this->create_service<Home>(
      "home",
      std::bind(&UrRobotManager::handle_home_service, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
      service_cb_group_);

  RCLCPP_INFO(this->get_logger(), "Robot Manager is ready!");
}

}; // namespace ur_robot_manager

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ur_robot_manager::UrRobotManager>();

  node->setup();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}