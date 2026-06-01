#include "ur_robot_manager/ur_robot_manager.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ur_robot_manager
{
  UrRobotManager::UrRobotManager() : Node("ur_robot_manager", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
    ns_ = this->get_parameter("ns").as_string();
    tf_prefix_ = this->get_parameter("tf_prefix").as_string();
    planning_group_ = tf_prefix_ + "manipulator"; 
    RCLCPP_INFO(this->get_logger(), "Initilizing Robot Manager with namespace: /%s and planning group: %s", ns_.c_str(), planning_group_.c_str());
  }

  // --- EE State Publisher Setup --- ///
  void UrRobotManager::setupEeStatePublisher() {
    pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(
        "ee_pose", 10);

    twist_pub_ =
      create_publisher<geometry_msgs::msg::TwistStamped>(
        "ee_twist", 10);

    accel_pub_ =
      create_publisher<geometry_msgs::msg::AccelStamped>(
        "ee_accel", 10);

    joint_model_group_ =
      move_group_->getRobotModel()->getJointModelGroup("manipulator");

    ee_link_ = move_group_->getEndEffectorLink();

    if (ee_link_.empty())
    {
      const auto &links =
        joint_model_group_->getLinkModelNames();

      ee_link_ = links.back();

      RCLCPP_WARN(
        get_logger(),
        "No EE link configured. Using last link: %s",
        ee_link_.c_str());
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(
        &UrRobotManager::publishState,
        this));

    RCLCPP_INFO(
      get_logger(),
      "Publishing EE state for link: %s",
      ee_link_.c_str());
  }

  void UrRobotManager::publishState()
  {
    auto state =
      move_group_->getCurrentState(0.1);

    if (!state)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Failed to get robot state");
      return;
    }

    //----------------------------------------------------
    // Pose
    //--------------------------------------------------

    const Eigen::Isometry3d &tf =
      state->getGlobalLinkTransform(ee_link_);

    geometry_msgs::msg::PoseStamped pose_msg;

    pose_msg.header.stamp = now();
    pose_msg.header.frame_id =
      move_group_->getPlanningFrame();

    pose_msg.pose.position.x =
      tf.translation().x();
    pose_msg.pose.position.y =
      tf.translation().y();
    pose_msg.pose.position.z =
      tf.translation().z();

    Eigen::Quaterniond q(tf.rotation());

    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    //----------------------------------------------------
    // Jacobian
    //--------------------------------------------------

    Eigen::MatrixXd jacobian;

    state->getJacobian(
      joint_model_group_,
      state->getLinkModel(ee_link_),
      Eigen::Vector3d::Zero(),
      jacobian);

    //----------------------------------------------------
    // Joint velocities
    //--------------------------------------------------

    Eigen::VectorXd qdot;
    state->copyJointGroupVelocities(
      joint_model_group_,
      qdot);

    Eigen::VectorXd twist =
      jacobian * qdot;

    geometry_msgs::msg::TwistStamped twist_msg;

    twist_msg.header = pose_msg.header;

    if (twist.size() >= 6)
    {
      twist_msg.twist.linear.x = twist(0);
      twist_msg.twist.linear.y = twist(1);
      twist_msg.twist.linear.z = twist(2);

      twist_msg.twist.angular.x = twist(3);
      twist_msg.twist.angular.y = twist(4);
      twist_msg.twist.angular.z = twist(5);
    }

    //----------------------------------------------------
    // Joint accelerations
    //--------------------------------------------------

    Eigen::VectorXd qddot;

    try
    {
      state->copyJointGroupAccelerations(
        joint_model_group_,
        qddot);
    }
    catch (...)
    {
      qddot =
        Eigen::VectorXd::Zero(
          joint_model_group_->getVariableCount());
    }

    Eigen::VectorXd accel =
      jacobian * qddot;

    geometry_msgs::msg::AccelStamped accel_msg;

    accel_msg.header = pose_msg.header;

    if (accel.size() >= 6)
    {
      accel_msg.accel.linear.x = accel(0);
      accel_msg.accel.linear.y = accel(1);
      accel_msg.accel.linear.z = accel(2);

      accel_msg.accel.angular.x = accel(3);
      accel_msg.accel.angular.y = accel(4);
      accel_msg.accel.angular.z = accel(5);
    }

    pose_pub_->publish(pose_msg);
    twist_pub_->publish(twist_msg);
    accel_pub_->publish(accel_msg);
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

    // EE State Publisher setup
    setupEeStatePublisher();

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