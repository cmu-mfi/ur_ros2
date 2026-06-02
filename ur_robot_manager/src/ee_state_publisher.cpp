#include "ur_robot_manager/ee_state_publisher.hpp"
#include <rclcpp/executor.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

EeStatePublisher::EeStatePublisher() : Node("ee_state_publisher", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
  ns_ = this->get_parameter("ns").as_string();
  tf_prefix_ = this->get_parameter("tf_prefix").as_string();
  planning_group_ = tf_prefix_ + "manipulator";
  RCLCPP_INFO(this->get_logger(), "1. Initializing EE State Publisher with namespace: /%s and planning group: %s", ns_.c_str(), planning_group_.c_str());

  // TF setup
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock()); 
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("ee_pose", 10);
  twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("ee_twist", 10);
  accel_pub_ = create_publisher<geometry_msgs::msg::AccelStamped>("ee_accel", 10);
}

void EeStatePublisher::initMoveGroup() {

  RCLCPP_INFO(this->get_logger(), "2. Initializing EE State Publisher with namespace: /%s and planning group: %s", ns_.c_str(), planning_group_.c_str());
  
  moveit::planning_interface::MoveGroupInterface::Options options(planning_group_, "robot_description", "/"+ns_);
  move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), options);  
  joint_model_group_ = move_group_->getRobotModel()->getJointModelGroup(planning_group_);
  ee_link_ = move_group_->getEndEffectorLink();

  if (ee_link_.empty()) {
    const auto &links = joint_model_group_->getLinkModelNames();
    ee_link_ = links.back();
    RCLCPP_WARN(get_logger(), "No EE link configured. Using last link: %s",
                ee_link_.c_str());
  }

  RCLCPP_INFO(get_logger(), "Publishing EE state for link: %s", ee_link_.c_str());    

}

void EeStatePublisher::publishState() {
  auto state = move_group_->getCurrentState();

  if (!state) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Failed to get robot state");
    return;
  }

  const std::string &planning_frame = tf_prefix_ + "base";
  const std::string &root_frame = move_group_->getRobotModel()->getModelFrame();
  
  //----------------------------------------------------
  // Pose - transform from root_frame to planning_frame (base_link)
  //--------------------------------------------------

  // Get transform from root frame to ee_link
  const Eigen::Isometry3d &root_T_ee = state->getGlobalLinkTransform(ee_link_);

  // Create pose message in root frame first
  geometry_msgs::msg::PoseStamped root_pose_msg;
  root_pose_msg.header.stamp = now();
  root_pose_msg.header.frame_id = root_frame;
  root_pose_msg.pose.position.x = root_T_ee.translation().x();
  root_pose_msg.pose.position.y = root_T_ee.translation().y();
  root_pose_msg.pose.position.z = root_T_ee.translation().z();
  Eigen::Quaterniond q(root_T_ee.rotation());
  root_pose_msg.pose.orientation.x = q.x();
  root_pose_msg.pose.orientation.y = q.y();
  root_pose_msg.pose.orientation.z = q.z();
  root_pose_msg.pose.orientation.w = q.w();

  // Transform pose to planning_frame
  geometry_msgs::msg::PoseStamped pose_msg;
  try {
    tf_buffer_->transform(root_pose_msg, pose_msg, planning_frame);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "Could not transform pose to %s: %s", 
                planning_frame.c_str(), ex.what());
    return;
  }

  //----------------------------------------------------
  // Twist - transform from root_frame to planning_frame
  //--------------------------------------------------

  // Get jacobian - velocities are in root frame
  Eigen::MatrixXd jacobian;
  state->getJacobian(joint_model_group_, state->getLinkModel(ee_link_),
                     Eigen::Vector3d::Zero(), jacobian);

  Eigen::VectorXd qdot;
  state->copyJointGroupVelocities(joint_model_group_, qdot);

  Eigen::VectorXd twist_root = jacobian * qdot;

  // Transform twist linear and angular components from root_frame to planning_frame
  geometry_msgs::msg::TwistStamped twist_msg;
  try {
    // Transform linear velocity (vector) from root_frame to planning_frame
    Eigen::Vector3d linear_vec(twist_root(0), twist_root(1), twist_root(2));
    Eigen::Vector3d angular_vec(twist_root(3), twist_root(4), twist_root(5));
    
    geometry_msgs::msg::Vector3Stamped root_linear_stamped, planning_linear_stamped;
    geometry_msgs::msg::Vector3Stamped root_angular_stamped, planning_angular_stamped;
    
    root_linear_stamped.vector.x = linear_vec.x(); 
    root_linear_stamped.vector.y = linear_vec.y(); 
    root_linear_stamped.vector.z = linear_vec.z();
    root_angular_stamped.vector.x = angular_vec.x(); 
    root_angular_stamped.vector.y = angular_vec.y(); 
    root_angular_stamped.vector.z = angular_vec.z();
    
    // Set frame_id and timestamp for transform
    root_linear_stamped.header.stamp = now();
    root_linear_stamped.header.frame_id = root_frame;
    root_angular_stamped.header.stamp = now();
    root_angular_stamped.header.frame_id = root_frame;
    
    tf_buffer_->transform(root_linear_stamped, planning_linear_stamped, planning_frame);
    tf_buffer_->transform(root_angular_stamped, planning_angular_stamped, planning_frame);
    
    twist_msg.header.stamp = now();
    twist_msg.header.frame_id = planning_frame;
    twist_msg.twist.linear.x = planning_linear_stamped.vector.x;
    twist_msg.twist.linear.y = planning_linear_stamped.vector.y;
    twist_msg.twist.linear.z = planning_linear_stamped.vector.z;
    twist_msg.twist.angular.x = planning_angular_stamped.vector.x;
    twist_msg.twist.angular.y = planning_angular_stamped.vector.y;
    twist_msg.twist.angular.z = planning_angular_stamped.vector.z;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "Could not transform twist to %s: %s", 
                planning_frame.c_str(), ex.what());
    return;
  }

  //----------------------------------------------------
  // Acceleration - transform from root_frame to planning_frame
  //--------------------------------------------------

  Eigen::VectorXd qddot;
  try {
    state->copyJointGroupAccelerations(joint_model_group_, qddot);
  } catch (...) {
    qddot = Eigen::VectorXd::Zero(joint_model_group_->getVariableCount());
  }

  Eigen::VectorXd accel_root = jacobian * qddot;

  geometry_msgs::msg::AccelStamped accel_msg;
  try {
    // Transform linear acceleration (vector) from root_frame to planning_frame
    Eigen::Vector3d linear_acc_vec(accel_root(0), accel_root(1), accel_root(2));
    Eigen::Vector3d angular_acc_vec(accel_root(3), accel_root(4), accel_root(5));
    
    geometry_msgs::msg::Vector3Stamped root_linear_acc_stamped, planning_linear_acc_stamped;
    geometry_msgs::msg::Vector3Stamped root_angular_acc_stamped, planning_angular_acc_stamped;
    
    root_linear_acc_stamped.vector.x = linear_acc_vec.x(); 
    root_linear_acc_stamped.vector.y = linear_acc_vec.y(); 
    root_linear_acc_stamped.vector.z = linear_acc_vec.z();
    root_angular_acc_stamped.vector.x = angular_acc_vec.x(); 
    root_angular_acc_stamped.vector.y = angular_acc_vec.y(); 
    root_angular_acc_stamped.vector.z = angular_acc_vec.z();
    
    // Set frame_id and timestamp for transform
    root_linear_acc_stamped.header.stamp = now();
    root_linear_acc_stamped.header.frame_id = root_frame;
    root_angular_acc_stamped.header.stamp = now();
    root_angular_acc_stamped.header.frame_id = root_frame;
    
    tf_buffer_->transform(root_linear_acc_stamped, planning_linear_acc_stamped, planning_frame);
    tf_buffer_->transform(root_angular_acc_stamped, planning_angular_acc_stamped, planning_frame);
    
    accel_msg.header.stamp = now();
    accel_msg.header.frame_id = planning_frame;
    accel_msg.accel.linear.x = planning_linear_acc_stamped.vector.x;
    accel_msg.accel.linear.y = planning_linear_acc_stamped.vector.y;
    accel_msg.accel.linear.z = planning_linear_acc_stamped.vector.z;
    accel_msg.accel.angular.x = planning_angular_acc_stamped.vector.x;
    accel_msg.accel.angular.y = planning_angular_acc_stamped.vector.y;
    accel_msg.accel.angular.z = planning_angular_acc_stamped.vector.z;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "Could not transform accel to %s: %s", 
                planning_frame.c_str(), ex.what());
    return;
  }

  pose_pub_->publish(pose_msg);
  twist_pub_->publish(twist_msg);
  accel_pub_->publish(accel_msg);

  RCLCPP_INFO(get_logger(), "Data published on ee states topics");
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<EeStatePublisher>();
    node->initMoveGroup();
    // node->setupPublisher();    

    rclcpp::WallRate loop_rate(50);
    rclcpp::executors::MultiThreadedExecutor  executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    node->publishState();

    RCLCPP_INFO(node->get_logger(), "Starting publishing on states topics");
    while (rclcpp::ok()) {
      node->publishState();
      loop_rate.sleep();     
    }

    RCLCPP_INFO(node->get_logger(), "End publishing on states topics");
    rclcpp::shutdown();
    return 0;
}
