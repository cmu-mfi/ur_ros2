#include "ur_robot_manager/ee_state_publisher.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <rclcpp/executor.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <geometry_msgs/msg/vector3_stamped.hpp>

EeStatePublisher::EeStatePublisher() : Node("ee_state_publisher", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
  ns_ = this->get_parameter("ns").as_string();
  tf_prefix_ = this->get_parameter("tf_prefix").as_string();
  planning_group_ = tf_prefix_ + "manipulator";
  RCLCPP_INFO(this->get_logger(), "Initializing EE State Publisher with namespace: /%s and planning group: %s", ns_.c_str(), planning_group_.c_str());

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("ee_pose", 10);
  twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("ee_twist", 10);
  accel_pub_ = create_publisher<geometry_msgs::msg::AccelStamped>("ee_accel", 10);
}

void EeStatePublisher::setup() {

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

  move_group_->startStateMonitor();
  move_group_->setPoseReferenceFrame(tf_prefix_+"base");

  // Wait until we can successfully fetch the current pose
  RCLCPP_INFO(get_logger(), "Waiting for current robot state...");
  while (rclcpp::ok()) {
      try {
          // This will log an error internally if it fails, but we catch/retry
          last_pose_ = move_group_->getCurrentPose(tf_prefix_+"tool0");
          
          // Check if we got valid data (if orientation w is 0, it's usually an uninitialized empty pose)
          if (last_pose_.pose.orientation.w != 0.0) {
              break; 
          }
      } catch (...) {
          // Ignore exceptions and keep trying
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(get_logger(), "Publishing EE state for link: %s", ee_link_.c_str());    
}

void EeStatePublisher::publishState() {
  //----------------------------------------------------
  // Pose - transform from root_frame to planning_frame (base_link)
  //--------------------------------------------------

  geometry_msgs::msg::PoseStamped pose_msg = move_group_->getCurrentPose(tf_prefix_+"tool0");
  // THE FIX:
  rclcpp::Time t_curr(pose_msg.header.stamp);
  rclcpp::Time t_last(last_pose_.header.stamp);
  double dt = (t_curr - t_last).seconds();

  if (dt <= 0.0) {
    // Avoid division by zero if messages arrive with the exact same timestamp
    return; 
  }

  //----------------------------------------------------
  // Twist - transform from root_frame to planning_frame
  //--------------------------------------------------

  // Transform twist linear and angular components from root_frame to planning_frame
  geometry_msgs::msg::TwistStamped twist_msg;    
  twist_msg.header = pose_msg.header;

  Eigen::Vector3d linear_velocity, angular_velocity;
  tf2::Quaternion q_last(last_pose_.pose.orientation.x, last_pose_.pose.orientation.y, last_pose_.pose.orientation.z, last_pose_.pose.orientation.w);
  tf2::Quaternion q_curr(pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);

  // Linear velocity
  linear_velocity(0) = (pose_msg.pose.position.x - last_pose_.pose.position.x) / dt;
  linear_velocity(1) = (pose_msg.pose.position.y - last_pose_.pose.position.y) / dt;
  linear_velocity(2) = (pose_msg.pose.position.z - last_pose_.pose.position.z) / dt;

  // Angular velocity
  tf2::Quaternion q_diff = q_curr * q_last.inverse();
  q_diff.normalize();

  double angle = q_diff.getAngle();
  tf2::Vector3 axis(0.0, 0.0, 0.0);

  if (std::abs(angle) > 1e-8) {
      axis = q_diff.getAxis();
  }

  angular_velocity(0) = axis.x() * angle / dt;
  angular_velocity(1) = axis.y() * angle / dt;
  angular_velocity(2) = axis.z() * angle / dt;

  twist_msg.twist.linear.x = linear_velocity(0);
  twist_msg.twist.linear.y = linear_velocity(1);
  twist_msg.twist.linear.z = linear_velocity(2);

  twist_msg.twist.angular.x = angular_velocity(0);
  twist_msg.twist.angular.y = angular_velocity(1);
  twist_msg.twist.angular.z = angular_velocity(2);
  
  //----------------------------------------------------
  // Acceleration - transform from root_frame to planning_frame
  //--------------------------------------------------

  geometry_msgs::msg::AccelStamped accel_msg;
  accel_msg.header = pose_msg.header;

  // Assuming twist_msg and last_twist_ are of type geometry_msgs::msg::TwistStamped
  geometry_msgs::msg::Vector3 delta_linear_velocity;
  geometry_msgs::msg::Vector3 delta_angular_velocity;

  delta_linear_velocity.x = twist_msg.twist.linear.x - last_twist_.twist.linear.x;
  delta_linear_velocity.y = twist_msg.twist.linear.y - last_twist_.twist.linear.y;
  delta_linear_velocity.z = twist_msg.twist.linear.z - last_twist_.twist.linear.z;

  delta_angular_velocity.x = twist_msg.twist.angular.x - last_twist_.twist.angular.x;
  delta_angular_velocity.y = twist_msg.twist.angular.y - last_twist_.twist.angular.y;
  delta_angular_velocity.z = twist_msg.twist.angular.z - last_twist_.twist.angular.z;

  geometry_msgs::msg::Vector3 linear_acceleration;
  geometry_msgs::msg::Vector3 angular_acceleration;

  linear_acceleration.x = delta_linear_velocity.x / dt;
  linear_acceleration.y = delta_linear_velocity.y / dt;
  linear_acceleration.z = delta_linear_velocity.z / dt;

  angular_acceleration.x = delta_angular_velocity.x / dt;
  angular_acceleration.y = delta_angular_velocity.y / dt;
  angular_acceleration.z = delta_angular_velocity.z / dt;

  accel_msg.accel.linear = linear_acceleration;
  accel_msg.accel.angular = angular_acceleration;

  //----------------------------------------------------
  // Publish msgs
  //--------------------------------------------------
  pose_pub_->publish(pose_msg);
  twist_pub_->publish(twist_msg);
  accel_pub_->publish(accel_msg);

  last_pose_ = pose_msg;
  last_twist_ = twist_msg;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EeStatePublisher>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    node->setup();

    rclcpp::WallRate loop_rate(200);
    RCLCPP_INFO(node->get_logger(), "Starting publishing on states topics");

    while (rclcpp::ok()) {
      node->publishState();
      loop_rate.sleep();     
    }

    RCLCPP_INFO(node->get_logger(), "End publishing on states topics");
    rclcpp::shutdown();
    return 0;
}
