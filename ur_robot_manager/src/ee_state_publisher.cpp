#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager {

  // --- EE State Publisher - Setup --- ///
  void UrRobotManager::ee_state_publisher_setup() {
    // Publishers
    ee_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ee_pose", 10);
    ee_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("ee_twist", 10);
    ee_accel_pub_ = this->create_publisher<geometry_msgs::msg::AccelStamped>("ee_accel", 10);
    // Setup EE Link
    ee_joint_model_group_ = moveit_move_group_->getRobotModel()->getJointModelGroup(moveit_planning_group_);
    ee_link_ = moveit_move_group_->getEndEffectorLink();
    if (ee_link_.empty()) {
      const auto &links = ee_joint_model_group_->getLinkModelNames();
      ee_link_ = links.back();
      RCLCPP_WARN(this->get_logger(), "[EE State Publisher] No EE link configured. Using last link: %s", ee_link_.c_str());
    }
    moveit_move_group_->startStateMonitor();
    moveit_move_group_->setPoseReferenceFrame(tf_prefix_+"base");
    // Wait until TF can provide the transform without relying on MoveIt's noisy state monitor 
    RCLCPP_INFO(this->get_logger(), "[EE State Publisher] Waiting for current robot state...");
    
    while (rclcpp::ok()) {
        if (tf_buffer_->canTransform(tf_prefix_ + "base", tf_prefix_ + "tool0", tf2::TimePointZero)) {
            try {
                // Grab the initial pose directly via TF to bypass the MoveIt uninitialized state error
                geometry_msgs::msg::TransformStamped tf_msg = 
                    tf_buffer_->lookupTransform(tf_prefix_ + "base", tf_prefix_ + "tool0", tf2::TimePointZero);
                
                ee_last_pose_.header = tf_msg.header;
                ee_last_pose_.pose.position.x = tf_msg.transform.translation.x;
                ee_last_pose_.pose.position.y = tf_msg.transform.translation.y;
                ee_last_pose_.pose.position.z = tf_msg.transform.translation.z;
                ee_last_pose_.pose.orientation = tf_msg.transform.rotation;
                // Check if we got valid orientation data 
                if (ee_last_pose_.pose.orientation.w != 0.0) {
                    break; 
                }
            } catch (const tf2::TransformException &) {
                // Ignore transient TF exceptions and keep trying
            }
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(this->get_logger(), "[EE State Publisher] Publishing EE state for link: %s", ee_link_.c_str());
    // Original loop was 200Hz, translating to a 5ms timer
    ee_state_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&UrRobotManager::ee_state_publisher_loop_callback_, this),
        service_cb_group_
    );
  }

  // --- EE State Publisher - Loop Callback --- ///
  void UrRobotManager::ee_state_publisher_loop_callback_() {
    // Pose - transform from root_frame to planning_frame (base_link)
    geometry_msgs::msg::PoseStamped pose_msg = moveit_move_group_->getCurrentPose(tf_prefix_+"tool0");

    rclcpp::Time t_curr(pose_msg.header.stamp);
    rclcpp::Time t_last(ee_last_pose_.header.stamp);
    double dt = (t_curr - t_last).seconds();
    
    if (dt <= 0.0) {
      return; 
    }

    // Twist - transform from root_frame to planning_frame
    geometry_msgs::msg::TwistStamped twist_msg;    
    twist_msg.header = pose_msg.header;
    Eigen::Vector3d linear_velocity, angular_velocity;
    tf2::Quaternion q_last(ee_last_pose_.pose.orientation.x, ee_last_pose_.pose.orientation.y, ee_last_pose_.pose.orientation.z, ee_last_pose_.pose.orientation.w);
    tf2::Quaternion q_curr(pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);
    // Linear velocity
    linear_velocity(0) = (pose_msg.pose.position.x - ee_last_pose_.pose.position.x) / dt;
    linear_velocity(1) = (pose_msg.pose.position.y - ee_last_pose_.pose.position.y) / dt;
    linear_velocity(2) = (pose_msg.pose.position.z - ee_last_pose_.pose.position.z) / dt;
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

    // Acceleration - transform from root_frame to planning_frame
    geometry_msgs::msg::AccelStamped accel_msg;
    accel_msg.header = pose_msg.header;
    geometry_msgs::msg::Vector3 delta_linear_velocity;
    geometry_msgs::msg::Vector3 delta_angular_velocity;
    delta_linear_velocity.x = twist_msg.twist.linear.x - ee_last_twist_.twist.linear.x;
    delta_linear_velocity.y = twist_msg.twist.linear.y - ee_last_twist_.twist.linear.y;
    delta_linear_velocity.z = twist_msg.twist.linear.z - ee_last_twist_.twist.linear.z;
    delta_angular_velocity.x = twist_msg.twist.angular.x - ee_last_twist_.twist.angular.x;
    delta_angular_velocity.y = twist_msg.twist.angular.y - ee_last_twist_.twist.angular.y;
    delta_angular_velocity.z = twist_msg.twist.angular.z - ee_last_twist_.twist.angular.z;
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

    // Publish msgs
    ee_pose_pub_->publish(pose_msg);
    ee_twist_pub_->publish(twist_msg);
    ee_accel_pub_->publish(accel_msg);
    
    // save for next run
    ee_last_pose_ = pose_msg;
    ee_last_twist_ = twist_msg;
  }

  // --- Helper Function - Get Pose In Base Frame --- ///
  geometry_msgs::msg::PoseStamped UrRobotManager::get_pose_in_base_frame(geometry_msgs::msg::PoseStamped tool_pose) {
    geometry_msgs::msg::PoseStamped pose_in_base;
    try {
      pose_in_base = tf_buffer_->transform(
          tool_pose,
          tf_prefix_ + "base",
          tf2::durationFromSec(0.5));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "[EE State Publisher] TF failed: %s", ex.what());
      throw;
    }
    return pose_in_base;
  }

}  // namespace ur_robot_manager
