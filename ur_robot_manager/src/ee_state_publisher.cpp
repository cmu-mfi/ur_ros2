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
    moveit_move_group_->setPoseReferenceFrame(tf_prefix_ + "base_link");

    // Wait until TF can provide the transform without relying on MoveIt's noisy state monitor 
    RCLCPP_INFO(this->get_logger(), "[EE State Publisher] Waiting for current robot state...");
    
    while (rclcpp::ok()) {
      if (tf_buffer_->canTransform(tf_prefix_ + "base_link", tf_prefix_ + "tool0", tf2::TimePointZero)) {
        try {
          // Grab the initial pose directly via TF to bypass the MoveIt uninitialized state error
          geometry_msgs::msg::TransformStamped tf_msg = 
             tf_buffer_->lookupTransform(tf_prefix_ + "base_link", tf_prefix_ + "tool0", tf2::TimePointZero);
          
          geometry_msgs::msg::PoseStamped initial_pose;
          initial_pose.header = tf_msg.header;
          initial_pose.pose.position.x = tf_msg.transform.translation.x;
          initial_pose.pose.position.y = tf_msg.transform.translation.y;
          initial_pose.pose.position.z = tf_msg.transform.translation.z;
          initial_pose.pose.orientation = tf_msg.transform.rotation;

          // Check if we got valid orientation data 
          if (initial_pose.pose.orientation.w != 0.0) {
            std::lock_guard<std::mutex> lock(ee_state_mutex_);
            ee_current_pose_ = initial_pose;
            ee_previous_pose_1 = initial_pose;
            ee_previous_pose_2 = initial_pose;
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
    geometry_msgs::msg::PoseStamped pose_msg = moveit_move_group_->getCurrentPose(tf_prefix_ + "tool0");

    geometry_msgs::msg::PoseStamped last_pose;
    geometry_msgs::msg::TwistStamped last_twist;
    {
      std::lock_guard<std::mutex> lock(ee_state_mutex_);
      last_pose = ee_current_pose_;
      last_twist = ee_current_twist_;
    }

    rclcpp::Time t_curr(pose_msg.header.stamp);
    rclcpp::Time t_last(last_pose.header.stamp);
    double dt = (t_curr - t_last).seconds();
    
    if (dt <= 0.0) {
      return;
    }

    // Twist - transform from root_frame to planning_frame
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header = pose_msg.header;
    Eigen::Vector3d linear_velocity, angular_velocity;

    tf2::Quaternion q_last(last_pose.pose.orientation.x, last_pose.pose.orientation.y, last_pose.pose.orientation.z, last_pose.pose.orientation.w);
    tf2::Quaternion q_curr(pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);

    // Linear velocity
    linear_velocity(0) = (pose_msg.pose.position.x - last_pose.pose.position.x) / dt;
    linear_velocity(1) = (pose_msg.pose.position.y - last_pose.pose.position.y) / dt;
    linear_velocity(2) = (pose_msg.pose.position.z - last_pose.pose.position.z) / dt;

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

    delta_linear_velocity.x = twist_msg.twist.linear.x - last_twist.twist.linear.x;
    delta_linear_velocity.y = twist_msg.twist.linear.y - last_twist.twist.linear.y;
    delta_linear_velocity.z = twist_msg.twist.linear.z - last_twist.twist.linear.z;

    delta_angular_velocity.x = twist_msg.twist.angular.x - last_twist.twist.angular.x;
    delta_angular_velocity.y = twist_msg.twist.angular.y - last_twist.twist.angular.y;
    delta_angular_velocity.z = twist_msg.twist.angular.z - last_twist.twist.angular.z;

    accel_msg.accel.linear.x = delta_linear_velocity.x / dt;
    accel_msg.accel.linear.y = delta_linear_velocity.y / dt;
    accel_msg.accel.linear.z = delta_linear_velocity.z / dt;
    accel_msg.accel.angular.x = delta_angular_velocity.x / dt;
    accel_msg.accel.angular.y = delta_angular_velocity.y / dt;
    accel_msg.accel.angular.z = delta_angular_velocity.z / dt;

    // Update internal state variables under mutex lock
    {
      std::lock_guard<std::mutex> lock(ee_state_mutex_);
      ee_previous_pose_2 = ee_previous_pose_1;
      ee_previous_pose_1 = ee_current_pose_;
      ee_current_pose_ = pose_msg;
      ee_current_twist_ = twist_msg;
      ee_current_accel_ = accel_msg;
    }

    // Publish msgs
    ee_pose_pub_->publish(pose_msg);
    ee_twist_pub_->publish(twist_msg);
    ee_accel_pub_->publish(accel_msg);
  }

}  // namespace ur_robot_manager
