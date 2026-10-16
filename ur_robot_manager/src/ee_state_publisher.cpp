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
        ee_state_cb_group_
        );
  }

  // --- EE State Publisher - Loop Callback --- ///
  void UrRobotManager::ee_state_publisher_loop_callback_() {
    // Pose from MoveIt is already in planning frame
    geometry_msgs::msg::PoseStamped pose_msg = moveit_move_group_->getCurrentPose(tf_prefix_ + "tool0");

    geometry_msgs::msg::PoseStamped last_pose;
    geometry_msgs::msg::TwistStamped last_twist;
    {
      std::lock_guard<std::mutex> lock(ee_state_mutex_);
      last_pose = ee_current_pose_;
      last_twist = ee_current_twist_;
    }

    const rclcpp::Time t_curr(pose_msg.header.stamp);
    const rclcpp::Time t_last(last_pose.header.stamp);
    const double dt = (t_curr - t_last).seconds();

    if (dt <= 0.0) {
      return;
    }

    // Calculate Twist (Linear and Angular Velocity)
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header = pose_msg.header;

    twist_msg.twist.linear.x = (pose_msg.pose.position.x - last_pose.pose.position.x) / dt;
    twist_msg.twist.linear.y = (pose_msg.pose.position.y - last_pose.pose.position.y) / dt;
    twist_msg.twist.linear.z = (pose_msg.pose.position.z - last_pose.pose.position.z) / dt;

    const tf2::Quaternion q_last(
        last_pose.pose.orientation.x,
        last_pose.pose.orientation.y,
        last_pose.pose.orientation.z,
        last_pose.pose.orientation.w);
    const tf2::Quaternion q_curr(
        pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z,
        pose_msg.pose.orientation.w);

    tf2::Quaternion q_diff = (q_curr * q_last.inverse()).normalized();
    const double angle = q_diff.getAngle();
    if (std::abs(angle) > 1e-8) {
      const tf2::Vector3 axis = q_diff.getAxis();
      const double scale = angle / dt;
      twist_msg.twist.angular.x = axis.x() * scale;
      twist_msg.twist.angular.y = axis.y() * scale;
      twist_msg.twist.angular.z = axis.z() * scale;
    }

    // Calculate Acceleration directly into message
    geometry_msgs::msg::AccelStamped accel_msg;
    accel_msg.header = pose_msg.header;

    accel_msg.accel.linear.x = (twist_msg.twist.linear.x - last_twist.twist.linear.x) / dt;
    accel_msg.accel.linear.y = (twist_msg.twist.linear.y - last_twist.twist.linear.y) / dt;
    accel_msg.accel.linear.z = (twist_msg.twist.linear.z - last_twist.twist.linear.z) / dt;

    accel_msg.accel.angular.x = (twist_msg.twist.angular.x - last_twist.twist.angular.x) / dt;
    accel_msg.accel.angular.y = (twist_msg.twist.angular.y - last_twist.twist.angular.y) / dt;
    accel_msg.accel.angular.z = (twist_msg.twist.angular.z - last_twist.twist.angular.z) / dt;

    // Update internal state under mutex lock
    {
      std::lock_guard<std::mutex> lock(ee_state_mutex_);
      ee_current_pose_ = pose_msg;
      ee_current_twist_ = twist_msg;
      ee_current_accel_ = accel_msg;
    }

    // Publish state
    ee_pose_pub_->publish(pose_msg);
    ee_twist_pub_->publish(twist_msg);
    ee_accel_pub_->publish(accel_msg);
  }
}  // namespace ur_robot_manager
