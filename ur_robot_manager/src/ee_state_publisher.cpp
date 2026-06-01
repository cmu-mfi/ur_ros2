#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager {

void UrRobotManager::publishState() {
  auto state = move_group_->getCurrentState(0.1);

  if (!state) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Failed to get robot state");
    return;
  }

  //----------------------------------------------------
  // Pose
  //--------------------------------------------------

  const Eigen::Isometry3d &tf = state->getGlobalLinkTransform(ee_link_);

  geometry_msgs::msg::PoseStamped pose_msg;

  pose_msg.header.stamp = now();
  pose_msg.header.frame_id = move_group_->getPlanningFrame();

  pose_msg.pose.position.x = tf.translation().x();
  pose_msg.pose.position.y = tf.translation().y();
  pose_msg.pose.position.z = tf.translation().z();

  Eigen::Quaterniond q(tf.rotation());

  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();

  //----------------------------------------------------
  // Jacobian
  //--------------------------------------------------

  Eigen::MatrixXd jacobian;

  state->getJacobian(joint_model_group_, state->getLinkModel(ee_link_),
                     Eigen::Vector3d::Zero(), jacobian);

  //----------------------------------------------------
  // Joint velocities
  //--------------------------------------------------

  Eigen::VectorXd qdot;
  state->copyJointGroupVelocities(joint_model_group_, qdot);

  Eigen::VectorXd twist = jacobian * qdot;

  geometry_msgs::msg::TwistStamped twist_msg;

  twist_msg.header = pose_msg.header;

  if (twist.size() >= 6) {
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

  try {
    state->copyJointGroupAccelerations(joint_model_group_, qddot);
  } catch (...) {
    qddot = Eigen::VectorXd::Zero(joint_model_group_->getVariableCount());
  }

  Eigen::VectorXd accel = jacobian * qddot;

  geometry_msgs::msg::AccelStamped accel_msg;

  accel_msg.header = pose_msg.header;

  if (accel.size() >= 6) {
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

}; // namespace ur_robot_manager