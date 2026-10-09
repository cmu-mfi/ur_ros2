#include "ur_robot_manager/ur_robot_manager.hpp"

namespace ur_robot_manager
{

  // --- Helper Function - Transform Pose to Tool0 Frame --- ///
  geometry_msgs::msg::Pose UrRobotManager::transform_pose_to_tool0_frame(
      const geometry_msgs::msg::Pose& pose,
      const std::string& input_frame)
  {
    std::string tool0_frame = moveit_move_group_ ? moveit_move_group_->getEndEffectorLink() : tf_prefix_ + "tool0";
    if (tool0_frame.empty()) {
      tool0_frame = tf_prefix_ + "tool0";
    }
    // Check if transformation is necessary
    if (input_frame.empty() || input_frame == tool0_frame) {
      return pose;
    }
    // Check if transformation is possible (only if it's a child transform of tool0)
    bool is_child = false;
    std::string current_frame = input_frame;
    std::string next_parent;
    tf2::TimePoint time = tf2::TimePointZero;

    while (tf_buffer_->_getParent(current_frame, time, next_parent)) {
      if (next_parent == tool0_frame) {
        is_child = true;
        break;
      }
      current_frame = next_parent;
    }

    if (!is_child) {
      RCLCPP_ERROR(this->get_logger(), "[TF] Verification Failed: Target Frame '%s' is not a Child of '%s'", input_frame.c_str(), tool0_frame.c_str());
      throw tf2::TransformException("Target Frame '" + input_frame + "' is not a Child of '" + tool0_frame + "'");
    }

    // Transform pose to tool0 frame
    try {
      geometry_msgs::msg::TransformStamped tool0_in_target_msg =
          tf_buffer_->lookupTransform(input_frame, tool0_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5));
      tf2::Transform target_pose;
      tf2::fromMsg(pose, target_pose);
      tf2::Transform tf_tool0_in_target;
      tf2::fromMsg(tool0_in_target_msg.transform, tf_tool0_in_target);
      tf2::Transform tf_tool0_in_frame = target_pose * tf_tool0_in_target;
      geometry_msgs::msg::Pose tool0_pose;
      tf2::toMsg(tf_tool0_in_frame, tool0_pose);
      return tool0_pose;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "[TF] Verification Failed: Could not transform %s to %s: %s",
                   tool0_frame.c_str(), input_frame.c_str(), ex.what());
      throw;
    }
  }

  // --- Helper Function - Transform Pose to Base Link Frame --- ///
  geometry_msgs::msg::Pose UrRobotManager::transform_pose_to_base_link_frame(
      const geometry_msgs::msg::Pose& pose,
      const std::string& input_frame)
  {
    std::string base_link_frame = tf_prefix_ + "base_link";
    // Check if transformation is necessary
    if (input_frame.empty() || input_frame == base_link_frame || input_frame == "base_link") {
      return pose;
    }
    // Transform pose to base_link frame
    try {
      geometry_msgs::msg::PoseStamped pose_in;
      pose_in.header.frame_id = input_frame;
      pose_in.header.stamp = rclcpp::Time(0);
      pose_in.pose = pose;

      geometry_msgs::msg::PoseStamped pose_out = tf_buffer_->transform(
          pose_in,
          base_link_frame,
          tf2::durationFromSec(0.5));
      return pose_out.pose;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "[TF] Verification Failed: Could not transform %s to %s: %s",
                   input_frame.c_str(), base_link_frame.c_str(), ex.what());
      throw;
    }
  }

}  // namespace ur_robot_manager
