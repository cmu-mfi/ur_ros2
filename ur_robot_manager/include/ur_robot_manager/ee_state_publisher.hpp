#ifndef EE_STATE_PUBLISHER__UR_ROBOT_MANAGER_HPP_
#define EE_STATE_PUBLISHER__UR_ROBOT_MANAGER_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <memory>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class EeStatePublisher : public rclcpp::Node {
public:
    EeStatePublisher();
    void setup();
    void publishState();

private:
    geometry_msgs::msg::PoseStamped getPoseInBaseFrame(geometry_msgs::msg::PoseStamped tool_pose);

    // Variables
    std::string ns_;
    std::string tf_prefix_;
    std::string base_frame_;
    std::string planning_group_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;   
    geometry_msgs::msg::PoseStamped last_pose_;
    geometry_msgs::msg::TwistStamped last_twist_;

    // EE State Publisher related members
    std::string ee_link_;
    const moveit::core::JointModelGroup *joint_model_group_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr accel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // EE_STATE_PUBLISHER__UR_ROBOT_MANAGER_HPP_
