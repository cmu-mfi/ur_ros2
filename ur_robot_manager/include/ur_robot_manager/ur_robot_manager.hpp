#ifndef UR_ROBOT_MANAGER__UR_ROBOT_MANAGER_HPP_
#define UR_ROBOT_MANAGER__UR_ROBOT_MANAGER_HPP_

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

#include "robot_manager_interfaces/action/joint_goal.hpp"
#include "robot_manager_interfaces/action/pose_goal.hpp"
#include "robot_manager_interfaces/srv/home.hpp"
#include "robot_manager_interfaces/srv/set_payload.hpp"
#include "ur_msgs/srv/set_payload.hpp"

namespace ur_robot_manager
{
  class UrRobotManager : public rclcpp::Node {
    public:
      using JointGoal = robot_manager_interfaces::action::JointGoal;
      using JointGoalHandle = rclcpp_action::ServerGoalHandle<JointGoal>;
      using PoseGoal = robot_manager_interfaces::action::PoseGoal;
      using PoseGoalHandle = rclcpp_action::ServerGoalHandle<PoseGoal>;
      using Home = robot_manager_interfaces::srv::Home;
      using SetPayload = robot_manager_interfaces::srv::SetPayload;

      UrRobotManager();
      void setup();

    private:
      // Joint Goal Action
      rclcpp_action::Server<JointGoal>::SharedPtr joint_goal_action_server_;
      rclcpp_action::GoalResponse joint_goal_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const JointGoal::Goal> goal);
      rclcpp_action::CancelResponse joint_goal_handle_cancel(const std::shared_ptr<JointGoalHandle> goal_handle);
      void joint_goal_handle_accepted(const std::shared_ptr<JointGoalHandle> goal_handle);
      void joint_goal_handle_execution(const std::shared_ptr<JointGoalHandle> goal_handle);
      double calculate_joint_distance(const std::vector<double>& current, const std::vector<double>& target);

      // Pose Goal Action
      rclcpp_action::Server<PoseGoal>::SharedPtr pose_goal_action_server_;
      rclcpp_action::GoalResponse pose_goal_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PoseGoal::Goal> goal);
      rclcpp_action::CancelResponse pose_goal_handle_cancel(const std::shared_ptr<PoseGoalHandle> goal_handle);
      void pose_goal_handle_accepted(const std::shared_ptr<PoseGoalHandle> goal_handle);
      void pose_goal_handle_execution(const std::shared_ptr<PoseGoalHandle> goal_handle);
      double calculate_cartesian_distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);
      bool is_frame_tool0_child(const std::string& target_frame);

      // Home Service
      rclcpp::Service<Home>::SharedPtr home_service_;
      void home_service_callback(const std::shared_ptr<Home::Request> request, std::shared_ptr<Home::Response> response);
      
      // Set Payload Service
      rclcpp::Service<SetPayload>::SharedPtr set_payload_service_;
      void set_payload_service_callback(const std::shared_ptr<SetPayload::Request> request, std::shared_ptr<SetPayload::Response> response);
      rclcpp::Client<ur_msgs::srv::SetPayload>::SharedPtr ur_set_payload_client_;

      // Parameters
      std::string ns_;
      std::string tf_prefix_;

      // MoveIt Variables
      std::string planning_group_;
      std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
      std::unique_ptr<moveit::planning_interface::MoveGroupInterface::Plan> current_plan_;
      std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

      // TF Variables
      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

      // ROS2 Variables
      rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  };

}  // namespace ur_robot_manager

#endif  // UR_ROBOT_MANAGER__UR_ROBOT_MANAGER_HPP_
