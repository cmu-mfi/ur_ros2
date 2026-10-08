#ifndef UR_ROBOT_MANAGER__UR_ROBOT_MANAGER_HPP_
#define UR_ROBOT_MANAGER__UR_ROBOT_MANAGER_HPP_

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <cmath>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <deque>
#include <mutex>
#include <atomic>
// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
// Messages
#include "robot_manager_interfaces/action/joint_goal.hpp"
#include "robot_manager_interfaces/action/pose_goal.hpp"
#include "robot_manager_interfaces/srv/home.hpp"
#include "robot_manager_interfaces/srv/park.hpp"
#include "robot_manager_interfaces/srv/set_payload.hpp"
#include "robot_manager_interfaces/srv/set_io.hpp"
#include "robot_manager_interfaces/msg/pose_servo.hpp"
#include "ur_msgs/srv/set_payload.hpp"
#include "ur_msgs/srv/set_io.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace ur_robot_manager
{
  class UrRobotManager : public rclcpp::Node {
    public:
      using JointGoal = robot_manager_interfaces::action::JointGoal;
      using JointGoalHandle = rclcpp_action::ServerGoalHandle<JointGoal>;
      using PoseGoal = robot_manager_interfaces::action::PoseGoal;
      using PoseGoalHandle = rclcpp_action::ServerGoalHandle<PoseGoal>;
      using PoseServo = robot_manager_interfaces::msg::PoseServo;
      using Home = robot_manager_interfaces::srv::Home;
      using Park = robot_manager_interfaces::srv::Park;
      using SetPayload = robot_manager_interfaces::srv::SetPayload;
      using SetIo = robot_manager_interfaces::srv::SetIo;
      using UrSetPayload = ur_msgs::srv::SetPayload;
      using UrSetIo = ur_msgs::srv::SetIO;
      using WrenchStamped = geometry_msgs::msg::WrenchStamped;

      UrRobotManager();
      void setup();

    private:
      // Parameters
      std::string ns_;
      std::string tf_prefix_;

      // MoveIt
      void moveit_setup();
      std::string moveit_planning_group_;
      std::unique_ptr<moveit::planning_interface::MoveGroupInterface> moveit_move_group_;
      std::unique_ptr<moveit::planning_interface::MoveGroupInterface::Plan> moveit_current_plan_;
      std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> moveit_planning_scene_interface_;

      // Joint Goal Action
      void joint_goal_setup();
      rclcpp_action::Server<JointGoal>::SharedPtr joint_goal_action_server_;
      rclcpp_action::GoalResponse joint_goal_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const JointGoal::Goal> goal);
      rclcpp_action::CancelResponse joint_goal_handle_cancel(const std::shared_ptr<JointGoalHandle> goal_handle);
      void joint_goal_handle_accepted(const std::shared_ptr<JointGoalHandle> goal_handle);
      void joint_goal_handle_execution(const std::shared_ptr<JointGoalHandle> goal_handle);
      double calculate_joint_distance(const std::vector<double>& current, const std::vector<double>& target);

      // Pose Goal Action
      void pose_goal_setup();
      rclcpp_action::Server<PoseGoal>::SharedPtr pose_goal_action_server_;
      rclcpp_action::GoalResponse pose_goal_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PoseGoal::Goal> goal);
      rclcpp_action::CancelResponse pose_goal_handle_cancel(const std::shared_ptr<PoseGoalHandle> goal_handle);
      void pose_goal_handle_accepted(const std::shared_ptr<PoseGoalHandle> goal_handle);
      void pose_goal_handle_execution(const std::shared_ptr<PoseGoalHandle> goal_handle);
      double calculate_cartesian_distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);
      bool is_frame_tool0_child(const std::string& target_frame);

      // Home Service
      void home_service_setup();
      rclcpp::Service<Home>::SharedPtr home_service_;
      void home_service_callback(const std::shared_ptr<Home::Request> request, std::shared_ptr<Home::Response> response);

      // Park Service
      void park_service_setup();
      rclcpp::Service<Park>::SharedPtr park_service_;
      void park_service_callback(const std::shared_ptr<Park::Request> request, std::shared_ptr<Park::Response> response);
      
      // Set Payload Service
      void set_payload_service_setup();
      rclcpp::Service<SetPayload>::SharedPtr set_payload_service_;
      void set_payload_service_callback(const std::shared_ptr<SetPayload::Request> request, std::shared_ptr<SetPayload::Response> response);
      rclcpp::Client<UrSetPayload>::SharedPtr ur_set_payload_client_;

      // Set Io Service
      void set_io_service_setup();
      rclcpp::Service<SetIo>::SharedPtr set_io_service_;
      void set_io_service_callback(const std::shared_ptr<SetIo::Request> request, std::shared_ptr<SetIo::Response> response);
      rclcpp::Client<UrSetIo>::SharedPtr ur_set_io_client_;

      // Wrench Publisher
      void wrench_publisher_setup();
      rclcpp::Publisher<WrenchStamped>::SharedPtr wrench_publisher_;
      rclcpp::Subscription<WrenchStamped>::SharedPtr ur_wrench_subscriber_;
      void ur_wrench_subscription_callback_(const WrenchStamped::SharedPtr msg);

      // Servo 
      void servo_setup();
      std::shared_ptr<const servo::ParamListener> servo_param_listener_;
      std::unique_ptr<moveit_servo::Servo> servo_;
      servo::Params servo_params_;
      planning_scene_monitor::PlanningSceneMonitorPtr servo_planning_scene_monitor_;
      rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr servo_trajectory_publisher_;

      // Pose Servo
      rclcpp::Subscription<PoseServo>::SharedPtr pose_servo_subscriber_;
      rclcpp::TimerBase::SharedPtr pose_servo_timer_;
      std::atomic<bool> pose_servo_active_{false};
      rclcpp::Time pose_servo_last_msg_time_;
      moveit_servo::PoseCommand pose_servo_target_pose_cmd_;
      std::mutex pose_servo_mutex_;
      std::deque<moveit_servo::KinematicState> pose_servo_joint_cmd_rolling_window_;
      void pose_servo_subscription_callback_(const PoseServo::SharedPtr msg);
      void pose_servo_loop_callback_();

      // TF Variables
      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

      // ROS2 Variables
      rclcpp::CallbackGroup::SharedPtr service_cb_group_;
      rclcpp::CallbackGroup::SharedPtr servo_cb_group_;
  };

}  // namespace ur_robot_manager

#endif  // UR_ROBOT_MANAGER__UR_ROBOT_MANAGER_HPP_
