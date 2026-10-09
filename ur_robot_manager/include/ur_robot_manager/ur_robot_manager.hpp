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
#include "robot_manager_interfaces/srv/start_admittance_pose_servo.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ur_msgs/srv/set_payload.hpp"
#include "ur_msgs/srv/set_io.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace ur_robot_manager
{
  // Admittance Pose Servo Configuration Struct
  struct AdmittancePoseServoConfig
  {
    geometry_msgs::msg::Wrench target_wrench;
    std::array<double, 6> mass;
    std::array<double, 6> damping;
    std::array<double, 6> stiffness;
    std::array<bool, 6> selection_vector;
    std::array<double, 6> max_velocity;
    std::array<double, 6> max_acceleration;
  };

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
      using StartAdmittancePoseServo = robot_manager_interfaces::srv::StartAdmittancePoseServo;
      using Trigger = std_srvs::srv::Trigger;

      UrRobotManager();
      void setup();

    private:
      // Parameters
      std::string ns_;
      std::string tf_prefix_;

      // TF 
      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
      geometry_msgs::msg::Pose transform_pose_to_tool0_frame(const geometry_msgs::msg::Pose& pose, const std::string& input_frame);
      geometry_msgs::msg::Pose transform_pose_to_base_link_frame(const geometry_msgs::msg::Pose& pose, const std::string& input_frame);

      // ROS2 Variables
      rclcpp::CallbackGroup::SharedPtr service_cb_group_;
      rclcpp::CallbackGroup::SharedPtr servo_cb_group_;

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
      std::mutex wrench_mutex_;
      WrenchStamped current_wrench_;
      WrenchStamped previous_wrench_;

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

      // Twist Servo
      rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_servo_subscriber_;
      rclcpp::TimerBase::SharedPtr twist_servo_timer_;
      std::atomic<bool> twist_servo_active_{false};
      rclcpp::Time twist_servo_last_msg_time_;
      moveit_servo::TwistCommand twist_servo_target_cmd_;
      std::mutex twist_servo_mutex_;
      std::deque<moveit_servo::KinematicState> twist_servo_joint_cmd_rolling_window_;
      void twist_servo_subscription_callback_(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
      void twist_servo_loop_callback_();

      // EE State Publisher
      void ee_state_publisher_setup();
      void ee_state_publisher_loop_callback_();
      geometry_msgs::msg::PoseStamped get_pose_in_base_frame(geometry_msgs::msg::PoseStamped tool_pose);
      std::string ee_link_;
      const moveit::core::JointModelGroup * ee_joint_model_group_;
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
      rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ee_twist_pub_;
      rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr ee_accel_pub_;
      rclcpp::TimerBase::SharedPtr ee_state_timer_;
      // State storage and Thread Safety
      std::mutex ee_state_mutex_;
      geometry_msgs::msg::PoseStamped ee_current_pose_;
      geometry_msgs::msg::PoseStamped ee_previous_pose_1;
      geometry_msgs::msg::PoseStamped ee_previous_pose_2;
      geometry_msgs::msg::TwistStamped ee_current_twist_;
      geometry_msgs::msg::AccelStamped ee_current_accel_;

      // Admittance Pose Servo
      void admittance_pose_servo_setup();
      void start_admittance_pose_servo_service_callback(
        const std::shared_ptr<StartAdmittancePoseServo::Request> request,
        std::shared_ptr<StartAdmittancePoseServo::Response> response);
      void stop_admittance_pose_servo_service_callback(
        const std::shared_ptr<Trigger::Request> request,
        std::shared_ptr<Trigger::Response> response);
      void admittance_pose_servo_loop_callback_();
      rclcpp::Service<StartAdmittancePoseServo>::SharedPtr start_admittance_pose_servo_service_;
      rclcpp::Service<Trigger>::SharedPtr stop_admittance_pose_servo_service_;
      rclcpp::TimerBase::SharedPtr admittance_pose_servo_timer_;
      std::atomic<bool> admittance_pose_servo_active_{false};
      AdmittancePoseServoConfig admittance_pose_servo_config_;
      std::mutex admittance_pose_servo_mutex_;
      rclcpp::Time admittance_pose_servo_previous_time_;
  };

}  // namespace ur_robot_manager

#endif  // UR_ROBOT_MANAGER__UR_ROBOT_MANAGER_HPP_
