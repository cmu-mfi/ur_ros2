import math
import rclpy
import time
from rclpy.node import Node
from rclpy.action.client import ActionClient
from action_msgs.msg import GoalStatus
from robot_manager_interfaces.action import PoseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, TwistStamped
from tf_transformations import quaternion_from_euler

class TwistServoExample(Node):
    def __init__(self):
        super().__init__('twist_servo_example')

        self.declare_parameter('ns', 'ur20')
        self.ns = str(self.get_parameter("ns").value) + "/"

        self.publisher_ = self.create_publisher(
            TwistStamped, self.ns + 'twist_servo', 10
        )
        self.pose_goal_client = ActionClient(self, PoseGoal, self.ns + "pose_goal")
        self.pose_goal_client.wait_for_server()

        goal_msg = PoseGoal.Goal()
        self.q = quaternion_from_euler(math.radians(0), math.radians(0), math.radians(-90))
        goal_msg.target_pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.5),
            orientation=Quaternion(x=self.q[0], y=self.q[1], z=self.q[2], w=self.q[3])
        )
        goal_msg.velocity_scaling = 0.4
        goal_msg.acceleration_scaling = 0.2
        goal_msg.frame_id = "world" 
        goal_msg.target_id = "ur20_tc_connector" 
        goal_msg.method = "PTP"
        
        # Execution wait until action finishes completely
        self.run_action(self.pose_goal_client, goal_msg)
        time.sleep(1)

        # 100 Hz publishing rate (0.01 seconds period)
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("Starting streaming twist commands")

    def run_action(self, action_client: ActionClient, goal_msg, show_progress = False):
        feedback_cb = (lambda msg: self.get_logger().info(f'Progress: {msg.feedback.progress:.1f}%')) if show_progress else None
        # 1. Send goal asynchronously and wait for acceptance
        send_goal_future = action_client.send_goal_async(goal_msg, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by action server")
            exit(1)
        # 2. Wait for full action execution result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            message = getattr(result.result, 'message', f'Action failed with status {result.status}')
            self.get_logger().error(message)
            exit(1)
        return result

    def timer_callback(self):
        msg = TwistStamped()
        # Update Header Stamp and Reference Frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        # Calculate sinusoidal linear velocity on x axis (m/s)
        msg.twist.linear.x = 0.2
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        # Angular velocity components (rad/s)
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistServoExample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
