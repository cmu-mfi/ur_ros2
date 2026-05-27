import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from action_msgs.msg import GoalStatus

import threading
import time
import math

from robot_manager_interfaces.action import PoseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from tf_transformations import quaternion_from_euler

class PoseGoalExample(Node):
    def __init__(self):
        super().__init__('pose_goal_example')

        self.declare_parameter('ns', '')
        self.ns = str(self.get_parameter("ns").value) + "/"
        
        self.pose_goal_client = ActionClient(self, PoseGoal, self.ns + "pose_goal")
        self.pose_goal_client.wait_for_server()

    def run_action(self, action_client: ActionClient, goal_msg, show_progress = False):
        if show_progress:
            result = action_client.send_goal(goal_msg, feedback_callback=lambda msg: self.get_logger().info(f'Progress: {msg.feedback.progress:.1f}%'))
        else:
            result = action_client.send_goal(goal_msg)
        status = result.status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(result.result.message)
            exit(1)

def main(args=None):
    rclpy.init(args=args)
    node = PoseGoalExample()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        # Create Goal
        goal_msg = PoseGoal.Goal()
        q = quaternion_from_euler(math.radians(180), math.radians(0), math.radians(-90))
        goal_msg.target_pose = Pose(
                position=Point(x=0.8, y=0.0, z=0.8),
                orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                )
        goal_msg.velocity_scaling = 0.2
        goal_msg.acceleration_scaling = 0.1
        goal_msg.frame_id = "" # Can be any frame. If empty -> base_link used
        goal_msg.target_id = "" # Can be any child of tool0. If empty -> tool0 used
        goal_msg.method = "PTP" # Point-to-Point

        # Execution without Feedback
        node.get_logger().info("Starting Execution")
        node.run_action(node.pose_goal_client, goal_msg)
        node.get_logger().info("Finished Execution")

        # Execution with Feedback
        goal_msg.target_pose.position.y = 0.2
        goal_msg.method = "LIN" # Linear
        node.get_logger().info("Starting Execution")
        node.run_action(node.pose_goal_client, goal_msg, True)
        node.get_logger().info("Finished Execution")

        # Cancelling Execution (works but errors the robot)
        goal_msg.target_pose.position.y = -0.2
        node.get_logger().info("Starting Execution")
        send_future = node.pose_goal_client.send_goal_async(goal_msg, feedback_callback=lambda msg: node.get_logger().info(f'Progress: {msg.feedback.progress:.1f}%'))
        while not send_future.done(): continue # Wait for Goal to be accepted
        # Uncommend these two lines to cancel:
        # time.sleep(0.5)
        # send_future.result().cancel_goal() 
        result = send_future.result().get_result() # Wait for Goal to be executed
        if not result.result.success: node.get_logger().error(result.result.message)
        else: node.get_logger().info("Finished Execution")

    except KeyboardInterrupt:
        node.get_logger().info("Script interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main()
