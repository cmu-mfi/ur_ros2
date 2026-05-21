import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import threading
import time
import math

from robot_manager_interfaces.action import JointGoal

class JointGoalExample(Node):
    def __init__(self):
        super().__init__('joint_goal_example')

        self.declare_parameter('ns', '')
        self.ns = str(self.get_parameter("ns").value) + "/"
        
        self.joint_goal_client = ActionClient(self, JointGoal, self.ns + "joint_goal")
        self.joint_goal_client.wait_for_server()

def main(args=None):
    rclpy.init(args=args)
    node = JointGoalExample()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        # Create Goal
        goal_msg = JointGoal.Goal()
        goal_msg.positions = [0.0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0.0]
        goal_msg.velocity_scaling = 0.1
        goal_msg.acceleration_scaling = 0.1

        # Blocking Execution without Feedback
        node.get_logger().info("Starting Execution")
        node.joint_goal_client.send_goal(goal_msg)
        node.get_logger().info("Finished Execution")

        # Asynchronous Execution with Feedback
        goal_msg.positions = [0.3, -2.0, 2.0, -2.0, -2.0, 0.5]
        node.get_logger().info("Starting Execution")
        send_future = node.joint_goal_client.send_goal_async(goal_msg, feedback_callback=lambda msg: node.get_logger().info(f'Progress: {msg.feedback.progress:.1f}%'))
        while not send_future.done(): continue # Wait for Goal to be accepted
        result = send_future.result().get_result() # Wait for Goal to be executed
        if not result.result.success: node.get_logger().error(result.result.message)
        else: node.get_logger().info("Finished Execution")

        # Cancelling Execution (works but errors out the robot because of rapid deceleration)
        goal_msg.positions = [0.0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0.0]
        node.get_logger().info("Starting Execution")
        send_future = node.joint_goal_client.send_goal_async(goal_msg, feedback_callback=lambda msg: node.get_logger().info(f'Progress: {msg.feedback.progress:.1f}%'))
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
