import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from action_msgs.msg import GoalStatus

import threading
import math

from robot_manager_interfaces.action import JointGoal, PoseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from tf_transformations import quaternion_from_euler

class MotionTesting(Node):
    def __init__(self):
        super().__init__('motion_testing')

        self.declare_parameter('ns', '')
        self.ns = str(self.get_parameter("ns").value) + "/"

        self.joint_goal_client = ActionClient(self, JointGoal, self.ns + "joint_goal")
        self.joint_goal_client.wait_for_server()

        self.pose_goal_client = ActionClient(self, PoseGoal, self.ns + "pose_goal")
        self.pose_goal_client.wait_for_server()

    def run_action(self, action_client: ActionClient, goal_msg, show_progress=False):
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
    node = MotionTesting()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        # --- Joint Goals ---
        joint_goal_msg = JointGoal.Goal()
        joint_goal_msg.velocity_scaling = 0.5
        joint_goal_msg.acceleration_scaling = 0.1
        start_position = [0.0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0.0]
        joint_goal_msg.positions = start_position
        node.run_action(node.joint_goal_client, joint_goal_msg, True)
        joint_goal_msg.positions = [x + 0.1 for x in start_position]
        node.run_action(node.joint_goal_client, joint_goal_msg, True)
        joint_goal_msg.positions = [x - 0.1 for x in start_position]
        node.run_action(node.joint_goal_client, joint_goal_msg, True)
        joint_goal_msg.positions = start_position
        node.run_action(node.joint_goal_client, joint_goal_msg, True)

        # --- Pose Goals ---
        pose_goal_msg = PoseGoal.Goal()
        q = quaternion_from_euler(math.radians(180), math.radians(0), math.radians(-90))
        pose_goal_msg.target_pose = Pose(
            position=Point(x=0.8, y=0.0, z=0.8),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        )
        pose_goal_msg.velocity_scaling = 0.5
        pose_goal_msg.acceleration_scaling = 0.2
        pose_goal_msg.frame_id = ""
        pose_goal_msg.target_id = ""
        pose_goal_msg.method = "PTP"
        node.run_action(node.pose_goal_client, pose_goal_msg, True)

        pose_goal_msg.method = "LIN"
        pose_goal_msg.target_pose.position.y = 0.2
        node.run_action(node.pose_goal_client, pose_goal_msg, True)
        pose_goal_msg.target_pose.position.y = -0.2
        node.run_action(node.pose_goal_client, pose_goal_msg, True)

        pose_goal_msg.method = "PTP"
        q = quaternion_from_euler(math.radians(160), math.radians(20), math.radians(-70))
        pose_goal_msg.target_pose = Pose(
            position=Point(x=1.0, y=0.0, z=0.6),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        )
        node.run_action(node.pose_goal_client, pose_goal_msg, True)

        joint_goal_msg.positions = start_position
        node.run_action(node.joint_goal_client, joint_goal_msg, True)

    except KeyboardInterrupt:
        node.get_logger().info("Script interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
