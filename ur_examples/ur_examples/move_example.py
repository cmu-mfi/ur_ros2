import rclpy
from rclpy.action.client import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, JointConstraint
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from shape_msgs.msg import SolidPrimitive
import math

class Move():
    def __init__(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node('move_example')
        self.node.declare_parameter('ns', '')
        self.ns = self.node.get_parameter("ns").value
        self.prefix = ""
        self.topic = "move_action"
        if self.ns != "":
            self.prefix = str(self.ns) + "_"
            self.topic = "/" + str(self.ns) + "/move_action"
        self.action_client = ActionClient(self.node, MoveGroup, self.topic)
        self.action_client.wait_for_server()

    def create_joint_goal(self, joint_positions):
        joint_names = ["elbow_joint","shoulder_lift_joint","shoulder_pan_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
        joint_constraints = []
        for name, pos in zip(joint_names, joint_positions):
            jc = JointConstraint()
            jc.joint_name = self.prefix + name
            jc.position = float(pos)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            joint_constraints.append(jc)
        return joint_constraints

    def create_joint_msg(self, joint_constraints, velocity=0.1, acceleration=0.1):
        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = self.prefix + 'ur_manipulator'
        request.pipeline_id = 'pilz_industrial_motion_planner'
        request.planner_id = "PTP"
        
        request.max_velocity_scaling_factor = velocity
        request.max_acceleration_scaling_factor = acceleration
        
        constraint = Constraints()
        constraint.joint_constraints = joint_constraints
        request.goal_constraints.append(constraint)
        
        goal_msg.request = request
        return goal_msg

    def create_pose_goal(self, frame, x, y, z, ax, ay, az):
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        q = quaternion_from_euler(ax, ay, az)
        msg.pose.orientation.x = q[0]; 
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]; 
        msg.pose.orientation.w = q[3]
        return msg

    def create_pose_msg(self, target_pose: PoseStamped, planner_id: str = "LIN", velocity:float = 0.1, acceleration:float = 0.1) -> MoveGroup.Goal:
        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = self.prefix + 'ur_manipulator'
        request.pipeline_id = 'pilz_industrial_motion_planner'
        request.planner_id = planner_id
        
        request.max_velocity_scaling_factor = velocity
        request.max_acceleration_scaling_factor = acceleration

        # Position Constraint (1mm tolerance sphere)
        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = self.prefix + 'tool0'
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.0001] 
        
        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(primitive)
        bounding_volume.primitive_poses.append(target_pose.pose)
        pos_constraint.constraint_region = bounding_volume
        pos_constraint.weight = 1.0

        # Orientation Constraint (approx 0.05 rad tolerance)
        ori_constraint = OrientationConstraint()
        ori_constraint.header = target_pose.header
        ori_constraint.link_name = self.prefix + 'tool0'
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.01
        ori_constraint.absolute_y_axis_tolerance = 0.01
        ori_constraint.absolute_z_axis_tolerance = 0.01
        ori_constraint.weight = 1.0

        constraint = Constraints()
        constraint.position_constraints.append(pos_constraint)
        constraint.orientation_constraints.append(ori_constraint)
        
        request.goal_constraints.append(constraint)
        goal_msg.request = request
        return goal_msg

    def run_action(self, goal_msg):
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        goal_handle = send_goal_future.result()
        # check if accepted
        if not goal_handle.accepted:
            self.node.get_logger().error('Goal was rejected by MoveIt! Aborting script.')
            return
        # Wait for the trajectory execution to finish
        self.node.get_logger().info('Goal accepted. Waiting for completion...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result
        # Verify Success (MoveItErrorCode 1 == SUCCESS)
        if result.error_code.val != 1:
            self.node.get_logger().error(f'Motion failed with error code: {result.error_code.val}. Aborting script.')
            return
        self.node.get_logger().info('Step completed successfully.\n')

def main(args=None):
    move = Move()

    # Joint Move
    joint_goal = move.create_joint_goal([2.0, -1.6, 4.3, -1.9, -1.6, -0.4])
    joint_msg = move.create_joint_msg(joint_goal, velocity=0.02, acceleration=0.02)
    move.run_action(joint_msg)

    # PTP Move
    pose_goal = move.create_pose_goal(move.prefix + "base", 0.0, 0.81, 0.8, math.pi, 0.0, 0.0)
    pose_msg = move.create_pose_msg(pose_goal, planner_id="PTP", velocity=0.03, acceleration=0.01)
    move.run_action(pose_msg)

    # LIN Move
    pose_goal = move.create_pose_goal(move.prefix + "base", 0.1, 0.8, 0.7, math.pi, 0.0, 0.0)
    pose_msg = move.create_pose_msg(pose_goal, planner_id="LIN", velocity=0.03, acceleration=0.01)
    move.run_action(pose_msg)

if __name__ == '__main__':
    main()
