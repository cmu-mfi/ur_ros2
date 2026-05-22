import rclpy
from rclpy.action.client import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, JointConstraint
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from shape_msgs.msg import SolidPrimitive
import math
from rclpy.node import Node

class JointGoal:
    def __init__(self, joint_names=None, joint_positions=None, velocity=None, acceleration=None):
        self.joint_names = joint_names if joint_names is not None else []
        self.joint_positions = joint_positions if joint_positions is not None else []
        self.velocity = velocity
        self.acceleration = acceleration
        if len(self.joint_names) != len(self.joint_positions):
            raise ValueError("The number of joint names must match the number of positions!")
        if self.velocity == None:
            raise ValueError("Velocity needs to be specified!")
        if self.velocity == None:
            raise ValueError("Acceleration needs to be specified!")

class PoseGoal:
    def __init__(self, frame_id=None, position=None, orientation=None, velocity=None, acceleration=None, method="PTP"):
        if position == None or len(position) != 3:
            raise ValueError("Position should be 3 values (ex: [0.0, 0.0, 0.0])")
        if orientation == None or len(orientation) != 3:
            raise ValueError("orientation should be 3 values (ex: [0.0, 0.0, 0.0])")
        if velocity == None:
            raise ValueError("Velocity needs to be specified!")
        if velocity == None:
            raise ValueError("Acceleration needs to be specified!")
        if frame_id == None:
            raise ValueError("frame_id needs to be specified!")
        self.position = position
        self.orientation = orientation
        self.frame_id = frame_id
        self.velocity = velocity
        self.acceleration = acceleration
        self.method = method

class Robot(Node):
    def __init__(self):
        super().__init__("robot")
        self.declare_parameter('ns', '')
        self.ns = self.get_parameter("ns").value
        self.prefix = ""
        self.topic = "move_action"
        if self.ns != "":
            self.prefix = str(self.ns) + "_"
            self.topic = "/" + str(self.ns) + "/move_action"
        else:
            self.prefix = ""
            self.topic = "/move_action"

        self.action_client = ActionClient(self, MoveGroup, self.topic)
        self.action_client.wait_for_server()

    def create_joint_action(self, goal: JointGoal):
        joint_constraints = []
        for name, pos in zip(goal.joint_names, goal.joint_positions):
            jc = JointConstraint()
            jc.joint_name = self.prefix + name
            jc.position = float(pos)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            joint_constraints.append(jc)
        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = self.prefix + 'manipulator'
        request.pipeline_id = 'pilz_industrial_motion_planner'
        request.planner_id = "PTP"
        request.max_velocity_scaling_factor = goal.velocity
        request.max_acceleration_scaling_factor = goal.acceleration
        constraint = Constraints()
        constraint.joint_constraints = joint_constraints
        request.goal_constraints.append(constraint)
        goal_msg.request = request
        return goal_msg

    def create_pose_action(self, goal: PoseGoal) -> MoveGroup.Goal:
        target_pose = PoseStamped()
        target_pose.header.frame_id = goal.frame_id
        target_pose.pose.position.x = goal.position[0]
        target_pose.pose.position.y = goal.position[1]
        target_pose.pose.position.z = goal.position[2]
        q = quaternion_from_euler(goal.orientation[0], goal.orientation[1], goal.orientation[2])
        target_pose.pose.orientation.x = q[0]; 
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]; 
        target_pose.pose.orientation.w = q[3]
        goal_msg = MoveGroup.Goal()

        request = MotionPlanRequest()
        request.group_name = self.prefix + 'manipulator'
        request.pipeline_id = 'pilz_industrial_motion_planner'
        request.planner_id = goal.method
        
        request.max_velocity_scaling_factor = goal.velocity
        request.max_acceleration_scaling_factor = goal.acceleration

        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = self.prefix + "tool0"
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.0001] 
        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(primitive)
        bounding_volume.primitive_poses.append(target_pose.pose)
        pos_constraint.constraint_region = bounding_volume
        pos_constraint.weight = 1.0
        ori_constraint = OrientationConstraint()
        ori_constraint.header = target_pose.header
        ori_constraint.link_name = self.prefix + "tool0"
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
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        # check if accepted
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by MoveIt! Aborting script.')
            return
        # Wait for the trajectory execution to finish
        self.get_logger().info('Goal accepted. Waiting for completion...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        # Verify Success (MoveItErrorCode 1 == SUCCESS)
        if result.error_code.val != 1:
            self.get_logger().error(f'Motion failed with error code: {result.error_code.val}. Aborting script.')
            return
        self.get_logger().info('Step completed successfully.\n')

def main(args=None):
    rclpy.init(args=None)

    ur20 = Robot()

    # Joint Move
    goal = JointGoal(
        joint_names=["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"],
        joint_positions=[0.0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0.0],
        velocity=0.1,
        acceleration=0.1,
    )
    joint_action = ur20.create_joint_action(goal)
    ur20.run_action(joint_action)

    # PTP Move
    goal = PoseGoal(
            frame_id="ur20_base_link",
            position=[0.6, 0.2, 0.7],
            orientation=[math.pi, -0.2, -math.pi/2],
            velocity=0.1,
            acceleration=0.1,
            method="PTP"
            )
    pose_action = ur20.create_pose_action(goal)
    ur20.run_action(pose_action)

    # LIN Move
    goal = PoseGoal(
            frame_id="ur20_base_link",
            position=[0.6, -0.2, 0.7],
            orientation=[math.pi, 0.2, -math.pi/2],
            velocity=0.1,
            acceleration=0.1,
            method="PTP"
            )
    pose_action = ur20.create_pose_action(goal)
    ur20.run_action(pose_action)

    # Joint Move
    goal = JointGoal(
        joint_names=["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"],
        joint_positions=[0.0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0.0],
        velocity=0.1,
        acceleration=0.1,
    )
    joint_action = ur20.create_joint_action(goal)
    ur20.run_action(joint_action)

    ur20.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
 
