import rclpy
from rclpy.action.client import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, JointConstraint
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from shape_msgs.msg import SolidPrimitive
import math
from std_srvs.srv import Trigger
from ur_msgs.srv import SetForceMode
import time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class UR():
    def __init__(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node('UR')
        self.node.declare_parameter('ns', '')
        self.ns = self.node.get_parameter("ns").value
        self.prefix = ""
        self.topic = ""
        if self.ns != "":
            self.prefix = str(self.ns) + "_"
            self.topic = "/" + str(self.ns)
        self.action_client = ActionClient(self.node, MoveGroup, self.topic + "/move_action")
        self.force_start_client = self.node.create_client(SetForceMode, self.topic + '/force_mode_controller/start_force_mode')
        self.force_stop_client = self.node.create_client(Trigger, self.topic + '/force_mode_controller/stop_force_mode')
        self.action_client.wait_for_server()
        self.force_start_client.wait_for_service()
        self.force_stop_client.wait_for_service()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

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
        request.group_name = self.prefix + 'manipulator'
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
        msg.pose.orientation.x = q[0] 
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2] 
        msg.pose.orientation.w = q[3]
        return msg

    def create_pose_msg(self, target_pose: PoseStamped, planner_id: str = "LIN", velocity:float = 0.1, acceleration:float = 0.1) -> MoveGroup.Goal:
        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = self.prefix + 'manipulator'
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

    def start_force_mode(self):
        req = SetForceMode.Request()
        # 1. Task Frame (PoseStamped)
        req.task_frame.header.frame_id = self.prefix + "base_link"
        req.task_frame.pose.position.x = 0.0
        req.task_frame.pose.position.y = 0.0
        req.task_frame.pose.position.z = 0.0
        req.task_frame.pose.orientation.w = 1.0
        # q = quaternion_from_euler(0.0, 0.0, 0.0)
        # req.task_frame.pose.orientation.x = q[0] 
        # req.task_frame.pose.orientation.y = q[1]
        # req.task_frame.pose.orientation.z = q[2] 
        # req.task_frame.pose.orientation.w = q[3]
        # 2. Selection Vector
        req.selection_vector_x = False
        req.selection_vector_y = False
        req.selection_vector_z = True
        req.selection_vector_rx = False
        req.selection_vector_ry = False
        req.selection_vector_rz = False

        # 3. Wrench (geometry_msgs/Wrench)
        req.wrench.force.x = 0.0
        req.wrench.force.y = 0.0
        req.wrench.force.z = -4.0  # Apply 1 Newtons in the Z-axis
        req.wrench.torque.x = 0.0
        req.wrench.torque.y = 0.0
        req.wrench.torque.z = 0.0

        # 4. Type (Integer 1-3)
        # 1: y-axis aligned with vector from TCP to force frame origin
        # 2: force frame is not transformed
        # 3: x-axis is projection of TCP velocity vector onto x-y plane of force frame
        req.type = 2

        # 5. Speed Limits (geometry_msgs/Twist)
        # Maximum allowed TCP speed relative to the task frame for compliant axes.
        req.speed_limits.linear.x = 0.3
        req.speed_limits.linear.y = 0.3
        req.speed_limits.linear.z = 0.3
        req.speed_limits.angular.x = 0.1
        req.speed_limits.angular.y = 0.1
        req.speed_limits.angular.z = 0.1

        # 6. Deviation Limits 
        # Maximum allowed deviation along/about an axis for non-compliant axes.
        # (Usually represented as an array of 6 floats depending on the exact ur_msgs structure)
        req.deviation_limits = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

        # 7. Damping Factor and Gain Scaling
        req.damping_factor = 0.025 # Range [0;1], 1 is full damping. Default is 0.025
        req.gain_scaling = 0.5     # Range [0;2]. Default is 0.5. >1 can make it unstable.

        self.node.get_logger().info('Calling start_force_mode...')
        future = self.force_start_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        
        try:
            response = future.result()
            if response.success:
                self.node.get_logger().info('Successfully started Force Mode.')
            else:
                self.node.get_logger().error('Failed to start Force Mode.')
        except Exception as e:
            self.node.get_logger().error(f'Service call failed: {e}')

    def stop_force_mode(self):
        """
        Calls the stop_force_mode service (std_srvs/Trigger).
        """
        req = Trigger.Request()
        
        self.node.get_logger().info('Calling stop_force_mode...')
        future = self.force_stop_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        
        try:
            response = future.result()
            if response.success:
                self.node.get_logger().info('Successfully stopped Force Mode.')
            else:
                self.node.get_logger().error(f'Failed to stop Force Mode: {response.message}')
        except Exception as e:
            self.node.get_logger().error(f'Service call failed: {e}')

    def get_z(self):
        while rclpy.ok():
            rclpy.spin_once(self.node)
            try:
                t = self.tf_buffer.lookup_transform(self.prefix + 'base', self.prefix + 'tool0', rclpy.time.Time())
                return t.transform.translation.z
            except:
                pass


def main(args=None):
    ur = UR()

    # For this to work the following modifications have to be made:
    # - switch from joint_trajectory_controller to passthrough_trajectory_controller
    # - change moveit_controllers.yaml config file

    ur.stop_force_mode()
    pose_goal = ur.create_pose_goal(ur.prefix + "base", 0.75, 0.8, 0.4, math.pi, 0.0, 0.0)
    pose_msg = ur.create_pose_msg(pose_goal, planner_id="LIN", velocity=0.05, acceleration=0.01)
    ur.run_action(pose_msg)
    ur.start_force_mode()
    try:
        while True:
            pose_goal = ur.create_pose_goal(ur.prefix + "base", 0.75, 0.9, ur.get_z(), math.pi, 0.0, 0.0)
            pose_msg = ur.create_pose_msg(pose_goal, planner_id="LIN", velocity=0.05, acceleration=0.03)
            ur.run_action(pose_msg)
            pose_goal = ur.create_pose_goal(ur.prefix + "base", 0.75, 0.8, ur.get_z(), math.pi, 0.0, 0.0)
            pose_msg = ur.create_pose_msg(pose_goal, planner_id="LIN", velocity=0.05, acceleration=0.03)
            ur.run_action(pose_msg)
    except KeyboardInterrupt:
        ur.stop_force_mode()
        print("Stop")

if __name__ == '__main__':
    main()
