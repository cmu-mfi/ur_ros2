import math
import rclpy
import time
from rclpy.node import Node
from rclpy.action.client import ActionClient
from action_msgs.msg import GoalStatus
from robot_manager_interfaces.action import PoseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from tf_transformations import quaternion_from_euler
from robot_manager_interfaces.msg import PoseServo
from robot_manager_interfaces.srv import StartAdmittancePoseServo
from std_srvs.srv import Trigger

class AdmittanceParameterTuning(Node):
    def __init__(self):
        super().__init__('admittance_parameter_tuning')
        self.declare_parameter('ns', 'ur20')
        self.ns = str(self.get_parameter("ns").value) + "/"

        # Publisher for admittance pose target stream
        self.publisher_ = self.create_publisher(
            PoseServo, self.ns + 'admittance_pose_servo', 10
        )

        # Action Client for MoveIt PTP initial positioning
        self.pose_goal_client = ActionClient(self, PoseGoal, self.ns + "pose_goal")
        self.pose_goal_client.wait_for_server()

        # Service Clients for starting and stopping admittance controller
        self.start_client = self.create_client(
            StartAdmittancePoseServo,
            self.ns + 'start_admittance_pose_servo'
        )
        self.stop_client = self.create_client(
            Trigger,
            self.ns + 'stop_admittance_pose_servo'
        )

        while not self.start_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service "start_admittance_pose_servo" to become available...')
        while not self.stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service "stop_admittance_pose_servo" to become available...')

        # Base Pose Parameters
        self.x = 1.0
        self.y = 0.0
        self.z = 0.345
        self.q = quaternion_from_euler(math.radians(160), math.radians(0), math.radians(-90))
        self.frame_id = "ur20_base_link"
        self.target_id = "ur20_tool0"

        # Move arm to initial position prior to experiments
        self.move_to_initial_pose()

    def move_to_initial_pose(self):
        goal_msg = PoseGoal.Goal()
        goal_msg.target_pose = Pose(
            position=Point(x=self.x, y=self.y, z=self.z),
            orientation=Quaternion(x=self.q[0], y=self.q[1], z=self.q[2], w=self.q[3])
        )
        goal_msg.velocity_scaling = 0.1
        goal_msg.acceleration_scaling = 0.1
        goal_msg.frame_id = self.frame_id
        goal_msg.target_id = self.target_id
        goal_msg.method = "PTP"
        
        # Execution wait until action finishes completely
        self.run_action(self.pose_goal_client, goal_msg)
        time.sleep(0.5)

    def start_admittance_pose_servo(self, mass: float, stiffness: float, damping: float):
        request = StartAdmittancePoseServo.Request()
        request.target_wrench.force.x = 0.0
        request.target_wrench.force.y = 0.0
        request.target_wrench.force.z = 15.0
        request.target_wrench.torque.x = 0.0
        request.target_wrench.torque.y = 0.0
        request.target_wrench.torque.z = 0.0
        request.selection_vector =  [False, False, True, False, False, False] 
        request.mass =              [0.0, 0.0, float(mass),      0.0, 0.0, 0.0]                 
        request.damping =           [0.0, 0.0, float(damping),   0.0, 0.0, 0.0]
        request.stiffness =         [0.0, 0.0, float(stiffness), 0.0, 0.0, 0.0]   
        request.max_velocity =      [0.0, 0.0, 0.1,              0.0, 0.0, 0.0]        
        request.max_acceleration =  [0.0, 0.0, 0.5,              0.0, 0.0, 0.0]   

        future = self.start_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response and response.success:
            self.get_logger().info(f"Started admittance controller (M={mass}, K={stiffness}, B={damping})")
        else:
            self.get_logger().error(f"Failed to start admittance controller: {response.message if response else 'No response'}")
        return response

    def stop_admittance_pose_servo(self):
        request = Trigger.Request()
        future = self.stop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response and response.success:
            self.get_logger().info("Stopped admittance controller.")
        else:
            self.get_logger().error(f"Failed to stop admittance controller: {response.message if response else 'No response'}")
        return response

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

    def publish_pose(self, z_target: float):
        msg = PoseServo()
        msg.frame_id = self.frame_id
        msg.target_id = self.target_id

        msg.target_pose.position.x = self.x
        msg.target_pose.position.y = self.y
        msg.target_pose.position.z = z_target

        msg.target_pose.orientation.x = self.q[0]
        msg.target_pose.orientation.y = self.q[1]
        msg.target_pose.orientation.z = self.q[2]
        msg.target_pose.orientation.w = self.q[3]

        self.publisher_.publish(msg)

    def execute_sigmoid_motion(self, start_z: float, end_z: float, duration: float, rate_hz: float = 100.0, k_slope: float = 6.0):
        period = 1.0 / rate_hz
        start_time = time.time()

        # Precompute constants for normalized sigmoid mapping [0, 1]
        s0 = 1.0 / (1.0 + math.exp(0.5 * k_slope))
        s1 = 1.0 / (1.0 + math.exp(-0.5 * k_slope))

        while rclpy.ok():
            elapsed = time.time() - start_time
            if elapsed >= duration:
                self.publish_pose(end_z)
                break

            # Normalized progression ratio u in [0, 1]
            u = elapsed / duration
            s_u = 1.0 / (1.0 + math.exp(-k_slope * (u - 0.5)))
            norm_sig = (s_u - s0) / (s1 - s0)

            # Map sigmoid output to target z height
            current_z = start_z + (end_z - start_z) * norm_sig
            self.publish_pose(current_z)

            rclpy.spin_once(self, timeout_sec=0.001)
            time.sleep(period)

    def hold_position(self, z_target: float, duration: float, rate_hz: float = 100.0):
        period = 1.0 / rate_hz
        start_time = time.time()

        while rclpy.ok():
            elapsed = time.time() - start_time
            if elapsed >= duration:
                break

            self.publish_pose(z_target)
            rclpy.spin_once(self, timeout_sec=0.001)
            time.sleep(period)


def main(args=None):
    rclpy.init(args=args)
    node = AdmittanceParameterTuning()

    # Parameter test matrix for tuning experiments: (Mass, Stiffness, Damping)
    experiments = [
        {'m': 300.0, 'k': 0000.01, 'b': 5000.0},
    ]
    start_height = node.z             # Initial z height (0.41 m)
    end_height = node.z - 0.00        # Target z height (0.31 m)
    motion_duration = 2.0             # Sigmoid transition time (seconds)
    hold_duration = 3.0               # Dwell time at target depth (seconds)

    try:
        for idx, exp in enumerate(experiments, start=1):
            m, k, b = exp['m'], exp['k'], exp['b']
            node.get_logger().info(f"=== Starting Experiment {idx}/{len(experiments)}: M={m}, K={k}, B={b} ===")

            # 1. Start Admittance Pose Servo with experiment parameters
            node.start_admittance_pose_servo(mass=m, stiffness=k, damping=b)
            time.sleep(0.5)

            # 2. Sigmoid trajectory: start_height -> end_height
            node.get_logger().info(f"Executing sigmoid motion down to {end_height:.3f} m...")
            node.execute_sigmoid_motion(start_z=start_height, end_z=end_height, duration=motion_duration)

            # 3. Dwell at end height for 3 seconds
            node.get_logger().info(f"Holding at target height for {hold_duration} seconds...")
            node.hold_position(z_target=end_height, duration=hold_duration)

            # 4. Sigmoid trajectory: end_height -> start_height
            node.get_logger().info(f"Executing sigmoid motion back to {start_height:.3f} m...")
            node.execute_sigmoid_motion(start_z=end_height, end_z=start_height, duration=motion_duration)

            # 5. Stop Admittance Pose Servo
            time.sleep(0.5)
            node.stop_admittance_pose_servo()
            time.sleep(0.5)
            # 6. Move back to start
            node.move_to_initial_pose()

        node.get_logger().info("All parameter tuning experiments completed successfully.")

    except KeyboardInterrupt:
        node.get_logger().info("Tuning experiment sequence interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
