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

class AdmittancePoseServoExample(Node):
    def __init__(self):
        super().__init__('admittance_pose_servo_example')
        self.declare_parameter('ns', '')
        self.ns = str(self.get_parameter("ns").value) + "/"
        self.publisher_ = self.create_publisher(
            PoseServo, self.ns + 'admittance_pose_servo', 10
        )
        self.pose_goal_client = ActionClient(self, PoseGoal, self.ns + "pose_goal")
        self.pose_goal_client.wait_for_server()

        # Parameters
        self.x = 1.0
        self.y = 0.0
        self.z = 0.8
        self.q = quaternion_from_euler(math.radians(180), math.radians(00), math.radians(-90))
        self.frame_id = ""
        self.target_id = ""

        goal_msg = PoseGoal.Goal()
        goal_msg.target_pose = Pose(
            position=Point(x=self.x, y=self.y, z=self.z),
            orientation=Quaternion(x=self.q[0], y=self.q[1], z=self.q[2], w=self.q[3])
        )
        goal_msg.velocity_scaling = 0.2
        goal_msg.acceleration_scaling = 0.1
        goal_msg.frame_id = self.frame_id
        goal_msg.target_id = self.target_id
        goal_msg.method = "PTP"
        
        # Execution wait until action finishes completely
        self.run_action(self.pose_goal_client, goal_msg)
        time.sleep(0.5)

        # Start Admittance Pose Servo
        self.client = self.create_client(
            StartAdmittancePoseServo,
            self.ns + 'start_admittance_pose_servo'
        )
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service "start_admittance_pose_servo" to become available...')
        self.start_admittance_pose_servo();

        # 100 Hz publishing rate (0.01 seconds period)
        self.timer_period = 0.01
        self.amplitude = 0.10  # Range: -0.1 to 0.1 meters
        self.frequency = 0.10  # 0.5 Hz
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("Starting streaming")

    def start_admittance_pose_servo(self):
        request = StartAdmittancePoseServo.Request()
        # Target Force and Torque
        request.target_wrench.force.x = 0.0
        request.target_wrench.force.y = 0.0
        request.target_wrench.force.z = 0.0
        request.target_wrench.torque.x = 0.0
        request.target_wrench.torque.y = 0.0
        request.target_wrench.torque.z = 0.0
        request.selection_vector =  [True, True, True, False, False, False] 
        # Virtual Mass (Linear: kg | Rotational: kg·m²)
        request.mass = [40.0, 40.0, 40.0, 0.5, 0.5, 0.5]
        # Virtual Damping (Linear: N·s/m | Rotational: N·m·s/rad)
        request.damping = [300.0, 300.0, 300.0, 2.0, 2.0, 2.0]
        # Virtual Stiffness (Linear: N/m | Rotational: N·m/rad)
        request.stiffness = [200.0, 200.0, 200.0, 20.0, 20.0, 20.0]
        # request.stiffness = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # Velocity Limits (Linear: m/s | Rotational: rad/s)
        request.max_velocity = [0.1, 0.1, 0.1, 0.5, 0.5, 0.5]
        # request.max_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        return self.client.call_async(request)

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
        msg = PoseServo()

        # Update Header Stamp and Frames
        msg.frame_id = self.frame_id
        msg.target_id = self.target_id

        # Calculate elapsed time in seconds
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time

        # msg.target_pose.position.y = self.amplitude * math.sin(
        #     2.0 * math.pi * self.frequency * elapsed_time
        # )
        msg.target_pose.position.x = self.x
        msg.target_pose.position.y = self.y
        msg.target_pose.position.z = self.z - self.amplitude * (1 - math.cos(2.0 * math.pi * self.frequency * elapsed_time))/2

        # Set orientation quaternion
        msg.target_pose.orientation.x = self.q[0]
        msg.target_pose.orientation.y = self.q[1]
        msg.target_pose.orientation.z = self.q[2]
        msg.target_pose.orientation.w = self.q[3]

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AdmittancePoseServoExample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

