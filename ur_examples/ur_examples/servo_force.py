import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, WrenchStamped
from moveit_msgs.srv import ServoCommandType
import pyspacemouse
import time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations
import threading
import numpy as np
import tf_transformations

class ProbeNode(Node):
    def __init__(self, ns):
        super().__init__('probe_node')
        self.ns = ns
        self.frame_id = self.ns + "_tool0"
        self.force = 0.0
        self.force_goal = 10.0
        self.offset = 0.0
        self.force_mode = False

        # Force Subscriber
        self.force_subscriber = self.create_subscription(
            WrenchStamped, 
            ns + '/force_torque_sensor_broadcaster/wrench_filtered',
            self.force_callback, 
            10
        )

        # Twist Publisher
        self.twist_publisher = self.create_publisher(
            TwistStamped, 
            self.ns + '/servo_node/delta_twist_cmds', 
            10
        )

        # Enable Servo
        self.servo_client = self.create_client(ServoCommandType, self.ns + '/servo_node/switch_command_type')
        while not self.servo_client.wait_for_service(timeout_sec=1.0): pass
        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.TWIST
        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        # Spacemouse
        self.current_t = 0.0
        self.previous_t = 0.0
        self.skipped_ts = 0
        self.speed = 0.2
        self.turn = 0.8
        self.current_buttons = [0,0]
        self.previous_buttons = [0,0]
        self.left_button_pressed = False
        self.right_button_pressed = False
        self.state = None
        success = pyspacemouse.open()
        if success:
            self.get_logger().info("Space Mouse connected succesfully!")
        else:
            self.get_logger().error("Failed to connect to Space Mouse!")
            exit()
        self.spacemouse_loop = self.create_timer((1 / 200), self.spacemouse_loop_callback)

    #### Spacemouse
    def spacemouse_loop_callback(self):
        self.state = pyspacemouse.read()
        if type(self.state) == pyspacemouse.pyspacemouse.SpaceNavigator:
            self.current_buttons = [self.state.buttons[0], self.state.buttons[14]]
            if self.current_buttons[0] == 1 and self.previous_buttons[0] == 0:
                self.left_button_pressed = True
            if self.current_buttons[1] == 1 and self.previous_buttons[1] == 0:
                self.right_button_pressed = True
            self.previous_buttons = self.current_buttons
    def navigate(self):
        x = 0
        y = 0
        z = 0
        if self.force_mode:
            z = 0.01 * (self.force - self.force_goal)
            if z > 0.1:
                z = 0.1
            elif z < -0.1:
                z = -0.1
        if type(self.state) == pyspacemouse.pyspacemouse.SpaceNavigator:
            # self.current_t = self.state.t
            # diff = abs(self.current_t - self.previous_t)
            # if diff == 0.0:
            #     self.skipped_ts += 1
            # else:
            #     self.skipped_ts = 0
            # if self.skipped_ts >= 10:
            #     self.stop()
            #     return
            # self.previous_t = self.current_t
            x = self.state.x
            y = self.state.y
            if not self.force_mode:
                z = self.state.z
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.frame_id
        twist_msg.twist.linear.x = x * self.speed
        twist_msg.twist.linear.y = -y * self.speed
        twist_msg.twist.linear.z = -z * self.speed
        self.twist_publisher.publish(twist_msg)

    def stop(self):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.frame_id
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0
        self.twist_publisher.publish(twist_msg)

    #### Force Sensor
    def force_callback(self, msg: WrenchStamped):
        self.force = abs(abs(msg.wrench.force.z) - abs(self.offset))
    def calibrate_sensor(self):
        self.get_logger().info("Starting sensor calibration")
        self.offset = 0.0
        data = []
        time.sleep(0.1)
        for _ in range(300):
            data.append(self.force)
            time.sleep(0.01)
        self.offset = sum(data) / len(data)
        time.sleep(0.1)
        self.get_logger().info(f"Calibration finished! Offset: {self.offset}")

def main(args=None):
    rclpy.init(args=args)
    ns = "ur20"
    node = ProbeNode(ns)
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        while True:
            if node.right_button_pressed:
                node.right_button_pressed = not node.right_button_pressed
                node.force_mode = not node.force_mode
                if node.force_mode:
                    node.calibrate_sensor()
            
            node.navigate()
            time.sleep(0.01)

    except KeyboardInterrupt:
        print('Keyboard interrupt, shutting down.')
    finally:
        node.stop()
        print("node stopped")
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
