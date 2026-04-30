import rclpy
import time 
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType
from tf_transformations import quaternion_from_euler
import math as m

class ServoExample(Node):
    def __init__(self):
        super().__init__('servo_example')
        self.declare_parameter('ns', '')
        self.ns = self.get_parameter("ns").value
        self.prefix = ""
        self.topic = ""
        if self.ns != "":
            self.prefix = str(self.ns) + "_"
            self.topic = "/" + str(self.ns)
        self.switch_type = self.create_client(ServoCommandType, self.topic + '/servo_node/switch_command_type')
        self.twist_pub = self.create_publisher(TwistStamped, self.topic + '/servo_node/delta_twist_cmds', 10)
        self.joint_pub = self.create_publisher(JointJog, self.topic + '/servo_node/delta_joint_cmds', 10)
        self.pose_pub  = self.create_publisher(PoseStamped, self.topic + '/servo_node/pose_target_cmds', 10)
            
    def set_mode(self, mode_enum):
        while not self.switch_type.wait_for_service(timeout_sec=1.0): pass
        req = ServoCommandType.Request()
        req.command_type = mode_enum
        future = self.switch_type.call_async(req)
        rclpy.spin_until_future_complete(self, future)

def main(args=None):
    rclpy.init(args=args)
    node = ServoExample()

    ### Twist Example
    node.set_mode(ServoCommandType.Request.TWIST)
    msg = TwistStamped()
    msg.header.frame_id = node.prefix + 'base_link'
    msg.twist.linear.x = 0.05 # move at 0.05 m/s in x direction

    # move for 2s
    for _ in range(20):
        msg.header.stamp = node.get_clock().now().to_msg()
        node.twist_pub.publish(msg)
        rclpy.spin_once(node,timeout_sec=0)
        node.get_logger().info("Ready. Publishing Twist commands...")
        time.sleep(0.1)


    ### Pose Example
    # Comment: the pose command does not support specifying a velocity, you need to do the interpolation yourself
    # Modify and test at your own risk. ROBOT WILL MOVE FULL SPEED TO DESIRED POSITION! BE CAREFUL!

    # node.set_mode(ServoCommandType.Request.POSE)
    # msg = PoseStamped()
    # msg.header.frame_id = 'bed_frame'
    # msg.pose.position.x = 0.7
    # msg.pose.position.y = 0.3
    # msg.pose.position.z = 0.3
    # q = quaternion_from_euler(m.pi, 0, 0)
    # msg.pose.orientation.x = q[0]; 
    # msg.pose.orientation.y = q[1]
    # msg.pose.orientation.z = q[2]; 
    # msg.pose.orientation.w = q[3]
    # for _ in range(20):
    #     msg.header.stamp = node.get_clock().now().to_msg()
    #     node.pose_pub.publish(msg)
    #     rclpy.spin_once(node,timeout_sec=0)
    #     time.sleep(0.01)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
