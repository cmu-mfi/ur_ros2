import rclpy
import time
from ur_msgs.srv import SetIO

class Io():
    def __init__(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node('io_example')
        self.node.declare_parameter('ns', '')
        self.ns = self.node.get_parameter("ns").value
        self.topic = ""
        if self.ns != "":
            self.topic = "/" + str(self.ns)
        self.io_client = self.node.create_client(SetIO, self.topic + '/io_and_status_controller/set_io')
        while not self.io_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting...')

def main(args=None):
    io = Io()
    req = SetIO.Request()
    req.fun = 1 # 1 corresponds to SET_DIGITAL_OUT
    req.pin = 1 # 1 for do1
    req.state = 1.0 # 1.0 for ON, 0.0 for OFF
    
    # Send the async request
    future = io.io_client.call_async(req)
    rclpy.spin_until_future_complete(io.node, future)

    time.sleep(1)

    req.state = 0.0 # 1.0 for ON, 0.0 for OFF
    future = io.io_client.call_async(req)
    rclpy.spin_until_future_complete(io.node, future)

if __name__ == '__main__':
    main()
