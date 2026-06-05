import rclpy
from rclpy.node import Node
from robot_manager_interfaces.srv import SetPayload

class PayloadClient(Node):
    def __init__(self):
        super().__init__('payload_client_node')
        self.declare_parameter('ns', '')
        self.ns = self.get_parameter("ns").value
        self.topic = ""
        if self.ns != "":
            self.topic = "/" + str(self.ns)
        self.payload_client = self.create_client(SetPayload, self.topic + '/set_payload')
        while not self.payload_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SetPayload.Request()

    def send_request(self, mass, x, y, z):
        self.req.mass = mass
        self.req.cog.x = x
        self.req.cog.y = y
        self.req.cog.z = z
        
        self.get_logger().info(f'Sending request: Mass={mass}kg, CoG=[{x}, {y}, {z}]')
        
        self.future = self.payload_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()
    
    node = PayloadClient()
    
    response = node.send_request(1.46, -0.015, 0.014, 0.016)
    
    if response.success:
        node.get_logger().info('Successfully updated payload!')
    else:
        node.get_logger().error('Failed to update payload.')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
