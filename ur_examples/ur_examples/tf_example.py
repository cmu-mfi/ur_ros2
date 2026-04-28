import rclpy
from tf2_ros import Buffer, TransformListener

def main():
    rclpy.init()
    node = rclpy.create_node('tf_example')
    node.declare_parameter('ns', '')
    ns = node.get_parameter("ns").value
    prefix = ""
    if ns != "":
        prefix = str(ns) + "_"
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    while rclpy.ok():
        rclpy.spin_once(node)
        try:
            t = tf_buffer.lookup_transform(prefix + 'base', prefix + 'tool0', rclpy.time.Time())
            data = {
                'x': t.transform.translation.x, 'y': t.transform.translation.y, 'z': t.transform.translation.z,
                'qx': t.transform.rotation.x, 'qy': t.transform.rotation.y, 'qz': t.transform.rotation.z, 'qw': t.transform.rotation.w
            }
            print(data)
            break
        except:
            pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
