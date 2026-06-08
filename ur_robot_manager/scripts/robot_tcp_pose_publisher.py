#!/usr/bin/env python3
"""Publish the robot TCP pose as [x, y, z, rx, ry, rz] on /robot_tcp_pose."""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_rotvec(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        return [0.0, 0.0, 0.0]

    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    if qw < 0.0:
        qx = -qx
        qy = -qy
        qz = -qz
        qw = -qw

    sin_half = math.sqrt(qx * qx + qy * qy + qz * qz)
    if sin_half <= 1e-12:
        return [0.0, 0.0, 0.0]

    angle = 2.0 * math.atan2(sin_half, qw)
    scale = angle / sin_half
    return [qx * scale, qy * scale, qz * scale]


class RobotTcpPosePublisher(Node):
    def __init__(self):
        super().__init__("robot_tcp_pose_publisher")

        self.declare_parameter("base_frame", "base")
        self.declare_parameter("tool_frame", "tool0")
        self.declare_parameter("topic_name", "/robot_tcp_pose")
        self.declare_parameter("rate", 100.0)

        self.base_frame = self.get_parameter("base_frame").value
        self.tool_frame = self.get_parameter("tool_frame").value
        self.topic_name = self.get_parameter("topic_name").value
        rate = float(self.get_parameter("rate").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(Float32MultiArray, self.topic_name, 1)
        self.last_warning_time = 0.0

        self.create_timer(1.0 / max(rate, 1.0), self.publish_tcp_pose)
        self.get_logger().info(
            f"Publishing {self.base_frame} -> {self.tool_frame} on {self.topic_name}"
        )

    def publish_tcp_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tool_frame,
                Time(),
            )
        except TransformException as exc:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self.last_warning_time >= 1.0:
                self.get_logger().warning(
                    f"Could not lookup {self.base_frame} -> {self.tool_frame}: {exc}"
                )
                self.last_warning_time = now
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rotvec = quaternion_to_rotvec(rotation.x, rotation.y, rotation.z, rotation.w)

        msg = Float32MultiArray()
        msg.data = [
            float(translation.x),
            float(translation.y),
            float(translation.z),
            float(rotvec[0]),
            float(rotvec[1]),
            float(rotvec[2]),
        ]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotTcpPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
