#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
import math


class SquarePathPublisher(Node):

    def __init__(self):
        super().__init__('square_path_publisher')

        self.publisher_ = self.create_publisher(Path, '/plan', 10)
        self.timer = self.create_timer(1.0, self.publish_path)

        self.get_logger().info("Square Path Publisher Started")

    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    def publish_path(self):
        path = Path()
        path.header = Header()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        # Square size (meters)
        size = 1.0

        # Define square corners
        points = [
            (0.0, 0.0, 0.0),
            (size, 0.0, 0.0),
            (size, size, math.pi/2),
            (0.0, size, math.pi),
            (0.0, 0.0, -math.pi/2)
        ]

        for x, y, yaw in points:
            pose = self.create_pose(x, y, yaw)
            path.poses.append(pose)

        self.publisher_.publish(path)
        self.get_logger().info("Published square path")


def main(args=None):
    rclpy.init(args=args)
    node = SquarePathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()