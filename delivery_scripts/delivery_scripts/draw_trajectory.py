#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from math import hypot

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy
)


class TrajectoryDrawer(Node):

    def __init__(self):
        super().__init__('trajectory_drawer')

        self.declare_parameter(
            'odom_topic',
            '/mecanum_controller/odom'
        )
        self.declare_parameter(
            'max_poses',
            5000
        )
        self.declare_parameter(
            'min_distance',
            0.02
        )

        self.odom_topic = self.get_parameter(
            'odom_topic'
        ).value

        self.max_poses = self.get_parameter(
            'max_poses'
        ).value

        self.min_distance = self.get_parameter(
            'min_distance'
        ).value

        self.last_x = None
        self.last_y = None

        self.path_msg = Path()

        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            odom_qos
        )

        self.path_pub = self.create_publisher(
            Path,
            '/mecanum_controller/trajectory',
            path_qos
        )

        self.clear_srv = self.create_service(
            Empty,
            '/clear_trajectory',
            self.clear_callback
        )

        self.get_logger().info(
            f'Subscribed to: {self.odom_topic}'
        )

        self.get_logger().info(
            'Publishing trajectory on: '
            '/mecanum_controller/trajectory'
        )

    def clear_callback(self, request, response):

        self.path_msg.poses.clear()

        self.last_x = None
        self.last_y = None

        self.path_pub.publish(self.path_msg)

        self.get_logger().info(
            'Trajectory cleared'
        )

        return response

    def odom_callback(self, msg: Odometry):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_x is not None:

            distance = hypot(
                x - self.last_x,
                y - self.last_y
            )

            if distance < self.min_distance:
                return

        self.last_x = x
        self.last_y = y

        pose = PoseStamped()

        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = msg.header.frame_id
        pose.pose = msg.pose.pose

        self.path_msg.header.stamp = msg.header.stamp
        self.path_msg.header.frame_id = msg.header.frame_id

        self.path_msg.poses.append(pose)

        excess = len(self.path_msg.poses) - self.max_poses

        if excess > 0:
            del self.path_msg.poses[:excess]

        self.path_pub.publish(self.path_msg)


def main(args=None):

    rclpy.init(args=args)

    node = TrajectoryDrawer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()