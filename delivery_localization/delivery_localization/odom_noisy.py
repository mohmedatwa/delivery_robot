#!/usr/bin/env python3


import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


def quaternion_to_yaw(q: Quaternion) -> float:
    """Convert quaternion to yaw (2D)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert yaw (2D) to quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class NoisyOdom(Node):
    def __init__(self):
        super().__init__('noisy_odom')

        # ---------------- Parameters ----------------
        self.declare_parameter('input_odom', '/mecanum_controller/odom')
        self.declare_parameter('output_odom', '/odom_noisy')

        # Position noise (meters)
        self.declare_parameter('noise_x_std', 0.01)
        self.declare_parameter('noise_y_std', 0.01)
        self.declare_parameter('noise_z_std', 0.0)

        # Orientation noise (radians)
        self.declare_parameter('noise_yaw_std', 0.005)

        # Velocity noise (m/s, rad/s)
        self.declare_parameter('noise_vx_std', 0.01)
        self.declare_parameter('noise_vy_std', 0.01)
        self.declare_parameter('noise_wz_std', 0.005)

        # Covariance handling
        self.declare_parameter('scale_covariance', True)
        self.declare_parameter('covariance_scale', 2.0)

        self.input_odom = self.get_parameter('input_odom').value
        self.output_odom = self.get_parameter('output_odom').value

        # ---------------- ROS interfaces ----------------
        self.sub = self.create_subscription(
            Odometry,
            self.input_odom,
            self.odom_callback,
            qos_profile_sensor_data
        )

        self.pub = self.create_publisher(
            Odometry,
            self.output_odom,
            10
        )

        self.get_logger().info(
            f'Noisy odom running : {self.input_odom} -> {self.output_odom}'
        )

    @staticmethod
    def gaussian(std: float) -> float:
        return random.gauss(0.0, std) if std > 0.0 else 0.0

    def odom_callback(self, msg: Odometry):
        noisy = Odometry()
        noisy.header = msg.header
        noisy.child_frame_id = msg.child_frame_id

        # ---------------- Pose ----------------
        noisy.pose.pose.position.x = msg.pose.pose.position.x + self.gaussian(
            self.get_parameter('noise_x_std').value
        )
        noisy.pose.pose.position.y = msg.pose.pose.position.y + self.gaussian(
            self.get_parameter('noise_y_std').value
        )
        noisy.pose.pose.position.z = msg.pose.pose.position.z + self.gaussian(
            self.get_parameter('noise_z_std').value
        )

        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        yaw += self.gaussian(self.get_parameter('noise_yaw_std').value)
        noisy.pose.pose.orientation = yaw_to_quaternion(yaw)

        # ---------------- Twist (mecanum important) ----------------
        noisy.twist.twist.linear.x = msg.twist.twist.linear.x + self.gaussian(
            self.get_parameter('noise_vx_std').value
        )
        noisy.twist.twist.linear.y = msg.twist.twist.linear.y + self.gaussian(
            self.get_parameter('noise_vy_std').value
        )
        noisy.twist.twist.linear.z = msg.twist.twist.linear.z

        noisy.twist.twist.angular.x = msg.twist.twist.angular.x
        noisy.twist.twist.angular.y = msg.twist.twist.angular.y
        noisy.twist.twist.angular.z = msg.twist.twist.angular.z + self.gaussian(
            self.get_parameter('noise_wz_std').value
        )

        # ---------------- Covariances ----------------
        noisy.pose.covariance = list(msg.pose.covariance)
        noisy.twist.covariance = list(msg.twist.covariance)

        if self.get_parameter('scale_covariance').value:
            scale = float(self.get_parameter('covariance_scale').value)
            noisy.pose.covariance = [c * scale for c in noisy.pose.covariance]
            noisy.twist.covariance = [c * scale for c in noisy.twist.covariance]

        self.pub.publish(noisy)


def main(args=None):
    rclpy.init(args=args)
    node = NoisyOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
