#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import FollowPath
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

import math


class FollowSquarePathClient(Node):

    def __init__(self):
        super().__init__('follow_square_path_client')

        self._action_client = ActionClient(self, FollowPath, '/follow_path')

        self.get_logger().info('Waiting for Nav2 FollowPath action server...')
        self._action_client.wait_for_server()

        self.send_goal()

    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y

        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    def create_square_path(self, size=1.0):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        points = [
            (0.0, 0.0, 0.0),
            (size, 0.0, 0.0),
            (size, size, math.pi/2),
            (0.0, size, math.pi),
            (0.0, 0.0, -math.pi/2),
        ]

        for x, y, yaw in points:
            path.poses.append(self.create_pose(x, y, yaw))

        return path

    def send_goal(self):
        goal_msg = FollowPath.Goal()

        goal_msg.path = self.create_square_path(size=1.0)
        goal_msg.controller_id = ''   # use default controller
        goal_msg.goal_checker_id = '' # use default goal checker

        self.get_logger().info('Sending path to Nav2...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted!')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_to_goal:.2f}')

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Path execution completed!')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FollowSquarePathClient()
    rclpy.spin(node)


if __name__ == '__main__':
    main()