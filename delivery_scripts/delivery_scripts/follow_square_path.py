#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException

from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

from action_msgs.msg import GoalStatus


class FollowSquarePath(Node):

    def __init__(self):
        super().__init__("follow_square_path")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.client = ActionClient(
            self,
            FollowPath,
            "/follow_path"
        )

        self.get_logger().info("Waiting for FollowPath server and TF...")

        self.create_timer(1.0, self.send_once)

        self.sent = False

    def send_once(self):

        if self.sent:
            return

        if not self.client.server_is_ready():

            self.get_logger().info("Waiting for FollowPath server...")
            return

        try:

            tf = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time()
            )

        except TransformException as ex:

            self.get_logger().warn(str(ex))
            return

        self.sent = True

        x0 = tf.transform.translation.x
        y0 = tf.transform.translation.y

        q = tf.transform.rotation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.get_logger().info(
            f"Robot pose : {x0:.2f} {y0:.2f} yaw={yaw:.2f}"
        )

        path = self.make_square(
            x0,
            y0,
            yaw,
            size=1.0,
            resolution=0.05
        )

        self.get_logger().info(
            f"Generated {len(path.poses)} poses"
        )

        goal = FollowPath.Goal()

        goal.path = path
        goal.controller_id = ""
        goal.goal_checker_id = ""
        goal.progress_checker_id = ""

        future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)

    def make_pose(self, x, y, yaw):

        pose = PoseStamped()

        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y

        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    def interpolate(self, x1, y1, x2, y2, yaw, resolution):

        poses = []

        dist = math.hypot(x2 - x1, y2 - y1)

        steps = max(2, int(dist / resolution))

        for i in range(steps):

            t = i / float(steps)

            x = x1 + (x2 - x1) * t
            y = y1 + (y2 - y1) * t

            poses.append(self.make_pose(x, y, yaw))

        return poses

    def make_square(
            self,
            x0,
            y0,
            yaw,
            size=1.0,
            resolution=0.05):

        path = Path()

        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        p0 = (x0, y0)

        p1 = (
            x0 + size * math.cos(yaw),
            y0 + size * math.sin(yaw)
        )

        p2 = (
            p1[0] - size * math.sin(yaw),
            p1[1] + size * math.cos(yaw)
        )

        p3 = (
            x0 - size * math.sin(yaw),
            y0 + size * math.cos(yaw)
        )

        path.poses.extend(
            self.interpolate(
                p0[0], p0[1],
                p1[0], p1[1],
                yaw,
                resolution
            )
        )

        path.poses.extend(
            self.interpolate(
                p1[0], p1[1],
                p2[0], p2[1],
                yaw + math.pi / 2,
                resolution
            )
        )

        path.poses.extend(
            self.interpolate(
                p2[0], p2[1],
                p3[0], p3[1],
                yaw + math.pi,
                resolution
            )
        )

        path.poses.extend(
            self.interpolate(
                p3[0], p3[1],
                p0[0], p0[1],
                yaw - math.pi / 2,
                resolution
            )
        )

        path.poses.append(
            self.make_pose(
                x0,
                y0,
                yaw
            )
        )

        return path

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().error("Goal rejected")
            self.destroy_node()
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback):

        self.get_logger().info(
            f"Distance Remaining : {feedback.feedback.distance_to_goal:.2f}"
        )

    def result_callback(self, future):

        result = future.result()

        self.get_logger().info(
            f"Status : {result.status}"
        )

        self.get_logger().info(
            f"Error Code : {result.result.error_code}"
        )

        self.get_logger().info(
            f"Message : {result.result.error_msg}"
        )

        if result.status == GoalStatus.STATUS_SUCCEEDED:

            self.get_logger().info("Succeeded")

        else:

            self.get_logger().error("Failed")

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    node = FollowSquarePath()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
