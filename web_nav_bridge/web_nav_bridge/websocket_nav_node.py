#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
import requests


class WebBridge(Node):
    def __init__(self):
        super().__init__('web_nav_bridge')

        self.declare_parameter('cloud_url', 'http://localhost:8080')
        self.cloud_url = self.get_parameter('cloud_url').get_parameter_value().string_value

        self._lock = threading.Lock()
        self._sending = False

        self.map_data = None
        self.pose_data = None
        self.scan_data = None
        self.counter = 0

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.create_subscription(Odometry, '/mecanum_controller/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/web_vel', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f"Web Bridge started — server: {self.cloud_url}")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def map_callback(self, msg):
        with self._lock:
            if self.map_data is None:
                self.get_logger().info("Map data received.")
            self.map_data = {
                "width": msg.info.width,
                "height": msg.info.height,
                "resolution": msg.info.resolution,
                "origin": {
                    "x": msg.info.origin.position.x,
                    "y": msg.info.origin.position.y,
                    "yaw": self.get_yaw(msg.info.origin.orientation),
                },
                "data": list(msg.data),
            }

    def pose_callback(self, msg):
        with self._lock:
            if self.pose_data is None:
                self.get_logger().info("AMCL pose received — robot localized.")
            p = msg.pose.pose
            self.pose_data = {
                "x": p.position.x,
                "y": p.position.y,
                "yaw": self.get_yaw(p.orientation),
                "source": "amcl",
            }

    def odom_callback(self, msg):
        with self._lock:
            if self.pose_data is None or self.pose_data.get("source") == "odom":
                p = msg.pose.pose
                self.pose_data = {
                    "x": p.position.x,
                    "y": p.position.y,
                    "yaw": self.get_yaw(p.orientation),
                    "source": "odom",
                }

    def scan_callback(self, msg):
        with self._lock:
            if self.scan_data is None:
                self.get_logger().info("Laser scan received.")
            self.scan_data = [
                r if math.isfinite(r) else 0.0 for r in msg.ranges[::10]
            ]

    # ------------------------------------------------------------------
    # Timer / Send
    # ------------------------------------------------------------------

    def timer_callback(self):
        if self._sending:
            return
        self._sending = True
        threading.Thread(target=self.send_to_cloud, daemon=True).start()

    def send_to_cloud(self):
        try:
            with self._lock:
                pose = self.pose_data
                scan = self.scan_data
                send_map = (self.counter % 20 == 0) and (self.map_data is not None)
                map_ = self.map_data if send_map else None

            if pose is None:
                if self.counter % 50 == 0:
                    self.get_logger().warning(
                        "No pose yet — waiting for /amcl_pose or /odom."
                    )
                self.counter += 1
                return

            self.counter += 1

            payload = {
                "pose": pose,
                "scan": scan,
                "map": map_,
            }

            response = requests.post(self.cloud_url, json=payload, timeout=0.5)

            if response.status_code == 200:
                if self.counter % 50 == 0:
                    self.get_logger().info(
                        f"Data sent OK {'(map included)' if map_ else ''} "
                        f"[source: {pose['source']}]"
                    )

                command = response.json()
                if command and (command.get('linear') != 0 or command.get('angular') != 0):
                    msg = Twist()
                    msg.linear.x = float(command['linear'])
                    msg.angular.z = float(command['angular'])
                    self.cmd_pub.publish(msg)
                    self.get_logger().info(
                        f"Web command received — linear: {msg.linear.x}, angular: {msg.angular.z}"
                    )
            else:
                self.get_logger().error(f"Server error: {response.status_code}")

        except requests.exceptions.ConnectionError:
            if self.counter % 50 == 0:
                self.get_logger().error(
                    f"Connection failed — is the server running at {self.cloud_url}?"
                )
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
        finally:
            self._sending = False


    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def get_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main():
    rclpy.init()
    node = WebBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()