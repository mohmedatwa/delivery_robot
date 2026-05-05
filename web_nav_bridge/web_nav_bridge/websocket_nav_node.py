import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
import requests
import threading
import math

CLOUD_URL = "http://localhost:8080"

class WebBridge(Node):
    def __init__(self):
        super().__init__('web_nav_bridge')
        
        # البيانات المخزنة وحالات التأكيد
        self.map_data = None
        self.pose_data = None
        self.scan_data = None
        self.counter = 0

        # المشتركين (Subscribers)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # الناشر (Publisher) لأوامر الحركة
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # تايمر سريع (0.1 ثانية)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info("🚀 Smart Web Bridge Started")
        self.get_logger().info("📡 Waiting for Robot Data (Map, Pose, Scan)...")

    def map_callback(self, msg):
        if self.map_data is None:
            self.get_logger().info("✅ Received Map Data for the first time!")
        self.map_data = {
            "width": msg.info.width,
            "height": msg.info.height,
            "resolution": msg.info.resolution,
            "origin": {"x": msg.info.origin.position.x, "y": msg.info.origin.position.y},
            "data": list(msg.data)
        }

    def pose_callback(self, msg):
        if self.pose_data is None:
            self.get_logger().info("✅ Received Pose Data (Robot Located)!")
        p = msg.pose.pose
        self.pose_data = {
            "x": p.position.x,
            "y": p.position.y,
            "yaw": self.get_yaw(p.orientation)
        }

    def scan_callback(self, msg):
        if self.scan_data is None:
            self.get_logger().info("✅ Received Laser Scan Data!")
        self.scan_data = [r if math.isfinite(r) else 0.0 for r in msg.ranges[::10]]

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def timer_callback(self):
        threading.Thread(target=self.send_to_cloud).start()

    def send_to_cloud(self):
        # التحقق من وجود البيانات قبل الإرسال
        if self.pose_data is None:
            # log كل 5 ثواني تقريباً لتجنب الإزعاج
            if self.counter % 50 == 0:
                self.get_logger().warning("⚠️ Waiting for Pose Data... Check if /amcl_pose is active.")
            self.counter += 1
            return

        self.counter += 1
        # إرسال الخريطة فقط كل ثانيتين
        send_map = (self.counter % 20 == 0)
        current_map = self.map_data if send_map else None

        payload = {
            "pose": self.pose_data,
            "scan": self.scan_data,
            "map": current_map
        }

        try:
            response = requests.post(CLOUD_URL, json=payload, timeout=0.08)
            
            if response.status_code == 200:
                # Log للإرسال الناجح (اختياري، يمكن تعطيله إذا كان يملأ الشاشة)
                if self.counter % 50 == 0:
                    self.get_logger().info(f"📤 Sending Data... Status: OK {'(Map included)' if send_map else ''}")
                
                # استقبال الأوامر
                command = response.json()
                if command and (command.get('linear') != 0 or command.get('angular') != 0):
                    msg = Twist()
                    msg.linear.x = float(command['linear'])
                    msg.angular.z = float(command['angular'])
                    self.cmd_pub.publish(msg)
                    self.get_logger().info(f"🕹️ Web Control Received: Lin={msg.linear.x}, Ang={msg.angular.z}")
            else:
                self.get_logger().error(f"❌ Server Error: {response.status_code}")

        except requests.exceptions.ConnectionError:
            if self.counter % 50 == 0:
                self.get_logger().error("🌐 Connection Error: Is Node.js Server running?")
        except Exception as e:
            self.get_logger().error(f"❗ Unexpected Error: {e}")

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