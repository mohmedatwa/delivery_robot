import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, QoSDurabilityPolicy
import tf2_ros
import requests
import threading
import rclpy.time
import math


CLOUD_URL = "http://localhost:8080"

class WebBridge(Node):
    def __init__(self):
        super().__init__('web_nav_bridge')
        
        # البيانات وحالات التأكيد
        self.map_data = None
        self.pose_data = None
        self.scan_data = None
        self.tf_map_to_odom = None
        self.tf_odom_to_base_footprint = None
        self.counter = 0

        # إعداد الـ QoS الخاص بـ Global Costmap
        # الـ Costmaps في ROS 2 غالباً ما تكون Transient Local لضمان وصولها للمشتركين الجدد
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, # Changed to KEEP_LAST
            depth=1
        )

        # المشتركين (Subscribers)
        # تم استهداف /global_costmap/costmap بدلاً من /map
        self.create_subscription(
            OccupancyGrid, 
            '/global_costmap/costmap', 
            self.map_callback, 
            map_qos
        )
        
        self.create_subscription(
            PoseWithCovarianceStamped, 
            '/amcl_pose', 
            self.pose_callback, 
            10
        )
        
        self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            10
        )

        # الناشر لأوامر الحركة
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # إعداد TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # تايمر التحديث (0.1 ثانية = 10 هرتز)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info("🚀 Web Bridge started with Global Costmap support.")

    def map_callback(self, msg):
        if self.map_data is None:
            self.get_logger().info(f"✅ Received Global Costmap ({msg.info.width}x{msg.info.height})")
        
        self.map_data = {
            "width": msg.info.width,
            "height": msg.info.height,
            "resolution": msg.info.resolution,
            "origin": {
                "x": msg.info.origin.position.x, 
                "y": msg.info.origin.position.y
            },
            "data": list(msg.data) # ملاحظة: الـ costmap يحتوي قيم من 0 لـ 100 و 254
        }

    def pose_callback(self, msg):
        if self.pose_data is None:
            self.get_logger().info("✅ Robot pose received from /amcl_pose")
        
        p = msg.pose.pose
        self.pose_data = {
            "x": p.position.x,
            "y": p.position.y,
            "yaw": self.get_yaw(p.orientation)
        }

    def scan_callback(self, msg):
        # أخذ عينة من بيانات الليزر (كل 10 قراءات) لتقليل حجم البيانات المرسلة
        self.scan_data = [r if math.isfinite(r) else 0.0 for r in msg.ranges[::10]]

    def get_yaw(self, q):
        """تحويل الـ Quaternion إلى زاوية Yaw بالراديان"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_transform_data(self, target_frame, source_frame):
        try:
            # Use rclpy.time.Time() for the current time
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            return {
                "translation": {"x": transform.transform.translation.x,
                                "y": transform.transform.translation.y,
                                "z": transform.transform.translation.z},
                "rotation": {"x": transform.transform.rotation.x,
                             "y": transform.transform.rotation.y,
                             "z": transform.transform.rotation.z,
                             "w": transform.transform.rotation.w}
            }
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Could not transform {target_frame} to {source_frame}: {ex}")
            return None

    def timer_callback(self):
        # تحديث بيانات التحويل (TF)
        self.tf_map_to_odom = self.get_transform_data('map', 'odom')
        self.tf_odom_to_base_footprint = self.get_transform_data('odom', 'base_footprint')
        threading.Thread(target=self.send_to_cloud, daemon=True).start()
    def send_to_cloud(self):
        # التحقق من وجود بيانات الموقع كحد أدنى للتشغيل
        if self.pose_data is None:
            if self.counter % 50 == 0:
                self.get_logger().warning("⚠️ Waiting for Pose Data (/amcl_pose)...")
            self.counter += 1
            return

        self.counter += 1
        
        # إرسال الخريطة كل ثانيتين (تقريباً كل 20 دورة تايمر)
        send_map = (self.counter % 20 == 0)
        current_map = self.map_data if send_map else None

        payload = {
            "pose": self.pose_data,
            "scan": self.scan_data,
            "map": current_map, # This will be None most of the time, only sent every 2 seconds
            "tf_map_to_odom": self.tf_map_to_odom,
            "tf_odom_to_base_footprint": self.tf_odom_to_base_footprint,
            
        }

        try:
            # مهلة زمنية قصيرة للطلب لتجنب تراكم الـ threads
            response = requests.post(CLOUD_URL, json=payload, timeout=0.1)
            
            if response.status_code == 200:
                if self.counter % 50 == 0:
                    self.get_logger().info("📤 Syncing with Cloud... Status: OK")
                
                # معالجة أوامر التحكم القادمة من السيرفر
                command = response.json()
                if command:
                    msg = Twist()
                    msg.linear.x = float(command.get('linear', 0.0))
                    msg.angular.z = float(command.get('angular', 0.0))
                    self.cmd_pub.publish(msg)
            else:
                self.get_logger().error(f"❌ Server Error: {response.status_code}")

        except requests.exceptions.RequestException:
            if self.counter % 50 == 0:
                self.get_logger().error("🌐 Connection Error: Is the Web Server online?")
        except Exception as e:
            self.get_logger().error(f"❗ Logic Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WebBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Node stopping...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()