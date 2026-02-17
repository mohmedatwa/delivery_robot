import os 

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

# Throttle IMU to 50Hz 
    imu_throttle = Node(
        package='topic_tools',
        executable='throttle',
        arguments=['messages', '/imu/data', '30', '/imu/data_throttled']
    )

# Throttle odometry to 30Hz 
    odom_throttle = Node(
        package='topic_tools',
        executable='throttle',
        arguments=['messages', '/mecanum_controller/odom', '20', '/mecanum_controller/odom_throttled']
    )
    
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[os.path.join(get_package_share_directory("delivery_localization"), "config", "ekf.yaml"),
        ]
        
    )

    return LaunchDescription([
        imu_throttle,
        robot_localization_node,
        odom_throttle,
            
        ])
