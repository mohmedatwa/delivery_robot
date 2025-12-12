import os 

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[os.path.join(get_package_share_directory("delivery_localization"), "config", "ekf.yaml")],
        
    )

    return LaunchDescription([robot_localization_node])
