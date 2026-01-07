import os 

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # static_transform_publisher = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=["--x", "0", "--y", "0","--z", "0.103",
    #                "--qx", "1", "--qy", "0", "--qz", "0", "--qw", "0",
    #                "--frame-id", "base_footprint",
    #                "--child-frame-id", "imu_link"],
    # )


    # odom_noise = Node(
    #     package="delivery_localization",
    #     executable="odom_noisy.py",
    #     name="odom_noisy"
    #     )
    # imu_republisher = Node(
    #     package="delivery_localization",
    #     executable="imu_republisher.py",
    #     name="imu_republisher"
    #     )
    
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[os.path.join(get_package_share_directory("delivery_localization"), "config", "ekf.yaml"),
        ]
        
    )

    return LaunchDescription([
        # static_transform_publisher,
        #  imu_republisher,
        robot_localization_node,
        # odom_noise
            
        ])
