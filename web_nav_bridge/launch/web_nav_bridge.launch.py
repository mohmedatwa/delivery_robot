import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    cloud_url = LaunchConfiguration('cloud_url')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'cloud_url',
            default_value='http://localhost:8080',
            description='URL of the cloud server'
        ),
        Node(
            package='web_nav_bridge',
            executable='websocket_nav_node',
            name='web_nav_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'cloud_url': cloud_url
            }]
        )
    ])