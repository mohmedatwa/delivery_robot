#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory("delivery_slam")  
    config_dir = os.path.join(pkg_dir, "config")

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock (true for Gazebo, false for real robot)",
    )

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time}
        ],
        arguments=[
            "-configuration_directory",
            config_dir,
            "-configuration_basename",
            "robot_2d.lua",
        ],
        remappings=[
            ("scan", "/scan"),
            ("odom", "/mecanum_controller/odom"),
        ],
    )

    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time}
        ],
        arguments=[
            "-resolution",
            "0.05",
            "-publish_period_sec",
            "1.0",
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        cartographer_node,
        occupancy_grid_node,
    ])