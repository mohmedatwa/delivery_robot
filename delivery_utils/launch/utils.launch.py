#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import os 
from ament_index_python import get_package_share_directory


def generate_launch_description():
    tf_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='tf_relay',
        arguments=['/mecanum_controller/tf_odometry', '/tf'],
        output='screen'
    )

    odometry_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='odometry_relay',
        arguments=['/mecanum_controller/odometry', '/mecanum_controller/odom'],
        output='screen'
    )

    cmd_vel_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/mecanum_controller/cmd_vel', '/mecanum_controller/reference'],
        output='screen'
    )
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter_chain',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory("delivery_utils"), "config", "laser_filter.yaml")
        ],
        remappings=[
            ('scan', '/scan'),
            ('scan_filtered', '/scan_filtered')
        ]
    )

    return LaunchDescription([
        tf_relay_node,
        odometry_relay_node,
        cmd_vel_relay_node,
        laser_filter_node
    ])