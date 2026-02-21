#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node



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

    return LaunchDescription([
        tf_relay_node,
        odometry_relay_node,
        cmd_vel_relay_node
    ])