#!/usr/bin/env python3 

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("delivery_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("delivery_controller"),
            "launch",
            "controller.launch.py"
        )
    )

    rviz_config = os.path.join(
    get_package_share_directory("delivery_description"),
    "rviz",
    "display.rviz")

    rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    output="screen",
    arguments=["-d", rviz_config],
    parameters=[{"use_sim_time": True}]
    )     
        
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("delivery_twist"),
            "launch",
            "joy_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

#     localization = IncludeLaunchDescription(
#       os.path.join(
#            get_package_share_directory("delivery_localization"),
#            "launch",
#            "ekf.launch.py"
#        )
#    )
    
    return LaunchDescription([
        gazebo,
        controller,
        rviz_node,
        joystick,
        # localization,
    ])
