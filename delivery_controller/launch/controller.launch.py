#/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

     

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    
   

    mecanum_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_controller", 
                   "--controller-manager", 
                   "/controller_manager"
        ],
        parameters=[os.path.join(
    get_package_share_directory("delivery_description"),
    "config",
    "mecanum_controller.yaml")]
    )

    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
             
            mecanum_controller
        ]
    )