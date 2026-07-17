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
                   "/controller_manager",
   
         ],
    )


    arm_controller= Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", 
                   "--controller-manager", 
                   "/controller_manager",
         ]
         ,
    )
    gripper_controller= Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", 
                   "--controller-manager", 
                   "/controller_manager",
         ]
         ,)


    return LaunchDescription(
        [         
            joint_state_broadcaster_spawner,           
            mecanum_controller,
            gripper_controller,
            arm_controller,
        ]
    )