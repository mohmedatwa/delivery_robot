#/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition , UnlessCondition

def generate_launch_description():

   
    use_mecanum_controller_arg = DeclareLaunchArgument(
        "use_mecanum_controller",
        default_value="true",
    )

    
    use_mecanum_controller = LaunchConfiguration("use_mecanum_controller")


    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[os.path.join(get_package_share_directory("delivery_controller"),"config","delivery_controllers.yaml")]
    #     )

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
        
        condition=IfCondition(use_mecanum_controller),
    )

    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", 
                   "--controller-manager", 
                   "/controller_manager",

            ],
        
        condition=UnlessCondition(use_mecanum_controller),

    )

    return LaunchDescription(
        [         
            
            
            use_mecanum_controller_arg,
            # control_node,
            joint_state_broadcaster_spawner,
            diff_drive_controller,           
            mecanum_controller,
            
        ]
    )