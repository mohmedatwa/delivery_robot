#!/usr/bin/env python3 

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    rviz_config = os.path.join(
    get_package_share_directory("nav2_bringup"),
    "rviz","nav2_default_view.rviz")
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true', 
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory("delivery_description"),
            "launch",
            "gazebo.launch.py"
        )
    ),
    launch_arguments={
        "use_sim_time": use_sim_time
    }.items()
)
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("delivery_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items()
    )


    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("delivery_twist"),
            "launch",
            "joy_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items()
    )

#     localization = IncludeLaunchDescription(
#       os.path.join(
#            get_package_share_directory("delivery_localization"),
#            "launch",
#            "ekf.launch.py"
#        )
#    )
    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("delivery_navigation"),
            "launch",
            "delivery_nav.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items()
    )    

    utils = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("delivery_utils"),
            "launch",
            "utils.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items()
    )
    navigation_delayed = TimerAction(
    period=5.0,  # delay in seconds
    actions=[navigation]
)
    rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    output="screen",
    arguments=["-d", rviz_config],
    parameters=[{"use_sim_time": use_sim_time}]
    )   

    return LaunchDescription([
        declare_use_sim_time_cmd,
        gazebo,
        controller,
        joystick,
        navigation_delayed,
        utils,
        rviz_node,
        # localization,
    ])
