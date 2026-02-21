#!/usr/bin/env python3 

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true', 
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("delivery_description"),
            "launch",
            "gazebo.launch.py"
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
            "delivary_nav.launch.py"
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

    return LaunchDescription([
        declare_use_sim_time_cmd,
        gazebo,
        controller,
        joystick,
        navigation,
        utils,
        # localization,
    ])
