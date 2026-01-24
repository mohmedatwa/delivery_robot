#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
import os 
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )
    delivery_nav_dir = get_package_share_directory("delivery_navigation")
    # lifecycle_nodes = ["controller_server", "planner_server", "smoother_server", "bt_navigator", "behavior_server"]
    delivery_nav = IncludeLaunchDescription(
        launch_description_source= os.path.join(
            get_package_share_directory("nav2_bringup"),"launch","bringup_launch.py"
            ),
        launch_arguments={
            "params_file":os.path.join(delivery_nav_dir,"config","nav2_params.yaml"),
            "slam_params_file":os.path.join(delivery_nav_dir,"config","slam_params.yaml"),
            "map":os.path.join(delivery_nav_dir,"maps","iscas_map.yaml"),

        }.items())
    
    # nav2_lifecycle_manager = Node(
    #     package="nav2_lifecycle_manager",
    #     executable="lifecycle_manager",
    #     name="lifecycle_manager_navigation",
    #     output="screen",
    #     parameters=[
    #         {"node_names": lifecycle_nodes},
    #         {"use_sim_time": use_sim_time},
    #         {"autostart": True}
    #     ],
    # )
    
    return LaunchDescription([
        delivery_nav,
    ])

    


