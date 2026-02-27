from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    delivery_nav_dir = get_package_share_directory('delivery_navigation')
    nav2_bringup_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py'
    )


    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(delivery_nav_dir, 'maps', 'small_house.yaml'),
        description='Full path to map yaml file to load',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(delivery_nav_dir, 'config', 'nav2_params.yaml'),
        description='Full path to Nav2 parameters file to load',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='false', 
        description='Use simulation (Gazebo) clock if true'
    )
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_launch_file_dir]),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    

    return LaunchDescription(
        [
            declare_map_yaml_cmd,
            declare_params_file_cmd,
            declare_use_sim_time_cmd,
            nav2_bringup_launch,
           
        ]
    )