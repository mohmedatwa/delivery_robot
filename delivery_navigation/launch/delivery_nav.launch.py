from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    delivery_nav_dir = get_package_share_directory('delivery_navigation')

    nav2_bringup_launch_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )

    map_name = LaunchConfiguration('map')

    map_yaml_file = PathJoinSubstitution([
        delivery_nav_dir,
        'maps',
        PythonExpression(["'", map_name, ".yaml'"])
    ])

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map_name_cmd = DeclareLaunchArgument(
        'map',
        default_value='small_house',
        description='Map name (without .yaml)',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(delivery_nav_dir, 'config', 'nav2_params.yaml'),
        description='Full path to Nav2 parameters file to load',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',  
        description='Use simulation (Gazebo) clock if true'
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch_file),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true', 
        }.items(),
    )

    return LaunchDescription([
        declare_map_name_cmd,
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        nav2_bringup_launch,
    ])