#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
import os 
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution



def generate_launch_description():
    map_name = LaunchConfiguration("map_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    nav_params = LaunchConfiguration("nav_params")
    delivery_navigation_dir = get_package_share_directory("delivery_navigation")
    lifecycle_nodes = [
        "map_server","amcl",
        "controller_server", "planner_server", 
        "smoother_server", "bt_navigator", 
        "behavior_server"
        ]
    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="small_warehouse.yaml"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )
    nav_params_arg = DeclareLaunchArgument(
        "nav_params",
        default_value=os.path.join(
            delivery_navigation_dir,
            "config",
            "nav2_params.yaml"
        ),
        description="Full path to navigatian yaml file paramter to load"
    )

    map_path = PathJoinSubstitution([
        delivery_navigation_dir,
        "maps",
        map_name
        
    ])
    
    nav2_controller_server = Node(package="nav2_controller",
        executable="controller_server",
        output="screen",
        parameters=[
            nav_params,
            {"use_sim_time": use_sim_time}
        ],)
    
    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            nav_params,
            {"use_sim_time": use_sim_time}
        ],
    )

    nav2_behaviors = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[
            nav_params,
            {"use_sim_time": use_sim_time}
        ],
    )
    
    nav2_bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            nav_params,
            {"use_sim_time": use_sim_time}
        ],
    )

    nav2_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[
            nav_params,
            {"use_sim_time": use_sim_time}
        ],
    )



    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_path},
            {"use_sim_time": use_sim_time}
        ],
    )

    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        emulate_tty=True,
        parameters=[
            nav_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )


    rviz_config = os.path.join(
    get_package_share_directory("nav2_bringup"),
    "rviz",
    "nav2_default_view.rviz")

    rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    output="screen",
    arguments=["-d", rviz_config],
    parameters=[{"use_sim_time": True}]
    )     
        
    
    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,
        nav_params_arg,
        nav2_map_server,
        nav2_amcl,
        nav2_controller_server,
        nav2_planner_server,
        nav2_smoother_server,
        nav2_behaviors,
        nav2_bt_navigator,
        nav2_lifecycle_manager,
        rviz_node,

    ])

    


