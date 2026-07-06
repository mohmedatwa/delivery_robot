import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # RViz config file
    rviz_config = os.path.join(
    get_package_share_directory("nav2_bringup"),
    "rviz","nav2_default_view.rviz")

    # Controller
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("delivery_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "false"
        }.items()
    )

    # Utils
    utilities = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("delivery_utils"),
                "launch",
                "utils.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "false"
        }.items()
    )

    # Navigation
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("delivery_navigation"),
                "launch",
                "delivery_nav.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "false"
        }.items()
    )

    # RViz
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     arguments=["-d", rviz_config],
    #     parameters=[
    #         {"use_sim_time": use_sim_time}
    #     ]
    # )

    return LaunchDescription([
        # Start controller immediately
        controller,

        # Start utils after 3 seconds
        TimerAction(
            period=3.0,
            actions=[utilities]
        ),

        # Start navigation after 10 seconds
        TimerAction(
            period=10.0,
            actions=[navigation]
        ),

        # Start RViz after navigation is up
        # TimerAction(
        #     period=15.0,
        #     actions=[rviz_node]
        # ),
    ])