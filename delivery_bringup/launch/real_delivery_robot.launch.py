import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    lidar_port_arg = DeclareLaunchArgument(
        "lidar_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for RPLIDAR A1"
    )

    hardware_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("delivery_firmware"),
                "launch",
                "hardware_interface.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "false"
        }.items()
    )

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

    joy_stick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("delivery_twist"),
                "launch",
                "joy_teleop.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "false"
        }.items()
    )

    mpu6050_driver = Node(
        package="delivery_firmware",
        executable="mpu6050_driver.py",
        name="mpu6050_driver",
        output="screen",
        parameters=[
            {"use_sim_time": False}
        ],
        respawn=True,
        respawn_delay=2.0,
    )

    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sllidar_ros2"),
                "launch",
                "sllidar_a1_launch.py"
            )
        ),
        launch_arguments={
            "serial_port": LaunchConfiguration("lidar_port"),
            "frame_id": "laser_link",
        }.items()
    )

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

    delayed_navigation = RegisterEventHandler(
        OnProcessStart(
            target_action=controller,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[navigation]
                )
            ]
        )
    )

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

    delayed_utilities = RegisterEventHandler(
        OnProcessStart(
            target_action=controller,
            on_start=[utilities]
        )
    )

    return LaunchDescription([
        lidar_port_arg,
        hardware_interface,
        controller,
        lidar_driver,
        mpu6050_driver,
        joy_stick,
        delayed_navigation,
        delayed_utilities,
    ])