import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("delivery_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )

    # lidar_driver = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("rplidar_ros"),
    #         "launch",
    #         "rplidar.launch.py"
    #     ),
    #     launch_arguments={
    #         "use_sim_time": "false"
    #     }.items()
    # )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("delivery_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'laser_link',
            "use_sim_time": "false"
        }.items()
    )

    joy_stick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("delivery_twist"),
            "launch",
            "joy_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "false"
        }.items()
    )

    mpu6050_driver = Node(
        package="delivery_firmware",
        executable="mpu6050_driver.py",
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("delivery_navigation"),
            "launch",
            "delivery_nav.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "false"
        }.items()
     )
    utilities = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("delivery_utils"),
            "launch",
            "utils.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "false"
        }.items()   

    )

    return LaunchDescription([
        hardware_interface,
        # lidar_driver,
        controller,
        joy_stick,
        mpu6050_driver,
        navigation,
        utilities
    ])

