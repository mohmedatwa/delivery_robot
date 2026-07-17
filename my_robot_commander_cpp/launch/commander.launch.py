from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("delivery_robot", package_name="arm_moveit")
        .robot_description(file_path="config/delivery.urdf.xacro")
        .robot_description_semantic(file_path="config/delivery.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    commander_node = Node(
        package="my_robot_commander_cpp",
        executable="commander",
        output="screen",
        parameters=[
            moveit_config.to_dict()
        ]
    )

    return LaunchDescription([
        commander_node
    ])