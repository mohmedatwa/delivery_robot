#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    is_sim = LaunchConfiguration("is_sim")

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="true",
        description="Use simulation time",
    )

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="delivery",
            package_name="arm_moveit",
        )
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("delivery_description"),
                "urdf",
                "delivery_mobile_base.xacro",
            )
        )
        .robot_description_semantic(
            file_path="config/delivery.srdf"
        )
        .robot_description_kinematics(
            file_path="config/kinematics.yaml"
        )
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml"
        )
        .joint_limits(
            file_path="config/joint_limits.yaml"
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": is_sim,
                "publish_robot_description": True,
                "publish_robot_description_semantic": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            },
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_config = os.path.join(
        get_package_share_directory("arm_moveit"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": is_sim},
        ],
    )

    return LaunchDescription(
        [
            is_sim_arg,
            move_group_node,
            rviz_node,
        ]
    )