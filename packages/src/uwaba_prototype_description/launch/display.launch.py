#!/usr/bin/env python3
import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare(package="uwaba_prototype_description").find(
        "uwaba_prototype_description"
    )
    default_model_path = os.path.join(pkg_share, "urdf/uwaba_prototype.urdf.xacro")
    default_rviz_config_path = os.path.join(pkg_share, "rviz/urdf_config.rviz")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": Command(["xacro ", LaunchConfiguration("model")])}
        ],
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    # robot_localization_node = Node(
    #      package='robot_localization',
    #      executable='ekf_node',
    #      name='ekf_filter_node',
    #      output='screen',
    #      parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    # )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            launch.actions.DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            joint_state_publisher_node,
            robot_state_publisher_node,
            # robot_localization_node,
            rviz_node,
        ]
    )
