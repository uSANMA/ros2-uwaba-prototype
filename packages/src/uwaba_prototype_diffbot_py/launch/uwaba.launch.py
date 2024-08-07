#!/usr/bin/env python3
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.actions import (
    RegisterEventHandler,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from os.path import join
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = FindPackageShare(package="uwaba_prototype_description").find(
        "uwaba_prototype_description"
    )
    nav2_launch_bringup = join(
        get_package_share_directory("uwaba_prototype_diffbot_py"),
        "launch",
        "bringup_launch.py",
    )

    default_model_path = join(pkg_share, "urdf/uwaba_prototype.urdf.xacro")
    default_rviz_config_path = join(pkg_share, "rviz/urdf_config_wodom.rviz")
    ekf_configs_path = PathJoinSubstitution(
        [FindPackageShare("uwaba_prototype_diffbot_py"), "config", "ekf.yaml"]
    )

    parameters = PathJoinSubstitution(
        [FindPackageShare("uwaba_prototype_diffbot_py"), "params", "params.yaml"]
    )

    agent_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        arguments=["udp4", "-p", "8888", "-v4"],
        output="screen",
    )

    client_node = Node(
        package="uwaba_prototype_diffbot_py",
        executable="uwaba_controller_manager_nav2",
        name="uwaba_controller_manager_node",
        parameters=[parameters],
    )

    server_node = Node(
        package="uwaba_prototype_diffbot_py",
        executable="uwaba_controller_server_nav2",
        name="uwaba_controller_server_node",
        parameters=[parameters],
    )

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
    #     package="robot_localization",
    #     executable="ekf_node",
    #     name="ekf_filter_node",
    #     output="screen",
    #     parameters=[ekf_configs_path],
    # )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(nav2_launch_bringup)
            # ),
            agent_node,
            server_node,
            client_node,
            # robot_localization_node,
            joint_state_publisher_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
