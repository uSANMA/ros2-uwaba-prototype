#!/usr/bin/env python3
import requests
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.actions import (
    RegisterEventHandler,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from os.path import join
from os import environ
from ament_index_python.packages import get_package_share_directory
import rclpy.logging
from uwaba_prototype_diffbot_py.uwaba_controller_server_nav2 import (
    restart_microcontroller,
)


def generate_launch_description():
    if not restart_microcontroller("http://172.16.14.13/restart.html"):
        rclpy.logging.get_logger("uwaba.launch").error(
            "Starting system in degraded mode..."
        )
    # micro_ros_agent_path = join(environ['HOME'], 'Micro-XRCE-DDS-Agent/build/MicroXRCEAgent')
    pkg_share = FindPackageShare(package="uwaba_prototype_description").find(
        "uwaba_prototype_description"
    )

    default_model_path = join(pkg_share, "urdf/uwaba_prototype.urdf.xacro")
    default_rviz_config_path = join(pkg_share, "rviz/urdf_config_with_laser.rviz")
    ekf_configs_path = PathJoinSubstitution(
        [FindPackageShare("uwaba_prototype_diffbot_py"), "config", "ekf.yaml"]
    )

    parameters = PathJoinSubstitution(
        [FindPackageShare("uwaba_prototype_diffbot_py"), "params", "params.yaml"]
    )

    # agent_node = Node(
    #     package="micro_ros_agent",
    #     executable="micro_ros_agent",
    #     name="micro_ros_agent",
    #     arguments=["udp4", "-p", "8888", "-v4"],
    #     output="screen",
    # )

    agent_node = ExecuteProcess(cmd=["MicroXRCEAgent", "udp4", "-p", "8888", "-v4"], output="screen")

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
            {"robot_description": Command(["xacro ", LaunchConfiguration("model")])},
            {"publish_frequency": 30.0},
        ],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"rate": 30}, {"source_list": ["new_joint_states"]}],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_configs_path],
    )

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
            agent_node,
            server_node,
            client_node,
            #joint_state_publisher_node,
            robot_state_publisher_node,
            robot_localization_node,
            rviz_node,
        ]
    )
