#!/usr/bin/env python3
import time
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

def restart_microcontroller(
    url, retries=5, timeout=0.2, retry_delay=5.0, sleep=3.0
) -> bool:
    for _ in range(retries):
        try:
            rclpy.logging.get_logger("URL Log").info(
                f"Sending request to the microcontroller URL to restart..."
            )
            response = requests.get(url, timeout=timeout)
            response.raise_for_status()
            rclpy.logging.get_logger("URL Log").info(
                f"Request successful. Response: {response}"
            )
            time.sleep(sleep)
            return True

        except requests.exceptions.RequestException as e:
            rclpy.logging.get_logger("URL Log").error(f"An error occurred: {e}")
            time.sleep(retry_delay)

    rclpy.logging.get_logger("URL Log").error(
        f"Failed to restart microcontroller after {retries} retries."
    )
    return False


def generate_launch_description():
    # if not restart_microcontroller("http://172.16.14.12/restart.html"):
    #     rclpy.logging.get_logger("uwaba.launch").error(
    #         "Starting system in degraded mode..."
    #     )
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
            robot_state_publisher_node,
            joint_state_publisher_node,
            robot_localization_node,
            rviz_node,
        ]
    )
