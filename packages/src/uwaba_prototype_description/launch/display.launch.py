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
    default_model_path = os.path.join(pkg_share, "urdf/uwaba_prototype.urdf")
    default_rviz_config_path = os.path.join(pkg_share, "rviz/urdf_config.rviz")
    # world_path=os.path.join(pkg_share, 'world/my_world.sdf')

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
    uwaba_server_node = Node(
        package="uwaba_prototype_diffbot_py",
        executable="uwaba_controller_server",
        name="uwaba_prototype_controller_server",
    )
    uwaba_client_node = Node(
        package="uwaba_prototype_diffbot_py",
        executable="uwaba_controller_manager",
        name="uwaba_prototype_controller_client",
    )

    # robot_localization_node = Node(
    #      package='robot_localization',
    #      executable='ekf_node',
    #      name='ekf_filter_node',
    #      output='screen',
    #      parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    # )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=uwaba_server_node,
            on_exit=[rviz_node],
        )
    )

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
            # launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
            uwaba_server_node,
            uwaba_client_node,
            joint_state_publisher_node,
            robot_state_publisher_node,
            # robot_localization_node,
            delay_rviz_after_joint_state_broadcaster_spawner,
        ]
    )
