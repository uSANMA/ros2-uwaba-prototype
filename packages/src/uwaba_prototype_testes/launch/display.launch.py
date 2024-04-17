#!/usr/bin/env python3
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share_urdf = launch_ros.substitutions.FindPackageShare(package='uwaba_prototype_testes').find('uwaba_prototype_testes')
    pkg_share_rvizcfg = launch_ros.substitutions.FindPackageShare(package='uwaba_prototype_testes').find('uwaba_prototype_testes')
    default_model_path = os.path.join(pkg_share_urdf, 'urdf/r2d2.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share_rvizcfg, 'urdf/r2d2.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='uwaba_prototype_testes',
        executable='state_publisher',
        name='state_publisher',
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])