from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("uwaba_prototype_bringup"),
                    "urdf",
                    "robot_description.urdf.xacro",
                ]
            ),
            " "
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # robot_controllers = PathJoinSubstitution(
    #     [
    #         FindPackageShare("uwaba_prototype_diffbot"),
    #         "config",
    #         "diffbot_controllers.yaml",
    #     ]
    # )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("uwaba_prototype_description"),
            "rviz",
            "urdf_config.rviz",
        ]
    )

    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_controllers],
    #     output="both",
    #     remappings=[
    #         ("~/robot_description", "/robot_description"),
    #     ],
    # )

    joint_state_pub_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name='joint_state_publisher',
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        # remappings=[
        #     ("/diff_drive_controller/cmd_vel", "/cmd_vel"),
        # ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "joint_state_broadcaster",
    #         "--controller-manager",
    #         "/controller_manager",
    #     ],
    # )

    # robot_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "diffbot_base_controller",
    #         "--controller-manager",
    #         "/controller_manager",
    #     ],
    # )

    # # Delay rviz start after `joint_state_broadcaster`
    # delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[rviz_node],
    #     )
    # )

    # # Delay start of robot_controller after `joint_state_broadcaster`
    # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=joint_state_broadcaster_spawner,
    #             on_exit=[robot_controller_spawner],
    #         )
    #     )
    # )
    # delay_rviz_after_joint_state_publisher = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_pub_node,
    #         on_exit=[rviz_node],
    #     )
    # )

    nodes = [
        joint_state_pub_node,
        robot_state_pub_node,
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes)