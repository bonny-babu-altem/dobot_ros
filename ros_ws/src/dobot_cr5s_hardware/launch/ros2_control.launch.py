import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_hardware = get_package_share_directory("dobot_cr5s_hardware")
    ros2_control_yaml = os.path.join(
        pkg_hardware, "config", "cr5s_ros2_control.yaml")

    # ros2_control node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_control_yaml],
        output="screen",
    )

    # Delay spawners slightly to ensure robot_description and controller manager are ready
    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen"
        )]
    )

    joint_trajectory_controller_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller"],
            output="screen"
        )]
    )

    ld = LaunchDescription()
    ld.add_action(ros2_control_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(joint_trajectory_controller_spawner)
    return ld
