from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_description = get_package_share_directory('cr5_description')
    pkg_hardware = get_package_share_directory('dobot_cr5s_hardware')

    declare_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false',
                                             description='Use simulation time')

    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'robot_state.launch.py')
        )
    )

    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_hardware, 'launch', 'ros2_control.launch.py')
        )
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'rviz.launch.py')
        )
    )

    return LaunchDescription([
        declare_sim_time,
        robot_state_launch,
        ros2_control_launch,
        rviz_launch
    ])
