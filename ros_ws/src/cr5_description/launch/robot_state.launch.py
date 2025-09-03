import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    package_name = 'cr5_description'
    pkg_path = get_package_share_directory(package_name)
    xacro_file = os.path.join(pkg_path, 'urdf', 'cr5.urdf.xacro')

    # Convert XACRO to URDF
    robot_description = xacro.process_file(xacro_file).toxml()  # type: ignore

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare argument
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation time'
    )

    # Robot State Publisher node
    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_sim_time)
    ld.add_action(robot_state_node)
    return ld
