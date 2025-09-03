import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description() -> LaunchDescription:
    package_name = 'cr5_description'
    pkg_path: str = os.path.join(get_package_share_directory(package_name))
    xacro_file: str = os.path.join(pkg_path, 'urdf', 'cr5.urdf.xacro')
    rviz_config_file: str = os.path.join(pkg_path, 'rviz', 'simulation.rviz')

    # Check if the xacro file exists
    if not os.path.exists(xacro_file):
        raise FileNotFoundError(f"Xacro file not found at: {xacro_file}")

    # Convert XACRO to URDF

    robot_description: str = xacro.process_file(
        xacro_file,
        mappings={'scale_for_web': 'false'}  # or 'false'
    ).toxml()  # type: ignore

    robot_description_web: str = xacro.process_file(
        xacro_file,
        mappings={'scale_for_web': 'true'}  # or 'false'
    ).toxml()  # type: ignore

    # Launch configuration variable
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Save generated URDF to shared location
    output_urdf_path: str = os.path.join(pkg_path, 'urdf', 'generated_cr5.urdf')
    with open(output_urdf_path, 'w') as f:
        f.write(robot_description_web)

    # Declare arguments
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time'
    )

    # robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()
    ld.add_action(declare_sim_time)
    ld.add_action(robot_state_publisher)
    ld.add_action(rviz_node)
    return ld
