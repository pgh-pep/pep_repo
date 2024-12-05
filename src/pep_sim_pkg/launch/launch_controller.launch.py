import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    controller_param_file = os.path.join(
        get_package_share_directory('pep_sim_pkg'),
        'config', 'teleop.yaml'
    )

    launch_description = LaunchDescription([
        # launch.actions.DeclareLaunchArgument('cmd_vel', default_value='cmd_vel'),
        DeclareLaunchArgument('controller_param_file', default_value=controller_param_file),
    ])

    launch_description.add_action(Node(package='joy', executable='joy_node'))

    launch_description.add_action(Node(
        package='joy_teleop', executable='joy_teleop',
        parameters=[LaunchConfiguration('controller_param_file')]))


    launch_description.add_action(Node(
        package='pep_sim_pkg', executable='pep_teleop',
        ))

    return launch_description
