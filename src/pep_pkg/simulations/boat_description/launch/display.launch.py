import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Path to the Xacro file
    xacro_file = os.path.join(get_package_share_directory("boat_description"), "urdf", "boat.urdf.xacro")

    # Path to the output URDF file
    urdf_file = "boat.urdf"

    # Run xacro command in Python
    subprocess.run(["ros2", "run", "xacro", "xacro", xacro_file, "-o", urdf_file], check=True)

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": urdf_file}],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", os.path.join(get_package_share_directory("boat_description"), "rviz", "boat.rviz")],
            ),
        ]
    )
