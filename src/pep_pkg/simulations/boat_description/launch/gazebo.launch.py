import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    urdf_file = os.path.join(get_package_share_directory("boat_description"), "urdf", "boat.urdf.xacro")

    return LaunchDescription(
        [
            ExecuteProcess(cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so"], output="screen"),
            ExecuteProcess(
                cmd=["ros2", "run", "gazebo_ros", "spawn_entity.py", "-file", urdf_file, "-entity", "boat"],
                output="screen",
            ),
        ]
    )
