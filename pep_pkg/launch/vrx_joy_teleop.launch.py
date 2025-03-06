import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch.actions
import launch_ros.actions


def generate_launch_description():
    parameters_file = os.path.join(
        get_package_share_directory("pep_pkg"), "config", "vrx_teleop.yaml"
    )

    ld = LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument("cmd_vel", default_value="cmd_vel"),
            launch.actions.DeclareLaunchArgument(
                "teleop_config", default_value=parameters_file
            ),
        ]
    )

    ld.add_action(launch_ros.actions.Node(package="joy", executable="joy_node"))

    ld.add_action(
        launch_ros.actions.Node(
            package="joy_teleop",
            executable="joy_teleop",
            parameters=[launch.substitutions.LaunchConfiguration("teleop_config")],
        )
    )

    return ld
