from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the absolute path to the parameter file
    config_file = os.path.join(
        os.path.dirname(__file__),  # Directory of this launch file
        '../config/path_planning_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='pep_pkg',
            executable='path_planning_node',
            name='path_planning_node',
            parameters=[config_file]  # Use the absolute path to the YAML file
        )
    ])
