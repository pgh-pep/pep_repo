# filepath: /home/ac/pep_ASNE/pep_ws/src/pep_repo/pep_pkg/launch/data_launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pep_pkg',
            executable='data_pub.py',
            name='data_publisher'
        ),
        Node(
            package='pep_pkg',
            executable='data_sub.py',
            name='data_subscriber'
        )
    ])