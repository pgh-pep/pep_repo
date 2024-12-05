"""
Launch file to visualze URDF in Rviz2
TO RUN: ros2 launch pep_sim_pkg rsp.launch.py
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share_path = get_package_share_directory('pep_sim_pkg')

    model_path = os.path.join(
        pkg_share_path,
        'models',
        'robot.urdf.xacro'
    )

    robot_description = xacro.process_file(model_path).toxml()

    # rviz_config_file = os.path.join(
    #     pkg_share_path,
    #     'rviz', 'urdf.rviz'
    # )

    return LaunchDescription([

        DeclareLaunchArgument(
            name='robot_description',
            default_value=robot_description,
        ),

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='False',
        ),

        DeclareLaunchArgument(
            name='use_gui',
            default_value='true'
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{
                'use_gui': LaunchConfiguration('use_gui'),
            }]
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': LaunchConfiguration('robot_description'),
            }]
        ),

        # Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # arguments=['-d', rviz_config_file],
        ),

    ])
