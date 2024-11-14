import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share_path = get_package_share_directory('pep_pkg')

    model_path = os.path.join(
        pkg_share_path,
        'models',
        'boat.urdf.xacro'
    )

    robot_description = xacro.process_file(model_path).toxml()


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

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{
                'use_gui': LaunchConfiguration('use_gui'),
            }]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': LaunchConfiguration('robot_description'),
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # arguments=['-d', rviz_config_file],
        ),
    ])
