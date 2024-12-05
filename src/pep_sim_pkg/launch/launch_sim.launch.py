"""
Launch file to run Gazebo sim
TO RUN: ros2 launch pep_sim_pkg launch_sim.launch.py
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # replace with FindPackageShare

    pkg_share_path = get_package_share_directory('pep_sim_pkg')

    world_path = os.path.join(pkg_share_path,'worlds','empty_world.sdf')

    model_path = os.path.join(
        pkg_share_path,
        'models',
        'robot.urdf.xacro'
    )

    robot_description = xacro.process_file(model_path).toxml()


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
        launch_arguments={"verbose": "false",
                          "sdf": world_path}.items(),
    )
    # gz_launch_path = PathJoinSubstitution([get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])


    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "pepbot"],
        output="screen",
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
        }]
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'x_pos',
            default_value='0.5',
        ),
        DeclareLaunchArgument(
            'y_pos',
            default_value='-0.392',
        ),
        DeclareLaunchArgument(
            'z_pos',
            default_value='0.101',
        ),
        DeclareLaunchArgument(
            name='robot_description',
            default_value=robot_description,
        ),

        gazebo,
        robot_state_publisher,
        spawn_entity,


        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[robot_controller_spawner],
            )
        ),

    ])
