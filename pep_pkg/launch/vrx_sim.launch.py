"""
Launch file to start VRX sim w/ boat

Must add to vrx_gz/worlds/nbpark.sdf to include pep_bot boat model (~line 343):

<include>
    <name>pep_bot</name>
    <pose>-175 1120 0 0 0 3.14</pose>
    <uri>pep_bot</uri>
</include>

To have realtime Gazebo camera visualizer, add to vrx_gz/worlds/nbpark.sdf (~line 30):

<plugin filename="ImageDisplay" name="Image Display 3">
    <gz-gui>
        <title>RGBD: image</title>
        <property key="state" type="string">floating</property>
        <property type="double" key="width">350</property>
        <property type="double" key="height">315</property>
        <property type="double" key="y">320</property>
    </gz-gui>
    <topic>rgbd_camera/image</topic>
    <topic_picker>false</topic_picker>
    </plugin>
    <plugin filename="ImageDisplay" name="Image Display 3">
    <gz-gui>
        <title>RGBD: depth</title>
        <property key="state" type="string">floating</property>
        <property type="double" key="width">350</property>
        <property type="double" key="height">315</property>
        <property type="double" key="x">500</property>
        <property type="double" key="y">320</property>
    </gz-gui>
    <topic>rgbd_camera/depth_image</topic>
    <topic_picker>false</topic_picker>
</plugin>

TO RUN: ros2 launch pep_pkg vrx_sim.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    share_directory = get_package_share_directory("pep_pkg")

    os.environ["GZ_SIM_RESOURCE_PATH"] = (
        f"{os.path.join(share_directory, 'models')}:{os.getenv('GZ_SIM_RESOURCE_PATH', '')}"
    )

    # print(os.getenv("GZ_SIM_RESOURCE_PATH", ""))

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="nbpark",
        description="VRX sim world",
    )

    vrx_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("vrx_gz"),
                "launch",
                "vrx_environment.launch.py",
            )
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    bridge_params = os.path.join(share_directory, "config", "vrx_bridge.yaml")

    gazebo_ros_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        output="screen",
    )

    rqt_arg = DeclareLaunchArgument("rqt", default_value="true")
    image_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/rgbd_camera/image",
    )
    rqt = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        arguments=[LaunchConfiguration("image_topic")],
        condition=IfCondition(LaunchConfiguration("rqt")),
    )

    gazebo_ros_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "rgbd_camera/image",
            "rgbd_camera/depth_image",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            world_arg,
            vrx_sim,
            gazebo_ros_bridge,
            gazebo_ros_image_bridge,
            image_arg,
            rqt_arg,
            rqt,
        ]
    )
