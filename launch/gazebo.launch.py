#!/usr/bin/env python3
"""
Gazebo Ignition Simulation Launch File for ABB IRB 1200 Welding Workcell.
Launches Gazebo Sim (Ignition Fortress) with the welding world, spawns the ABB workcell,
and provides ros_gz_bridge for clock and status.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory("abb_welding_standalone")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world_path = os.path.join(pkg_share, "worlds", "welding_workcell.sdf")
    xacro_file = os.path.join(pkg_share, "urdf", "welding_workcell.xacro")

    # Process Xacro
    doc = xacro.process_file(
        xacro_file,
        mappings={
            "use_fake_hardware": "true",
            "prefix": "",
        }
    )
    robot_description = {"robot_description": doc.toxml()}

    # 1. Gazebo Sim Launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r -v 2 {world_path}",
        }.items(),
    )

    # 2. Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # 3. Spawn Robot Entity in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "abb_irb1200_welding_workcell",
            "-allow_renaming", "true",
            "-z", "0.0",
        ],
    )

    # 4. ROS-GZ Bridge (Clock)
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        output="screen",
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher_node,
        spawn_entity,
        bridge_node,
    ])

