#!/usr/bin/env python3
"""
Single Launch File for ABB IRB 1200 Automated Welding Simulation.
Starts:
- Robot State Publisher (unified workcell URDF)
- ros2_control node with fake/mock hardware
- Joint State Broadcaster & Joint Trajectory Controller spawners
- MoveIt 2 MoveGroup (Cartesian & OMPL pipelines)
- Welding Visualizer (electric arc, sparks, weld bead, 3D HUD)
- Welding Controller (automatic 8-step welding state machine)
- RViz 2 with pre-configured welding display layout
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import xacro


def load_yaml(package_name, file_path):
    pkg_path = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_path, file_path)
    try:
        with open(abs_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception:
        return {}


def launch_setup(context, *args, **kwargs):
    pkg_name = "abb_welding_standalone"
    pkg_share = get_package_share_directory(pkg_name)

    # Launch Configurations
    pattern = LaunchConfiguration("pattern").perform(context)
    auto_start = LaunchConfiguration("auto_start").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz")
    weld_speed = float(LaunchConfiguration("welding_speed").perform(context))

    # 1. Process Master Workcell Xacro
    xacro_file = os.path.join(pkg_share, "urdf", "welding_workcell.xacro")
    doc = xacro.process_file(
        xacro_file,
        mappings={
            "use_fake_hardware": "true",
            "prefix": "",
        }
    )
    robot_description = {"robot_description": doc.toxml()}

    # 2. Process MoveIt Semantic Description (SRDF)
    srdf_file = os.path.join(pkg_share, "config", "abb_irb1200_welding.srdf.xacro")
    srdf_doc = xacro.process_file(srdf_file)
    robot_description_semantic = {"robot_description_semantic": srdf_doc.toxml()}

    # 3. Load Configurations
    kinematics_yaml = load_yaml(pkg_name, "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    ompl_planning_yaml = load_yaml(pkg_name, "config/ompl_planning.yaml")
    ompl_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_planning_yaml,
    }

    joint_limits_yaml = load_yaml(pkg_name, "config/joint_limits.yaml")
    joint_limits = {"robot_description_planning": joint_limits_yaml}

    moveit_controllers_yaml = load_yaml(pkg_name, "config/moveit_controllers.yaml")
    
    ros2_controllers_path = os.path.join(pkg_share, "config", "ros2_controllers.yaml")
    welding_params_path = os.path.join(pkg_share, "config", "welding_params.yaml")

    # Trajectory Execution parameters
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.5,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.05,
        "trajectory_execution.trajectory_duration_monitoring": True,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    # 4. Define Nodes

    # Unified Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ros2_control Controller Manager Node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, ros2_controllers_path],
    )

    # Controller Spawners
    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawner_jtc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # MoveIt 2 MoveGroup Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_config,
            trajectory_execution,
            moveit_controllers_yaml,
            planning_scene_monitor_parameters,
            joint_limits,
        ],
    )

    # RViz 2 Node
    rviz_config_file = os.path.join(pkg_share, "rviz", "welding_sim.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
        condition=IfCondition(launch_rviz),
    )

    # Welding Visualizer Node
    welding_visualizer_node = Node(
        package="abb_welding_standalone",
        executable="welding_visualizer_node",
        name="welding_visualizer",
        output="screen",
        parameters=[welding_params_path],
    )

    # Welding Controller Node
    auto_start_bool = (auto_start.lower() == "true")
    welding_controller_node = Node(
        package="abb_welding_standalone",
        executable="welding_controller_node",
        name="welding_controller",
        output="screen",
        parameters=[
            welding_params_path,
            {
                "default_pattern": pattern,
                "auto_start_cycle": auto_start_bool,
                "welding_speed_mps": weld_speed,
            }
        ],
    )

    return [
        robot_state_publisher_node,
        ros2_control_node,
        spawner_jsb,
        spawner_jtc,
        move_group_node,
        rviz_node,
        welding_visualizer_node,
        welding_controller_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "pattern",
            default_value="straight",
            description="Welding seam pattern: straight | l_shaped | multi_waypoint",
        ),
        DeclareLaunchArgument(
            "auto_start",
            default_value="true",
            description="Automatically trigger the welding cycle after initialization",
        ),
        DeclareLaunchArgument(
            "welding_speed",
            default_value="0.015",
            description="Welding travel speed in meters per second (default: 0.015 m/s = 15 mm/s)",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz 2 visualization",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

