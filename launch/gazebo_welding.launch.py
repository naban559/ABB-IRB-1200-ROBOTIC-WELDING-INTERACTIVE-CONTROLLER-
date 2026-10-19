#!/usr/bin/env python3
"""
Single Launch File for ABB IRB 1200 Automated Welding Simulation in Gazebo Ignition.

Launches:
- Ignition Gazebo (Fortress) with industrial welding cell world
- Robot Description with Gazebo ros2_control system plugin
- Robot State Publisher
- Spawns ABB IRB 1200 workcell into Gazebo
- ros_gz_bridge for clock synchronization
- Spawns joint_state_broadcaster and joint_trajectory_controller
- MoveIt 2 MoveGroup motion planning pipeline
- Welding Visualizer (arc, sparks, weld bead, HUD)
- Welding Controller (automatic 8-step welding state machine)
- RViz 2 synchronized with Gazebo
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import xacro


def load_yaml(package_name, file_path):
    pkg_path = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_path, file_path)
    try:
        with open(abs_path, "r") as f:
            return yaml.safe_load(f)
    except Exception:
        return {}


def launch_setup(context, *args, **kwargs):
    pkg_name = "abb_welding_standalone"
    pkg_share = get_package_share_directory(pkg_name)
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    # Configurations
    pattern = LaunchConfiguration("pattern").perform(context)
    auto_start = LaunchConfiguration("auto_start").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz")
    weld_speed = float(LaunchConfiguration("welding_speed").perform(context))
    headless = LaunchConfiguration("headless").perform(context)

    # File paths
    world_path = os.path.join(pkg_share, "worlds", "welding_workcell.sdf")
    xacro_file = os.path.join(pkg_share, "urdf", "welding_workcell.xacro")
    gazebo_controllers_path = os.path.join(pkg_share, "config", "gazebo_controllers.yaml")
    welding_params_path = os.path.join(pkg_share, "config", "welding_params.yaml")

    # 1. Process Master Workcell Xacro configured for Gazebo Ignition
    doc = xacro.process_file(
        xacro_file,
        mappings={
            "use_gazebo": "true",
            "controllers_file": gazebo_controllers_path,
            "prefix": "",
            "workpiece_x": LaunchConfiguration("workpiece_x").perform(context),
            "workpiece_y": LaunchConfiguration("workpiece_y").perform(context),
            "workpiece_yaw": LaunchConfiguration("workpiece_yaw").perform(context),
        },
    )
    robot_description_content = doc.toxml()
    robot_description = {"robot_description": robot_description_content}

    # 2. Process MoveIt Semantic Description (SRDF)
    srdf_file = os.path.join(pkg_share, "config", "abb_irb1200_welding.srdf.xacro")
    srdf_doc = xacro.process_file(srdf_file)
    robot_description_semantic = {"robot_description_semantic": srdf_doc.toxml()}

    # 3. Load MoveIt Configurations
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

    # 4. Gazebo Sim Launch
    gz_args = f"-r -v 2 {world_path}"
    if headless.lower() == "true":
        gz_args = f"-s {gz_args}"

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": gz_args,
        }.items(),
    )

    # 5. Robot State Publisher (sim time enabled)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
    )

    # 6. Spawn Robot Entity in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robot_description_content,
            "-name", "abb_irb1200_welding_workcell",
            "-allow_renaming", "true",
            "-z", "0.0",
        ],
    )

    # 7. ROS-GZ Bridge (Clock and Camera Topics)
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/overhead_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/overhead_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/overhead_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        ],
        output="screen",
    )

    # 8. Controller Spawners (wait for Gazebo controller manager)
    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    spawner_jtc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # 9. MoveIt 2 MoveGroup Node
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
            {"use_sim_time": True},
        ],
    )

    # 10. RViz 2 Node (synchronized with Gazebo)
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
            {"use_sim_time": True},
        ],
        condition=IfCondition(launch_rviz),
    )

    # 11. Welding Visualizer Node
    welding_visualizer_node = Node(
        package="abb_welding_standalone",
        executable="welding_visualizer_node",
        name="welding_visualizer",
        output="screen",
        parameters=[
            welding_params_path,
            {"use_sim_time": True},
        ],
    )

    # 12. Welding Controller Node
    auto_start_bool = (auto_start.lower() == "true")
    welding_controller_node = Node(
        package="abb_welding_standalone",
        executable="welding_controller_node",
        name="welding_controller",
        output="screen",
        parameters=[
            welding_params_path,
            {
                "use_sim_time": True,
                "default_pattern": pattern,
                "auto_start_cycle": auto_start_bool,
                "welding_speed_mps": weld_speed,
                "cycle_delay_sec": 5.0,
            },
        ],
    )

    return [
        gz_sim,
        robot_state_publisher_node,
        spawn_entity,
        bridge_node,
        spawner_jsb,
        spawner_jtc,
        move_group_node,
        rviz_node,
        welding_visualizer_node,
        welding_controller_node,
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("abb_welding_standalone")
    ws_dir = os.path.abspath(os.path.join(pkg_share, "..", "..", "..", ".."))

    # Configure Environment Variables for Ignition Gazebo Fortress
    plugin_path = (
        f"/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins:"
        + os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "")
    )
    resource_path = (
        f"{ws_dir}/src:{ws_dir}/install:{pkg_share}:{pkg_share}/..:"
        + os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    )

    env_actions = [
        SetEnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", plugin_path),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", plugin_path),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
    ]

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
            description="Welding travel speed in m/s (default: 0.015 m/s = 15 mm/s)",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz 2 visualization synchronized with Gazebo",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo in headless mode (server only)",
        ),
        DeclareLaunchArgument(
            "workpiece_x",
            default_value="0.0",
            description="Workpiece displacement along table X in meters",
        ),
        DeclareLaunchArgument(
            "workpiece_y",
            default_value="0.0",
            description="Workpiece displacement along table Y in meters",
        ),
        DeclareLaunchArgument(
            "workpiece_yaw",
            default_value="0.0",
            description="Workpiece rotation yaw in radians",
        ),
    ]

    return LaunchDescription(env_actions + declared_arguments + [OpaqueFunction(function=launch_setup)])
