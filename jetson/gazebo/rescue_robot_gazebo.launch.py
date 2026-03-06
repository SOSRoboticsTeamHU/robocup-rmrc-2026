#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Gazebo Sim Launch
============================================
Launches URDF (robot_state_publisher) and optional Gazebo world for simulation.
When USE_SIM=1, start_robot.sh skips Pico and real drive_bridge; run this launch
then start nav2_bridge + autonomy_executor separately (or via start_robot.sh with USE_SIM=1).

Worlds: flat (default), incline, sand — set GAZEBO_WORLD=incline or sand if you have them.
Point-LIO in sim: run unitree_lidar + point_lio with simulated lidar, or use a bag.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Resolve paths relative to this file
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
JETSON_DIR = os.path.dirname(SCRIPT_DIR)
URDF_PATH = os.path.join(JETSON_DIR, "robot_description", "urdf", "rescue_robot.urdf")


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="flat",
        description="World name: flat, incline, or sand (requires world files in gazebo/worlds/)",
    )

    with open(URDF_PATH, "r") as f:
        robot_description = f.read()

    # Robot state publisher — same URDF as real robot (aft_mapped base)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "publish_frequency": 30.0}],
    )

    # Optional: Gazebo with empty or custom world (if gazebo_ros is installed)
    # Validation: ros2 launch jetson/gazebo rescue_robot_gazebo.launch.py
    use_gazebo = os.environ.get("USE_GAZEBO", "0") == "1"
    actions = [world_arg, robot_state_publisher]

    if use_gazebo:
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_gazebo = get_package_share_directory("gazebo_ros")
            gazebo_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo, "launch", "gazebo.launch.py")
                ),
                launch_arguments={
                    "world": os.path.join(pkg_gazebo, "worlds", "empty.world"),
                }.items(),
            )
            actions.append(gazebo_launch)
        except Exception:
            pass  # Gazebo not installed; robot_state_publisher only

    return LaunchDescription(actions)
