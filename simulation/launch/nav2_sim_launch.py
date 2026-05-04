#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 — Nav2 Simulation Launch
=================================================
Loads nav2_params.yaml with use_sim_time overridden to true.
Run AFTER sim_launch.py (Gazebo + SLAM must be up first).

Usage:
  ros2 launch simulation/launch/nav2_sim_launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.dirname(SCRIPT_DIR)
REPO_ROOT = os.path.dirname(SIM_DIR)
NAV2_PARAMS = os.path.join(REPO_ROOT, "jetson", "config", "nav2_params.yaml")


def generate_launch_description():
    # Nav2 bringup launch (from nav2_bringup package)
    nav2_bringup_dir = ""
    try:
        nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    except Exception:
        print("[NAV2_SIM] nav2_bringup package not found. Install: sudo apt install ros-humble-nav2-bringup")
        return LaunchDescription([])

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": NAV2_PARAMS,
            "autostart": "true",
        }.items(),
    )

    # Autonomy executor (connects to Nav2 + SLAM ZMQ)
    autonomy_executor = Node(
        package=None,
        executable="python3",
        arguments=[os.path.join(REPO_ROOT, "jetson", "autonomy_executor.py")],
        output="screen",
        additional_env={"PYTHONPATH": REPO_ROOT},
    )

    return LaunchDescription([
        SetEnvironmentVariable("PYTHONPATH", REPO_ROOT),
        nav2_launch,
        autonomy_executor,
    ])
