#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 — Gazebo Simulation Launch
====================================================
Launches:
  1. Gazebo with empty world (or rmrc_arena if available)
  2. robot_state_publisher (URDF → TF)
  3. slam_toolbox (online async SLAM from /scan)
  4. drive_bridge --sim (ZMQ → /cmd_vel)
  5. slam_bridge --placeholder (ZMQ 5562 for GUI)

Usage:
  ros2 launch simulation/launch/sim_launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.dirname(SCRIPT_DIR)
REPO_ROOT = os.path.dirname(SIM_DIR)
URDF_PATH = os.path.join(SIM_DIR, "robot_description", "robot.urdf")
WORLD_PATH = os.path.join(SIM_DIR, "worlds", "rmrc_arena.world")


def generate_launch_description():
    # Read URDF
    with open(URDF_PATH, "r") as f:
        robot_description = f.read()

    # Gazebo
    gazebo_args = ["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so"]
    if os.path.isfile(WORLD_PATH):
        gazebo_args.append(WORLD_PATH)
    gazebo = ExecuteProcess(cmd=gazebo_args, output="screen")

    # Robot state publisher (publishes TF from URDF)
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    # Spawn robot in Gazebo
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "rescue_robot", "-x", "0", "-y", "0", "-z", "0.1"],
        output="screen",
    )

    # SLAM Toolbox (online async — builds map from /scan)
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[{
            "use_sim_time": True,
            "odom_frame": "odom",
            "map_frame": "map",
            "base_frame": "base_link",
            "scan_topic": "/scan",
            "mode": "mapping",
            "resolution": 0.05,
            "max_laser_range": 20.0,
        }],
        output="screen",
    )

    # Drive bridge in sim mode (ZMQ 5555 → /cmd_vel)
    drive_bridge_sim = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(REPO_ROOT, "jetson", "drive_bridge.py"),
            "--sim",
        ],
        output="screen",
        additional_env={"PYTHONPATH": REPO_ROOT},
    )

    # SLAM bridge placeholder (provides ZMQ 5562 for GUI without Point-LIO)
    slam_bridge = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(REPO_ROOT, "jetson", "slam_bridge.py"),
            "--placeholder",
        ],
        output="screen",
        additional_env={"PYTHONPATH": REPO_ROOT},
    )

    return LaunchDescription([
        gazebo,
        TimerAction(period=2.0, actions=[robot_state_pub]),
        TimerAction(period=4.0, actions=[spawn]),
        TimerAction(period=6.0, actions=[slam_toolbox]),
        TimerAction(period=3.0, actions=[drive_bridge_sim]),
        TimerAction(period=3.0, actions=[slam_bridge]),
    ])
