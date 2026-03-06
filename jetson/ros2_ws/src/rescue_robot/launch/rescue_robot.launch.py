# RoboCupRescue RMRC 2026 - ROS2 launch
# Starts all robot nodes: drive_bridge, status_node, camera_node, yolo_node;
# optional lidar_node and arm_bridge via launch args.
# Usage: ros2 launch rescue_robot rescue_robot.launch.py
#   [lidar:=true] [arm:=true]
# Or run from repo root: python3 -m launch ... (see docs)

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess


def get_script_dir():
    """Path to rescue_robot/rescue_robot/ (Python scripts)."""
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(launch_dir)
    return os.path.join(pkg_dir, "rescue_robot")


def get_repo_root():
    """Repo root for PYTHONPATH (shared/)."""
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(launch_dir, "..", "..", "..", ".."))


def generate_launch_description():
    script_dir = get_script_dir()
    repo_root = get_repo_root()
    env = os.environ.copy()
    env["PYTHONPATH"] = repo_root + os.pathsep + env.get("PYTHONPATH", "")

    return LaunchDescription([
        DeclareLaunchArgument("jetson_ip", default_value="192.168.2.100"),
        DeclareLaunchArgument("use_sim", default_value="false"),
        DeclareLaunchArgument("lidar", default_value="false", description="Start lidar node (ZMQ 5558)"),
        DeclareLaunchArgument("arm", default_value="false", description="Start arm bridge (ZMQ 5556)"),
        ExecuteProcess(
            name="drive_bridge",
            cmd=["python3", os.path.join(script_dir, "drive_bridge.py")],
            cwd=repo_root,
            env=env,
            output="screen",
        ),
        ExecuteProcess(
            name="status_node",
            cmd=["python3", os.path.join(script_dir, "status_node.py")],
            cwd=repo_root,
            env=env,
            output="screen",
        ),
        ExecuteProcess(
            name="camera_node",
            cmd=["python3", os.path.join(script_dir, "camera_node.py")],
            cwd=repo_root,
            env=env,
            output="screen",
        ),
        ExecuteProcess(
            name="yolo_node",
            cmd=["python3", os.path.join(script_dir, "yolo_node.py")],
            cwd=repo_root,
            env=env,
            output="screen",
        ),
    ])
