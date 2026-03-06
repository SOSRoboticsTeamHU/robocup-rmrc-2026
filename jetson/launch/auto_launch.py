#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2025 - Auto Mission Launch
==============================================
Starts AutoMissionNode (and optionally slam_bridge, drive_bridge) for autonomy tests.
Prerequisite: Point-LIO must be running (/Odometry, /cloud_registered).
Usage:
  From repo root:  PYTHONPATH=$PWD python3 jetson/launch/auto_launch.py
  From jetson/:    PYTHONPATH=.. python3 launch/auto_launch.py
  Or after start_robot.sh (Point-LIO + drive_bridge --ros2 already up):
      cd jetson && python3 auto_mission_node.py
"""

import os
import sys
import subprocess


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    jetson_dir = os.path.dirname(script_dir)
    repo_root = os.path.dirname(jetson_dir)
    env = os.environ.copy()
    env["PYTHONPATH"] = repo_root + os.pathsep + env.get("PYTHONPATH", "")

    node_script = os.path.join(jetson_dir, "auto_mission_node.py")
    if not os.path.isfile(node_script):
        print(f"[AUTO_LAUNCH] Not found: {node_script}")
        sys.exit(1)

    print("[AUTO_LAUNCH] Starting Auto Mission Node (Point-LIO must be running)")
    print("  /Odometry, /cloud_registered -> auto_mission_node -> /cmd_vel")
    print("  ZMQ: commands 5560, status 5565")
    try:
        subprocess.run(
            [sys.executable, node_script],
            cwd=repo_root,
            env=env,
        )
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
