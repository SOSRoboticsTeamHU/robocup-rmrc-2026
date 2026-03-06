#!/bin/bash
# RoboCupRescue RMRC 2026 - Nav2 stack launch
# Run after Point-LIO + nav2_bridge are running. Use with USE_NAV2=1 from start_robot.sh.
#
# Validation: ros2 topic echo /map --once ; ros2 run tf2_tools view_frames

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
PARAMS_FILE="$SCRIPT_DIR/config/nav2_params.yaml"

export PYTHONPATH="${PROJECT_DIR}:${PYTHONPATH}"

# Source ROS2
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/iron/setup.bash

# Optional: source Nav2 workspace if built from source
# source /path/to/nav2_ws/install/setup.bash

if [ ! -f "$PARAMS_FILE" ]; then
    echo "[launch_nav2] ERROR: $PARAMS_FILE not found"
    exit 1
fi

# Launch Nav2 bringup (assumes nav2_bringup is installed: sudo apt install ros-humble-nav2-bringup)
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=false \
    params_file:="$PARAMS_FILE" \
    autostart:=true
