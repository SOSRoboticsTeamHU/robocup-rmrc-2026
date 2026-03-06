#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
URDF_FILE="${SCRIPT_DIR}/robot_description/urdf/rescue_robot.urdf"
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat $URDF_FILE)"
