#!/bin/bash
# RoboCupRescue RMRC 2026 - SLAM Stack

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================="
echo "  RoboCupRescue RMRC 2026 - SLAM Stack"
echo "=========================================="

# Parse args
RVIZ_ARG="true"
USE_FOLLOW_RVIZ="false"
for arg in "$@"; do
    case $arg in
        --no-rviz) RVIZ_ARG="false" ;;
        --follow) USE_FOLLOW_RVIZ="true"; RVIZ_ARG="false" ;;
    esac
done

# Source ROS2
source /opt/ros/humble/setup.bash
source /home/sosrobo/unilidar_sdk2/install/setup.bash 2>/dev/null
source /home/sosrobo/catkin_point_lio_unilidar/install/setup.bash 2>/dev/null

# Cleanup
echo "[*] Cleanup..."
pkill -f pointlio_mapping 2>/dev/null
pkill -f unitree_lidar 2>/dev/null
pkill -f robot_state_publisher 2>/dev/null
pkill -f rviz2 2>/dev/null
sleep 1

# Start Lidar
echo "[*] Starting Unitree L2 Lidar..."
ros2 launch unitree_lidar_ros2 launch.py &
LIDAR_PID=$!
sleep 3

# Start robot model (Python launch)
echo "[*] Starting robot model..."
ros2 launch "$SCRIPT_DIR/robot_description/launch/robot.launch.py" &
RSP_PID=$!
sleep 1

# Start Point-LIO
echo "[*] Starting Point-LIO SLAM..."
ros2 launch point_lio mapping_unilidar_l2.launch.py rviz:=$RVIZ_ARG &
POINTLIO_PID=$!
sleep 3

# Custom RViz
if [ "$USE_FOLLOW_RVIZ" = "true" ]; then
    echo "[*] Starting RViz (auto-follow)..."
    rviz2 -d "$SCRIPT_DIR/rviz/point_lio_follow.rviz" &
    RVIZ_PID=$!
fi

echo ""
echo "=========================================="
echo "  SLAM Running!"
echo "=========================================="
echo "Press Ctrl+C to stop"

cleanup() {
    echo "[*] Stopping..."
    pkill -f pointlio_mapping 2>/dev/null
    pkill -f unitree_lidar 2>/dev/null
    pkill -f robot_state_publisher 2>/dev/null
    pkill -f rviz2 2>/dev/null
    exit 0
}

trap cleanup SIGINT SIGTERM
wait
