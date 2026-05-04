#!/bin/bash
# RoboCupRescue RMRC 2026 - Robot Startup Script
# Starts: drive, cameras (vision), SLAM, arm, status, autonomy, RViz
# Serial ports: Pico=shared.constants PICO_SERIAL_PORT, Arm=ARM_SERIAL_PORT (adjust if USB order varies)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$LOG_DIR"

# Log rotation: cap each log at 5 MB to prevent filling the Jetson disk
MAX_LOG_BYTES=$((5 * 1024 * 1024))
for logfile in "$LOG_DIR"/*.log; do
    [ -f "$logfile" ] || continue
    logsize=$(stat -c%s "$logfile" 2>/dev/null || stat -f%z "$logfile" 2>/dev/null || echo 0)
    if [ "$logsize" -gt "$MAX_LOG_BYTES" ]; then
        mv -f "$logfile" "${logfile}.old"
        echo "[LOG] Rotated $(basename "$logfile") (${logsize} bytes)"
    fi
done

# Camera: 1=full DeepStream SDK (TensorRT in-pipeline), 0=vision_node
USE_DEEPSTREAM_SDK="${USE_DEEPSTREAM_SDK:-0}"
USE_VISION_NODE=1

# RViz: 0=off (saves Jetson CPU), 1=on
USE_RVIZ_ON_JETSON="${USE_RVIZ_ON_JETSON:-1}"

# Simulation: 1=skip Pico/drive (Gazebo only)
USE_SIM="${USE_SIM:-0}"

# Nav2 + autonomy: 1=nav2_bridge + autonomy_executor
USE_NAV2="${USE_NAV2:-0}"

echo "==========================================="
echo "  RoboCupRescue RMRC 2026 - Robot Start"
echo "==========================================="

export PYTHONPATH="${PROJECT_DIR}:${PYTHONPATH}"

# Source ROS2 (non-fatal if workspaces missing)
echo "[*] Sourcing ROS2..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/iron/setup.bash ]; then
    source /opt/ros/iron/setup.bash
else
    echo "[!] No ROS2 found; SLAM/lidar will fail"
fi
[ -f /home/sosrobo/unilidar_sdk2/install/setup.bash ] && source /home/sosrobo/unilidar_sdk2/install/setup.bash 2>/dev/null || true
[ -f /home/sosrobo/catkin_point_lio_unilidar/install/setup.bash ] && source /home/sosrobo/catkin_point_lio_unilidar/install/setup.bash 2>/dev/null || true

# Kill old processes — allow 2s for clean exit (avoids "Address already in use")
echo "[*] Cleaning up old processes..."
pkill -f drive_bridge.py 2>/dev/null || true
pkill -f camera_node.py 2>/dev/null || true
pkill -f gpu_camera_node.py 2>/dev/null || true
pkill -f deepstream_yolo_node.py 2>/dev/null || true
pkill -f deepstream_vision_node.py 2>/dev/null || true
pkill -f vision_node.py 2>/dev/null || true
pkill -f status_publisher.py 2>/dev/null || true
pkill -f followerarm.py 2>/dev/null || true
pkill -f arm_bridge.py 2>/dev/null || true
pkill -f slam_bridge.py 2>/dev/null || true
pkill -f point_cloud_downsampler.py 2>/dev/null || true
pkill -f rerun_bridge.py 2>/dev/null || true
pkill -f autonomy_node.py 2>/dev/null || true
pkill -f autonomy_executor.py 2>/dev/null || true
pkill -f auto_mission_node.py 2>/dev/null || true
pkill -f nav2_bridge.py 2>/dev/null || true
pkill -f lidar_node.py 2>/dev/null || true
pkill -f pointlio_mapping 2>/dev/null || true
pkill -f unitree_lidar 2>/dev/null || true
pkill -f stream_rviz 2>/dev/null || true
pkill -f "gst-launch.*ximagesrc" 2>/dev/null || true
pkill -f rviz2 2>/dev/null || true
sleep 2

# Gazebo sim (when USE_SIM=1): launch robot_state_publisher + optional Gazebo world
GAZEBO_LAUNCH_PID=""
if [ "$USE_SIM" = "1" ]; then
    echo ""
    echo "[*] === Gazebo Sim (USE_SIM=1) ==="
    if [ -f "$SCRIPT_DIR/gazebo/rescue_robot_gazebo.launch.py" ]; then
        ros2 launch "$SCRIPT_DIR/gazebo/rescue_robot_gazebo.launch.py" > "$LOG_DIR/gazebo_launch.log" 2>&1 &
        GAZEBO_LAUNCH_PID=$!
        sleep 2
        ps -p $GAZEBO_LAUNCH_PID > /dev/null && echo "    ✓ Gazebo launch (robot_state_publisher)" || echo "    (Gazebo launch in background)"
    else
        echo "    (rescue_robot_gazebo.launch.py not found)"
    fi
fi

# === CORE SERVICES (staggered: drive first, then status+arm, then vision to reduce USB contention) ===
echo ""
echo "[*] === CORE SERVICES ==="

DRIVE_PID=""
if [ "$USE_SIM" = "1" ]; then
    echo "    (skipping drive bridge — USE_SIM=1)"
elif [ "$USE_NAV2" = "1" ]; then
    # Nav2 mode: use ROS2 drive_bridge (subscribes to /cmd_vel) from rescue_robot package
    DRIVE_SCRIPT="$SCRIPT_DIR/ros2_ws/src/rescue_robot/rescue_robot/drive_bridge.py"
    if [ -f "$DRIVE_SCRIPT" ]; then
        python3 "$DRIVE_SCRIPT" --ros2 > "$LOG_DIR/drive_bridge.log" 2>&1 &
        DRIVE_PID=$!
        sleep 0.5
        ps -p $DRIVE_PID > /dev/null && echo "    ✓ Drive bridge (ROS2)" || echo "    ✗ Drive bridge"
    else
        python3 "$SCRIPT_DIR/drive_bridge.py" > "$LOG_DIR/drive_bridge.log" 2>&1 &
        DRIVE_PID=$!
        sleep 0.5
        ps -p $DRIVE_PID > /dev/null && echo "    ✓ Drive bridge" || echo "    ✗ Drive bridge"
    fi
else
    DRIVE_ARGS=""
    [ -n "$PICO_SERIAL_PORT" ] && DRIVE_ARGS="--serial $PICO_SERIAL_PORT"
    python3 "$SCRIPT_DIR/drive_bridge.py" $DRIVE_ARGS > "$LOG_DIR/drive_bridge.log" 2>&1 &
    DRIVE_PID=$!
    sleep 0.5
    ps -p $DRIVE_PID > /dev/null && echo "    ✓ Drive bridge" || echo "    ✗ Drive bridge"
fi

# Camera / Vision (port 5557) — 4 Jetson cameras, 2 with AI (front, arm)
# 640x480 @ 15fps for USB cameras (GStreamer MJPEG caps)
export GST_MJPEG_RESOLUTION="${GST_MJPEG_RESOLUTION:-640x480}"
# Priority: DeepStream SDK (if USE_DEEPSTREAM_SDK=1) → vision_node (default) → camera_node → deepstream_yolo
CAM_PID=""
if [ -n "$USE_DEEPSTREAM_SDK" ] && [ "$USE_DEEPSTREAM_SDK" = "1" ] && [ -f "$SCRIPT_DIR/deepstream/deepstream_vision_node.py" ]; then
    (cd "$SCRIPT_DIR" && python3 deepstream/deepstream_vision_node.py --num-cameras 4) > "$LOG_DIR/deepstream_vision_node.log" 2>&1 &
    CAM_PID=$!
    sleep 2
    if ps -p $CAM_PID > /dev/null 2>&1; then
        echo "    ✓ DeepStream vision (TensorRT in-pipeline)"
    else
        CAM_PID=""
    fi
fi
if [ -z "$CAM_PID" ] && [ -n "$USE_VISION_NODE" ] && [ "$USE_VISION_NODE" = "1" ] && [ -f "$SCRIPT_DIR/vision/vision_node.py" ]; then
    pip3 install -q 'numpy<2' 2>/dev/null || true
    sleep 1  # Stagger: let drive+arm claim serial before vision opens USB cameras
    (cd "$SCRIPT_DIR" && python3 vision/vision_node.py --fps 15 --yolo-cameras front arm) > "$LOG_DIR/vision_node.log" 2>&1 &
    CAM_PID=$!
    sleep 3
    if ps -p $CAM_PID > /dev/null 2>&1; then
        echo "    ✓ Vision node (3 cameras, 2 AI: front+arm)"
    else
        CAM_PID=""
    fi
fi
if [ -z "$CAM_PID" ]; then
    CAMERA_NODE="$SCRIPT_DIR/ros2_ws/src/rescue_robot/rescue_robot/camera_node.py"
    if [ -f "$CAMERA_NODE" ]; then
        python3 "$CAMERA_NODE" --fps 15 --scan > "$LOG_DIR/camera_node.log" 2>&1 &
        CAM_PID=$!
        sleep 2
        ps -p $CAM_PID > /dev/null && echo "    ✓ Camera node (fallback)" || CAM_PID=""
    fi
fi
if [ -z "$CAM_PID" ]; then
    if [ -f "$SCRIPT_DIR/deepstream_yolo_node.py" ]; then
        python3 "$SCRIPT_DIR/deepstream_yolo_node.py" --fps 15 > "$LOG_DIR/deepstream_yolo.log" 2>&1 &
        CAM_PID=$!
        sleep 2
        ps -p $CAM_PID > /dev/null && echo "    ✓ DeepStream YOLO" || echo "    ✗ Camera"
    else
        python3 "$SCRIPT_DIR/gpu_camera_node.py" --fps 15 > "$LOG_DIR/gpu_camera.log" 2>&1 &
        CAM_PID=$!
        sleep 2
        ps -p $CAM_PID > /dev/null && echo "    ✓ GPU Camera" || echo "    ✗ Camera"
    fi
fi

python3 "$SCRIPT_DIR/status_publisher.py" > "$LOG_DIR/status_publisher.log" 2>&1 &
STATUS_PID=$!

ARM_ARGS=""
[ -n "$ARM_SERIAL_PORT" ] && ARM_ARGS="--serial $ARM_SERIAL_PORT"
python3 "$SCRIPT_DIR/followerarm.py" $ARM_ARGS > "$LOG_DIR/followerarm.log" 2>&1 &
ARM_PID=$!
sleep 0.5
ps -p $STATUS_PID > /dev/null && echo "    ✓ Status publisher" || echo "    ✗ Status publisher"
ps -p $ARM_PID > /dev/null && echo "    ✓ Follower arm" || echo "    ✗ Follower arm"

# === SLAM STACK ===
echo ""
echo "[*] === SLAM STACK ==="

ros2 launch unitree_lidar_ros2 launch.py > "$LOG_DIR/unitree_lidar.log" 2>&1 &
LIDAR_DRIVER_PID=$!
sleep 3
echo "    ✓ Unitree LiDAR driver"

ros2 launch point_lio mapping_unilidar_l2.launch.py rviz:=false > "$LOG_DIR/pointlio.log" 2>&1 &
POINTLIO_PID=$!
sleep 2
ps -p $POINTLIO_PID > /dev/null && echo "    ✓ Point-LIO SLAM" || echo "    ✗ Point-LIO"

# When USE_NAV2=1 use nav2_bridge (TF + /map + /odom + ZMQ); else slam_bridge only
if [ "$USE_NAV2" = "1" ]; then
    python3 "$SCRIPT_DIR/nav2_bridge.py" --zmq-port 5562 > "$LOG_DIR/nav2_bridge.log" 2>&1 &
    SLAM_PID=$!
else
    python3 "$SCRIPT_DIR/slam_bridge.py" --ros2 > "$LOG_DIR/slam_bridge.log" 2>&1 &
    SLAM_PID=$!
fi

LIDAR_PID=""
LIDAR_NODE="$SCRIPT_DIR/ros2_ws/src/rescue_robot/rescue_robot/lidar_node.py"
if [ -f "$LIDAR_NODE" ]; then
    python3 "$LIDAR_NODE" --ros2 > "$LOG_DIR/lidar_node.log" 2>&1 &
    LIDAR_PID=$!
fi
sleep 0.5
if [ "$USE_NAV2" = "1" ]; then
    ps -p $SLAM_PID > /dev/null && echo "    ✓ Nav2 bridge" || echo "    ✗ Nav2 bridge"
else
    ps -p $SLAM_PID > /dev/null && echo "    ✓ SLAM bridge" || echo "    ✗ SLAM bridge"
fi

# Point cloud downsampler for Foxglove (keeps /cloud_downsampled <10k pts, 5 Hz)
DOWNSAMPLER_PID=""
if [ -f "$SCRIPT_DIR/point_cloud_downsampler.py" ]; then
    python3 "$SCRIPT_DIR/point_cloud_downsampler.py" --ros-args -p voxel_leaf_size:=0.05 -p rate_hz:=5.0 -p max_points:=10000 > "$LOG_DIR/point_cloud_downsampler.log" 2>&1 &
    DOWNSAMPLER_PID=$!
    sleep 0.5
    ps -p $DOWNSAMPLER_PID > /dev/null && echo "    ✓ Point cloud downsampler (/cloud_downsampled)" || true
fi
# Foxglove bridge: use topic /cloud_downsampled (not /cloud_registered) for 3D view to avoid ~20 MB limit.

# Rerun.io 3D view (optional; laptop connects with "Rerun 3D View" or rerun --connect tcp://<jetson_ip>:9876)
RERUN_PID=""
if [ "${USE_RERUN:-0}" = "1" ] && [ -f "$SCRIPT_DIR/rerun_bridge.py" ]; then
    python3 "$SCRIPT_DIR/rerun_bridge.py" --serve-port 9876 > "$LOG_DIR/rerun_bridge.log" 2>&1 &
    RERUN_PID=$!
    sleep 0.5
    ps -p $RERUN_PID > /dev/null && echo "    ✓ Rerun bridge (port 9876)" || true
fi

# === RViz 3D STREAM (optional - SLAM viz on laptop by default) ===
RVIZ_STREAM_PID=""
if [ "$USE_RVIZ_ON_JETSON" = "1" ]; then
    echo ""
    echo "[*] === RViz 3D Stream ==="
    "$SCRIPT_DIR/stream_rviz.sh" > "$LOG_DIR/rviz_stream.log" 2>&1 &
    RVIZ_STREAM_PID=$!
    sleep 4
    ps -p $RVIZ_STREAM_PID > /dev/null && echo "    ✓ RViz stream (TCP 5600)" || echo "    ✗ RViz stream"
else
    echo ""
    echo "[*] === RViz 3D Stream === (skipped; USE_RVIZ_ON_JETSON=1 to enable)"
fi

# === NAV2 (when USE_NAV2=1) ===
NAV2_LAUNCH_PID=""
if [ "$USE_NAV2" = "1" ]; then
    echo ""
    echo "[*] === Nav2 stack ==="
    # Robot state publisher for base_link
    ros2 launch "$SCRIPT_DIR/robot_description/launch/robot_description.launch.py" > "$LOG_DIR/robot_state_pub.log" 2>&1 &
    sleep 1
    # Nav2 bringup (controller, planner, BT navigator)
    "$SCRIPT_DIR/launch_nav2.sh" > "$LOG_DIR/nav2_bringup.log" 2>&1 &
    NAV2_LAUNCH_PID=$!
    sleep 2
    ps -p $NAV2_LAUNCH_PID > /dev/null && echo "    ✓ Nav2 bringup" || echo "    (Nav2 bringup in background)"
fi

# === AUTONOMY ===
echo ""
if [ "$USE_NAV2" = "1" ]; then
    python3 "$SCRIPT_DIR/autonomy_executor.py" > "$LOG_DIR/autonomy_executor.log" 2>&1 &
else
    python3 "$SCRIPT_DIR/autonomy_node.py" > "$LOG_DIR/autonomy_node.log" 2>&1 &
fi
AUTONOMY_PID=$!
sleep 0.5
if [ "$USE_NAV2" = "1" ]; then
    ps -p $AUTONOMY_PID > /dev/null && echo "    ✓ Autonomy executor" || echo "    ✗ Autonomy executor"
else
    ps -p $AUTONOMY_PID > /dev/null && echo "    ✓ Autonomy node" || echo "    ✗ Autonomy"
fi

echo ""
echo "==========================================="
echo "  ✓ All services started!"
echo "==========================================="
echo ""
echo "Ports:"
echo "  5555 - Drive     | 5556 - Arm"
echo "  5557 - Camera    | 5558 - Arm teleop"
echo "  5559 - Status    | 5560 - Autonomy"
echo "  5561 - YOLO/QR   | 5562 - SLAM / Nav2 bridge (20Hz)"
echo "  5563 - Lidar     | 5565 - Autonomy status | 5571 - Snapshot result (vision->laptop)"
echo "  5600 - RViz SLAM stream (TCP MJPEG, if USE_RVIZ_ON_JETSON=1)"
echo ""
echo "USE_NAV2=1: Nav2 + autonomy_executor. USE_SIM=1: skip Pico/drive + launch Gazebo (robot_state_publisher)."
echo "USE_RERUN=1: Rerun bridge (port 9876); laptop: Rerun 3D View or rerun --connect tcp://<jetson_ip>:9876"
echo "SLAM: view on laptop (2D map or 3D RViz stream). Jetson RViz: USE_RVIZ_ON_JETSON=1"
echo "RViz stream: WIDTH=640 HEIGHT=360 FPS=15 (default). RVIZ_STREAM_CPU=1 if nvjpegenc fails."
echo "Camera: 3 Jetson (front/arm/backward), 2 AI (front+arm). If only 1 cam: CAMERA_DEVICES=0,1,2"
echo "Serial override: PICO_SERIAL_PORT=/dev/ttyACM1 ARM_SERIAL_PORT=/dev/ttyACM0 (if USB order differs)"
echo "Logs: $LOG_DIR/"
echo "Press Ctrl+C to stop"

cleanup() {
    echo ""
    echo "[*] Stopping all services..."
    for pid in $DRIVE_PID $CAM_PID $STATUS_PID $ARM_PID $SLAM_PID $LIDAR_PID $DOWNSAMPLER_PID $RERUN_PID $AUTONOMY_PID $NAV2_LAUNCH_PID $POINTLIO_PID $RVIZ_STREAM_PID $LIDAR_DRIVER_PID $GAZEBO_LAUNCH_PID; do
        [ -n "$pid" ] && kill $pid 2>/dev/null || true
    done
    sleep 1
    pkill -f drive_bridge.py 2>/dev/null || true
    pkill -f camera_node.py 2>/dev/null || true
    pkill -f deepstream_vision_node.py 2>/dev/null || true
    pkill -f vision_node.py 2>/dev/null || true
    pkill -f status_publisher.py 2>/dev/null || true
    pkill -f deepstream_yolo_node 2>/dev/null || true
    pkill -f gpu_camera_node 2>/dev/null || true
    pkill -f followerarm.py 2>/dev/null || true
    pkill -f arm_bridge.py 2>/dev/null || true
    pkill -f slam_bridge.py 2>/dev/null || true
    pkill -f point_cloud_downsampler.py 2>/dev/null || true
    pkill -f nav2_bridge.py 2>/dev/null || true
    pkill -f autonomy_node.py 2>/dev/null || true
    pkill -f autonomy_executor.py 2>/dev/null || true
    pkill -f pointlio_mapping 2>/dev/null || true
    pkill -f unitree_lidar 2>/dev/null || true
    pkill -f "gst-launch" 2>/dev/null || true
    pkill -f rviz2 2>/dev/null || true
    pkill -f rerun_bridge.py 2>/dev/null || true
    echo "[*] Done"
    exit 0
}
trap cleanup SIGINT SIGTERM

tail -f "$LOG_DIR"/*.log 2>/dev/null &
wait
