#!/bin/bash
#
# RoboCupRescue RMRC 2026 - Robot Startup Script
# ===============================================
# Launches all robot services on the Jetson.
# Usage: bash start_robot.sh [--no-camera] [--no-yolo] [--no-ros]
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
LOG_DIR="$PROJECT_DIR/logs"

# Create logs directory
mkdir -p "$LOG_DIR"

# Timestamp for log files
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Default settings
ENABLE_CAMERA=true
ENABLE_YOLO=true
ENABLE_ROS=false  # Disabled by default, use --ros to enable
ENABLE_DRIVE=true
ENABLE_STATUS=true
ENABLE_LIDAR=false
ENABLE_ARM=false
USE_STANDALONE_DRIVE=true  # Use jetson/drive_bridge.py (L,R protocol) if true

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-camera)
            ENABLE_CAMERA=false
            shift
            ;;
        --no-yolo)
            ENABLE_YOLO=false
            shift
            ;;
        --ros)
            ENABLE_ROS=true
            shift
            ;;
        --no-drive)
            ENABLE_DRIVE=false
            shift
            ;;
        --no-status)
            ENABLE_STATUS=false
            shift
            ;;
        --lidar)
            ENABLE_LIDAR=true
            shift
            ;;
        --arm)
            ENABLE_ARM=true
            shift
            ;;
        --ros-drive)
            USE_STANDALONE_DRIVE=false
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  --no-camera    Disable camera streaming"
            echo "  --no-yolo      Disable YOLO detection"
            echo "  --no-status    Disable status node"
            echo "  --ros          Enable ROS2 nodes"
            echo "  --no-drive     Disable drive bridge"
            echo "  --lidar        Start lidar node (placeholder scan)"
            echo "  --arm          Start arm bridge (ZMQ 5556)"
            echo "  --ros-drive    Use ROS2 drive_bridge instead of standalone"
            echo "  -h, --help     Show this help"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

echo "=============================================="
echo "RoboCupRescue 2026 - Robot Startup"
echo "=============================================="
echo "Project directory: $PROJECT_DIR"
echo "Log directory: $LOG_DIR"
echo ""

# Source ROS2 if needed
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source workspace if exists
if [ -f "$PROJECT_DIR/ros2_ws/install/setup.bash" ]; then
    source "$PROJECT_DIR/ros2_ws/install/setup.bash"
fi

# Add shared module to Python path (repo root shared/)
export PYTHONPATH="$PROJECT_DIR/../shared:$PROJECT_DIR:$PYTHONPATH"

# Function to start a service in background
start_service() {
    local name=$1
    local command=$2
    local log_file="$LOG_DIR/${name}_${TIMESTAMP}.log"
    
    echo "[START] $name -> $log_file"
    nohup $command > "$log_file" 2>&1 &
    echo $! > "$LOG_DIR/${name}.pid"
    sleep 0.5
    
    # Check if process is running
    if kill -0 $(cat "$LOG_DIR/${name}.pid") 2>/dev/null; then
        echo "[OK] $name started (PID: $(cat "$LOG_DIR/${name}.pid"))"
    else
        echo "[FAIL] $name failed to start - check $log_file"
    fi
}

# Function to stop all services
stop_services() {
    echo ""
    echo "[SHUTDOWN] Stopping all services..."
    
    for pidfile in "$LOG_DIR"/*.pid; do
        if [ -f "$pidfile" ]; then
            pid=$(cat "$pidfile")
            name=$(basename "$pidfile" .pid)
            if kill -0 "$pid" 2>/dev/null; then
                echo "[STOP] $name (PID: $pid)"
                kill "$pid" 2>/dev/null || true
            fi
            rm -f "$pidfile"
        fi
    done
    
    echo "[DONE] All services stopped"
}

# Trap signals for clean shutdown
trap stop_services EXIT INT TERM

# Check for existing PID files and clean up
for pidfile in "$LOG_DIR"/*.pid; do
    if [ -f "$pidfile" ]; then
        pid=$(cat "$pidfile")
        if ! kill -0 "$pid" 2>/dev/null; then
            rm -f "$pidfile"
        else
            name=$(basename "$pidfile" .pid)
            echo "[WARN] $name already running (PID: $pid)"
        fi
    fi
done

echo ""
echo "Starting services..."
echo ""

# Start Drive Bridge (Pico communication)
if [ "$ENABLE_DRIVE" = true ]; then
    if [ "$USE_STANDALONE_DRIVE" = true ] && [ -f "$PROJECT_DIR/drive_bridge.py" ]; then
        start_service "drive_bridge" "python3 $PROJECT_DIR/drive_bridge.py"
    else
        DRIVE_SCRIPT="$PROJECT_DIR/ros2_ws/src/rescue_robot/rescue_robot/drive_bridge.py"
        if [ -f "$DRIVE_SCRIPT" ]; then
            start_service "drive_bridge" "python3 $DRIVE_SCRIPT"
        else
            echo "[SKIP] drive_bridge - script not found"
        fi
    fi
fi

# Start Status Node (ZMQ 5559)
if [ "$ENABLE_STATUS" = true ]; then
    STATUS_SCRIPT="$PROJECT_DIR/ros2_ws/src/rescue_robot/rescue_robot/status_node.py"
    if [ -f "$STATUS_SCRIPT" ]; then
        start_service "status_node" "python3 $STATUS_SCRIPT"
    else
        echo "[SKIP] status_node - script not found"
    fi
fi

# Start Arm Bridge (ZMQ 5556 -> serial)
if [ "$ENABLE_ARM" = true ]; then
    ARM_SCRIPT="$PROJECT_DIR/ros2_ws/src/rescue_robot/rescue_robot/arm_bridge.py"
    if [ -f "$ARM_SCRIPT" ]; then
        start_service "arm_bridge" "python3 $ARM_SCRIPT --no-serial"
    else
        echo "[SKIP] arm_bridge - script not found"
    fi
fi

# Start Lidar Node (ZMQ 5558 placeholder)
if [ "$ENABLE_LIDAR" = true ]; then
    LIDAR_SCRIPT="$PROJECT_DIR/ros2_ws/src/rescue_robot/rescue_robot/lidar_node.py"
    if [ -f "$LIDAR_SCRIPT" ]; then
        start_service "lidar_node" "python3 $LIDAR_SCRIPT --rate 10"
    else
        echo "[SKIP] lidar_node - script not found"
    fi
fi

# Wait for drive bridge to initialize
sleep 1

# Start Camera Streaming
if [ "$ENABLE_CAMERA" = true ]; then
    CAMERA_SCRIPT="$PROJECT_DIR/ros2_ws/src/rescue_robot/rescue_robot/camera_node.py"
    if [ -f "$CAMERA_SCRIPT" ]; then
        start_service "camera_node" "python3 $CAMERA_SCRIPT --scan"
    else
        echo "[SKIP] camera_node - script not found"
    fi
fi

# Start YOLO Detection
if [ "$ENABLE_YOLO" = true ] && [ "$ENABLE_CAMERA" = true ]; then
    YOLO_SCRIPT="$PROJECT_DIR/ros2_ws/src/rescue_robot/rescue_robot/yolo_node.py"
    if [ -f "$YOLO_SCRIPT" ]; then
        # Wait for camera to start
        sleep 2
        start_service "yolo_node" "python3 $YOLO_SCRIPT --interval 3"
    else
        echo "[SKIP] yolo_node - script not found"
    fi
fi

# Start ROS2 nodes if enabled
if [ "$ENABLE_ROS" = true ]; then
    echo ""
    echo "Starting ROS2 nodes..."
    
    # Check if workspace is built
    if [ ! -d "$PROJECT_DIR/ros2_ws/install" ]; then
        echo "[WARN] ROS2 workspace not built. Building now..."
        cd "$PROJECT_DIR/ros2_ws"
        colcon build --symlink-install
        source install/setup.bash
    fi
    
    # Start ROS2 launch file if exists
    LAUNCH_FILE="$PROJECT_DIR/ros2_ws/src/rescue_robot/launch/robot.launch.py"
    if [ -f "$LAUNCH_FILE" ]; then
        start_service "ros2_launch" "ros2 launch rescue_robot robot.launch.py"
    fi
fi

echo ""
echo "=============================================="
echo "All services started!"
echo "=============================================="
echo ""
echo "Services running:"
for pidfile in "$LOG_DIR"/*.pid; do
    if [ -f "$pidfile" ]; then
        name=$(basename "$pidfile" .pid)
        pid=$(cat "$pidfile")
        echo "  - $name (PID: $pid)"
    fi
done
echo ""
echo "Logs directory: $LOG_DIR"
echo ""
echo "Commands:"
echo "  - View logs: tail -f $LOG_DIR/drive_bridge_*.log"
echo "  - Stop all: kill -INT $$"
echo ""
echo "Press Ctrl+C to stop all services"
echo ""

# Keep script running and wait for interrupt
while true; do
    # Check if all processes are still running
    all_running=true
    for pidfile in "$LOG_DIR"/*.pid; do
        if [ -f "$pidfile" ]; then
            pid=$(cat "$pidfile")
            if ! kill -0 "$pid" 2>/dev/null; then
                name=$(basename "$pidfile" .pid)
                echo "[WARN] $name (PID: $pid) has stopped"
                rm -f "$pidfile"
                all_running=false
            fi
        fi
    done
    
    sleep 5
done
