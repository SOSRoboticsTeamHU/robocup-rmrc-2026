#!/bin/bash
# RViz 3D SLAM Stream - starts RViz2, maximizes window, then GStreamer MJPEG (5600)
# Laptop GUI: "RViz SLAM" tab -> "Start RViz Stream" (connects to port 5600)
# Uses nvjpegenc (GPU) + 640x360 + use-damage for higher FPS. Override: WIDTH HEIGHT FPS QUALITY
# RVIZ_STREAM_CPU=1 = fallback to CPU jpegenc. For fullscreen: wmctrl or F11 in RViz.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RVIZ_CONFIG="$SCRIPT_DIR/rviz/slam_3rd_person.rviz"
URDF_FILE="$SCRIPT_DIR/robot_description/urdf/rescue_robot.urdf"

ZMQ_PORT=${ZMQ_PORT:-5572}
STREAM_PORT=${STREAM_PORT:-5600}
# 640x360 @ 30% quality, 15 fps — lower res + GPU encode for higher FPS (was ~3 fps at 960x540 CPU)
# Override: WIDTH=640 HEIGHT=360 FPS=15 QUALITY=30 (or FPS=20 for smoother stream)
WIDTH=${WIDTH:-640}
HEIGHT=${HEIGHT:-360}
FPS=${FPS:-20}
QUALITY=${QUALITY:-25}

# Source ROS2
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/iron/setup.bash

[ -z "$DISPLAY" ] && export DISPLAY=:0
echo "Display: $DISPLAY"

cleanup() {
    echo "Stopping..."
    [ -n "$STREAMER_PID" ] && kill $STREAMER_PID 2>/dev/null
    jobs -p | xargs -r kill 2>/dev/null
    [ -n "$RVIZ_PID" ] && kill $RVIZ_PID 2>/dev/null
    [ -n "$RSP_PID" ] && kill $RSP_PID 2>/dev/null
    [ -f "$TEMP_YAML" ] && rm -f "$TEMP_YAML"
    exit 0
}
trap cleanup EXIT INT TERM

# Create temporary YAML file with robot description
TEMP_YAML="/tmp/robot_description.yaml"
if [ ! -f "$URDF_FILE" ]; then
    echo "[ERROR] URDF not found: $URDF_FILE"
    exit 1
fi
cat > "$TEMP_YAML" << EOF
robot_state_publisher:
  ros__parameters:
    robot_description: |
$(cat "$URDF_FILE" | sed 's/^/      /')
EOF

# Start robot_state_publisher
echo "Starting robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args --params-file "$TEMP_YAML" &
RSP_PID=$!

# Start RViz2
echo "Starting RViz2..."
rviz2 -d "$RVIZ_CONFIG" &
RVIZ_PID=$!
sleep 3

# Position RViz window and maximize to fullscreen
sleep 2
if command -v wmctrl &>/dev/null; then
    for title in "RViz" "rviz" "RVIZ"; do
        wmctrl -r "$title" -e 0,0,0,$WIDTH,$HEIGHT 2>/dev/null && break
    done
    for title in "RViz" "rviz" "RVIZ"; do
        wmctrl -r "$title" -b add,maximized_vert,maximized_horz 2>/dev/null && echo "RViz maximized" && break
    done
elif command -v xdotool &>/dev/null; then
    WINDOW_ID=$(xdotool search --name -i "rviz" 2>/dev/null | head -1)
    [ -n "$WINDOW_ID" ] && {
        xdotool windowactivate $WINDOW_ID
        xdotool windowsize $WINDOW_ID $WIDTH $HEIGHT
        xdotool windowmove $WINDOW_ID 0 0
        xdotool key --window $WINDOW_ID F11
        echo "RViz positioned (xdotool F11)"
    }
fi

sleep 1

STREAMER_PID=""
# Prefer GStreamer (TCP 5600) - works without ZMQ, laptop connects directly
# Set STREAM_MODE=zmq to use ZMQ screenshot stream (port 5572) instead
if [ "${STREAM_MODE}" = "zmq" ] && [ -f "$SCRIPT_DIR/rviz_streamer.py" ]; then
    echo ""
    echo "Starting ZMQ screenshot stream on port $ZMQ_PORT (rviz_streamer.py)..."
    echo "Laptop GUI: Start RViz Stream (ZMQ subprocess)"
    echo ""
    python3 "$SCRIPT_DIR/rviz_streamer.py" --port "$ZMQ_PORT" --interval-ms 150 --jpeg-quality 75 &
    STREAMER_PID=$!
    wait $STREAMER_PID
else
    echo ""
    echo "Starting MJPEG stream on port $STREAM_PORT (${WIDTH}x${HEIGHT} @ ${FPS} fps)..."
    echo "Laptop GUI: Start RViz Stream (connects to $STREAM_PORT)"
    echo ""
    # GPU encode (nvjpegenc) — much faster than CPU jpegenc on Jetson. RVIZ_STREAM_CPU=1 to force CPU.
    if [ "${RVIZ_STREAM_CPU}" = "1" ]; then
        gst-launch-1.0 -v \
            ximagesrc display-name=$DISPLAY use-damage=true show-pointer=false \
            ! queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream \
            ! video/x-raw,framerate=$FPS/1 \
            ! videoscale method=0 ! video/x-raw,width=$WIDTH,height=$HEIGHT \
            ! videoconvert \
            ! jpegenc quality=$QUALITY \
            ! queue max-size-buffers=1 max-size-time=0 leaky=downstream \
            ! multipartmux boundary="--frame" \
            ! tcpserversink host=0.0.0.0 port=$STREAM_PORT sync=false buffers-max=1 buffers-soft-max=1
    else
        # nvjpegenc accepts I420; use-damage=true = only capture changed regions (faster)
        gst-launch-1.0 -v \
            ximagesrc display-name=$DISPLAY use-damage=true show-pointer=false \
            ! queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream \
            ! video/x-raw,framerate=$FPS/1 \
            ! videoscale method=0 ! video/x-raw,width=$WIDTH,height=$HEIGHT \
            ! videoconvert ! video/x-raw,format=I420 \
            ! nvjpegenc quality=$QUALITY \
            ! queue max-size-buffers=1 max-size-time=0 leaky=downstream \
            ! multipartmux boundary="--frame" \
            ! tcpserversink host=0.0.0.0 port=$STREAM_PORT sync=false buffers-max=1 buffers-soft-max=1
    fi
fi
