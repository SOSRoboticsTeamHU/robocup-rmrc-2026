#!/bin/bash
# Test script: receive first 500 bytes from GStreamer stream and hexdump
# Run on laptop: ./test_rviz_stream.sh 192.168.2.100
# Ensure stream_rviz.sh is running on Jetson first.

HOST=${1:-192.168.2.100}
PORT=${2:-5600}
echo "Connecting to $HOST:$PORT (ensure stream_rviz.sh is running on Jetson)..."
timeout 3 nc "$HOST" "$PORT" 2>/dev/null | head -c 500 | xxd | head -40
echo ""
echo "Look for: JPEG start ff d8, or multipart --frame"
