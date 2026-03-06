#!/bin/bash
echo "Logitech C920 Test Script"
echo "========================="
echo ""
echo "1. Detecting Logitech cameras..."
lsusb | grep -i logitech

echo ""
echo "2. Checking video devices..."
for dev in /dev/video*; do
    if v4l2-ctl --device=$dev --all 2>/dev/null | grep -qi "logitech\|c920"; then
        echo ""
        echo "Found Logitech camera: $dev"
        echo "Supported formats:"
        v4l2-ctl --device=$dev --list-formats-ext | grep -E "H264|MJPEG|YUYV|Size|Interval" | head -30
    fi
done

echo ""
echo "3. If you see H264 above, your C920 supports hardware compression!"
echo "   This will use MUCH less USB bandwidth than MJPEG cameras."
