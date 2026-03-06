#!/bin/bash
# List available /dev/video* devices and suggest mapping for robot_params.yaml
echo "=== Available video devices ==="
ls -la /dev/video* 2>/dev/null || echo "No /dev/video* found"
echo ""
echo "=== v4l2 device info (if v4l2-ctl installed) ==="
for d in /dev/video*; do
  [ -e "$d" ] || continue
  echo "--- $d ---"
  v4l2-ctl -d "$d" --info 2>/dev/null | head -3 || true
done
echo ""
echo "Suggested robot_params.yaml cameras section:"
echo "# Run this script to discover. Common layouts:"
echo "#   3 cams: video0,2,4 (front, arm, backward)"
echo "#   3 cams: video0,2,4"
echo "#   Edit jetson/config/robot_params.yaml to match your hardware"
