#!/bin/bash
# SLAM + 3rd Person Visualizer indítása
# Használat: ./start_slam_viz.sh [robot|laptop|full]

MODE=${1:-full}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== SLAM 3rd Person Visualizer ==="
echo "Mode: $MODE"

# Source ROS2
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/iron/setup.bash 2>/dev/null

# Source Point-LIO workspace if exists
if [ -f ~/ws_livox/install/setup.bash ]; then
    source ~/ws_livox/install/setup.bash
fi

case $MODE in
    robot)
        echo "Robot mode - csak state publisher (GUI nélkül)"
        echo "A laptop-on futtasd: ./start_slam_viz.sh laptop"
        ros2 launch $SCRIPT_DIR/slam_visualizer.launch.py mode:=robot
        ;;
    laptop)
        echo "Laptop mode - csak RViz GUI"
        echo "Győződj meg róla, hogy ROS_DOMAIN_ID egyezik a robottal!"
        ros2 launch $SCRIPT_DIR/slam_visualizer.launch.py mode:=laptop
        ;;
    full)
        echo "Full mode - state publisher + RViz"
        ros2 launch $SCRIPT_DIR/slam_visualizer.launch.py mode:=full
        ;;
    *)
        echo "Ismeretlen mode: $MODE"
        echo "Használat: $0 [robot|laptop|full]"
        exit 1
        ;;
esac
