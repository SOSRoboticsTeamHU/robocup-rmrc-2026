#!/bin/bash
#
# RoboCupRescue RMRC 2026 - Jetson Orin Nano Setup Script
# =======================================================
# This script sets up the Jetson Orin Nano with all required software.
# Run with: sudo bash setup_jetson.sh
#
# Prerequisites:
# - JetPack 6.0+ installed
# - Internet connection for initial setup
#

set -e  # Exit on error

echo "=============================================="
echo "RoboCupRescue RMRC 2026 - Jetson Setup"
echo "=============================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    log_error "Please run as root (sudo bash setup_jetson.sh)"
    exit 1
fi

# Get the actual user (not root)
ACTUAL_USER=${SUDO_USER:-$USER}
ACTUAL_HOME=$(eval echo ~$ACTUAL_USER)

log_info "Setting up for user: $ACTUAL_USER"
log_info "Home directory: $ACTUAL_HOME"

# =============================================================================
# SYSTEM UPDATE
# =============================================================================

log_info "Updating system packages..."
apt update && apt upgrade -y

# =============================================================================
# STATIC IP CONFIGURATION
# =============================================================================

log_info "Configuring static IP for Ethernet (192.168.1.1)..."

# Create netplan configuration for static IP
NETPLAN_FILE="/etc/netplan/01-ethernet-static.yaml"

cat > $NETPLAN_FILE << 'EOF'
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      addresses:
        - 192.168.1.1/24
      routes:
        - to: 192.168.1.0/24
          via: 192.168.1.1
      nameservers:
        addresses: []
      optional: true
EOF

chmod 600 $NETPLAN_FILE
log_info "Applying network configuration..."
netplan apply || log_warn "Netplan apply failed - may need reboot"

# =============================================================================
# ROS2 HUMBLE INSTALLATION
# =============================================================================

log_info "Installing ROS2 Humble..."

# Add ROS2 apt repository
apt install -y software-properties-common curl
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt update

# Install ROS2 Humble desktop (includes rviz2, rqt, etc.)
apt install -y ros-humble-desktop

# Install additional ROS2 packages
apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-tf2-tools \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-compressed-image-transport \
    python3-colcon-common-extensions \
    python3-rosdep

# Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init
fi
su - $ACTUAL_USER -c "rosdep update"

# Add ROS2 to bashrc
if ! grep -q "source /opt/ros/humble/setup.bash" $ACTUAL_HOME/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> $ACTUAL_HOME/.bashrc
fi

log_info "ROS2 Humble installed successfully"

# =============================================================================
# PYTHON DEPENDENCIES
# =============================================================================

log_info "Installing Python dependencies..."

apt install -y \
    python3-pip \
    python3-venv \
    python3-opencv \
    python3-numpy \
    python3-serial

# Install pip packages
pip3 install --upgrade pip
pip3 install \
    pyzmq \
    pyzbar \
    pillow \
    scipy \
    transforms3d \
    pyserial \
    opencv-python-headless

# =============================================================================
# YOLO AND TENSORRT
# =============================================================================

log_info "Installing YOLO and TensorRT dependencies..."

# TensorRT should be pre-installed with JetPack
# Install ultralytics for YOLOv8
pip3 install ultralytics

# Create models directory
mkdir -p $ACTUAL_HOME/robocup_rescue_2026/jetson/models
chown -R $ACTUAL_USER:$ACTUAL_USER $ACTUAL_HOME/robocup_rescue_2026

log_info "Download YOLOv8 model and export to TensorRT with:"
log_info "  python3 -c \"from ultralytics import YOLO; model = YOLO('yolov8n.pt'); model.export(format='engine')\""

# =============================================================================
# SERIAL PORT PERMISSIONS
# =============================================================================

log_info "Setting up serial port permissions..."

# Add user to dialout group for serial port access
usermod -a -G dialout $ACTUAL_USER

# Create udev rules for consistent device naming
cat > /etc/udev/rules.d/99-rescue-robot.rules << 'EOF'
# Raspberry Pi Pico (Drive Controller)
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0005", SYMLINK+="pico", MODE="0666"

# Feetech Servo Controller (SO-ARM101)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="arm_servo", MODE="0666"

# Unitree L2 Lidar
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar", MODE="0666"
EOF

udevadm control --reload-rules
udevadm trigger

log_info "Serial ports configured. Devices will be available at /dev/pico, /dev/arm_servo, /dev/lidar"

# =============================================================================
# CAMERA SETUP
# =============================================================================

log_info "Setting up camera permissions..."

# Add user to video group
usermod -a -G video $ACTUAL_USER

# Increase USB buffer size for multiple cameras
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="*",  ATTR{bMaxPower}="500mA"' > /etc/udev/rules.d/99-usb-power.rules

# Set v4l2 buffer size
if ! grep -q "usbcore.usbfs_memory_mb" /etc/default/grub; then
    sed -i 's/GRUB_CMDLINE_LINUX_DEFAULT="/GRUB_CMDLINE_LINUX_DEFAULT="usbcore.usbfs_memory_mb=1000 /' /etc/default/grub
    update-grub
    log_warn "USB buffer size increased - reboot required"
fi

# =============================================================================
# LEROBOT SETUP
# =============================================================================

log_info "Setting up LeRobot for arm control..."

# Clone lerobot if not exists
if [ ! -d "$ACTUAL_HOME/lerobot" ]; then
    su - $ACTUAL_USER -c "git clone https://github.com/huggingface/lerobot.git $ACTUAL_HOME/lerobot"
fi

# Install lerobot
cd $ACTUAL_HOME/lerobot
su - $ACTUAL_USER -c "pip3 install -e ."

log_info "LeRobot installed"

# =============================================================================
# UNITREE LIDAR SETUP
# =============================================================================

log_info "Setting up Unitree L2 Lidar SDK..."

# Clone unilidar SDK
if [ ! -d "$ACTUAL_HOME/unilidar_sdk" ]; then
    su - $ACTUAL_USER -c "git clone https://github.com/unitreerobotics/unilidar_sdk.git $ACTUAL_HOME/unilidar_sdk"
fi

# Build the SDK
cd $ACTUAL_HOME/unilidar_sdk
mkdir -p build && cd build
cmake ..
make -j$(nproc)

log_info "Unitree Lidar SDK built"

# =============================================================================
# ROS2 WORKSPACE SETUP
# =============================================================================

log_info "Setting up ROS2 workspace..."

ROS2_WS="$ACTUAL_HOME/robocup_rescue_2026/jetson/ros2_ws"
mkdir -p $ROS2_WS/src
chown -R $ACTUAL_USER:$ACTUAL_USER $ROS2_WS

# Create package structure
PACKAGE_DIR="$ROS2_WS/src/rescue_robot"
mkdir -p $PACKAGE_DIR/{rescue_robot,launch,config}

# Create package.xml
cat > $PACKAGE_DIR/package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rescue_robot</name>
  <version>1.0.0</version>
  <description>RoboCupRescue RMRC 2026 Robot Package</description>
  <maintainer email="team@sosrobotics.hu">SOS Robotics Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>image_transport</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

# Create CMakeLists.txt
cat > $PACKAGE_DIR/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(rescue_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
EOF

# Create __init__.py
touch $PACKAGE_DIR/rescue_robot/__init__.py

chown -R $ACTUAL_USER:$ACTUAL_USER $ROS2_WS

# Add workspace to bashrc
if ! grep -q "source.*rescue_robot" $ACTUAL_HOME/.bashrc; then
    echo "source $ROS2_WS/install/setup.bash 2>/dev/null || true" >> $ACTUAL_HOME/.bashrc
fi

log_info "ROS2 workspace created at $ROS2_WS"

# =============================================================================
# PERFORMANCE OPTIMIZATION
# =============================================================================

log_info "Applying performance optimizations..."

# Set Jetson to max performance mode
if command -v nvpmodel &> /dev/null; then
    nvpmodel -m 0  # MAXN mode
    jetson_clocks
    log_info "Jetson set to maximum performance mode"
fi

# Disable GUI for headless operation (optional)
# systemctl set-default multi-user.target

# =============================================================================
# FIREWALL CONFIGURATION
# =============================================================================

log_info "Configuring firewall for ZMQ ports..."

# Allow ZMQ ports
ufw allow 5555:5565/tcp comment "ZMQ ports for robot communication"
ufw allow from 192.168.1.0/24 comment "Allow local network"

# Enable firewall if not already
ufw --force enable

# =============================================================================
# CREATE STARTUP SERVICE
# =============================================================================

log_info "Creating robot startup service..."

cat > /etc/systemd/system/rescue-robot.service << EOF
[Unit]
Description=RoboCupRescue Robot Services
After=network.target

[Service]
Type=simple
User=$ACTUAL_USER
WorkingDirectory=$ACTUAL_HOME/robocup_rescue_2026/jetson
ExecStart=/bin/bash $ACTUAL_HOME/robocup_rescue_2026/jetson/scripts/start_robot.sh
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
# Don't enable by default - let user enable when ready
# systemctl enable rescue-robot.service

log_info "Startup service created (not enabled - run 'sudo systemctl enable rescue-robot.service' when ready)"

# =============================================================================
# SUMMARY
# =============================================================================

echo ""
echo "=============================================="
echo "Setup Complete!"
echo "=============================================="
echo ""
log_info "Installed components:"
echo "  - ROS2 Humble"
echo "  - Navigation2"
echo "  - SLAM Toolbox"
echo "  - Python dependencies (zmq, pyzbar, ultralytics)"
echo "  - LeRobot for arm control"
echo "  - Unitree Lidar SDK"
echo "  - Serial port permissions"
echo ""
log_info "Network configuration:"
echo "  - Static IP: 192.168.1.1"
echo "  - ZMQ ports: 5555-5565"
echo ""
log_warn "IMPORTANT: Please reboot the system to apply all changes!"
echo ""
log_info "After reboot, test with:"
echo "  - Check Pico: ls -la /dev/ttyACM0 or /dev/pico"
echo "  - Check cameras: ls -la /dev/video*"
echo "  - Check ROS2: ros2 topic list"
echo "  - Ping laptop: ping 192.168.1.2"
echo ""
