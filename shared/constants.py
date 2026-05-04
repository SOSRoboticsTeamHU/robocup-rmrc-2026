"""
RoboCupRescue RMRC 2026 - Shared Constants
==========================================
Common constants used across laptop and Jetson components.
"""

# =============================================================================
# NETWORK CONFIGURATION
# =============================================================================

# Static IP addresses (Ethernet tether)
JETSON_IP = "192.168.2.100"
LAPTOP_IP = "192.168.2.101"
PI_IP = "192.168.2.102"  # Deprecated — no Pi cameras; all 3 on Jetson.

# ZMQ Ports
ZMQ_PORT_DRIVE = 5555      # Joystick commands -> Jetson
ZMQ_PORT_ARM = 5556        # Arm commands (JSON) -> Jetson
ZMQ_PORT_CAMERA = 5557     # Camera streams <- Jetson
ZMQ_PORT_ARM_TELEOP = 5558 # LeRobot arm teleop: leader sends msgpack to Jetson (followerarm.py)
ZMQ_PORT_STATUS = 5559     # Robot status <- Jetson
ZMQ_PORT_AUTONOMY = 5560   # Autonomy commands -> Jetson
ZMQ_PORT_CAMERA_CONTROL = 5561  # Laptop -> Jetson: enable/disable YOLO and QR (higher FPS when off)
ZMQ_PORT_SLAM = 5562       # SLAM pose + occupancy map <- Jetson (Point-LIO bridge)
ZMQ_PORT_LIDAR = 5563      # Lidar scan (ranges/angles) <- Jetson
ZMQ_PORT_CLOUD = 5564      # Point cloud (3D) <- Jetson (optional, for advanced viz)
ZMQ_PORT_AUTONOMY_STATUS = 5565  # Autonomy status (mapping progress) <- Jetson
ZMQ_PORT_SNAPSHOT_REQUEST = 5570  # Executor -> Vision: request HAZMAT/Landolt snapshot (vision subscribes)
ZMQ_PORT_SNAPSHOT_RESULT = 5571   # Jetson vision -> Laptop: snapshot_result (GUI subscribes)
ZMQ_PORT_RVIZ_STREAM = 5572       # RViz window screenshot stream (Jetson -> Laptop)
RVIZ_STREAM_TCP_PORT = 5600       # GStreamer MJPEG stream (Jetson -> Laptop, no ZMQ)

# ZMQ Connection strings
def zmq_connect_str(ip: str, port: int) -> str:
    return f"tcp://{ip}:{port}"

def zmq_bind_str(port: int) -> str:
    return f"tcp://*:{port}"

# =============================================================================
# SERIAL CONFIGURATION
# =============================================================================

# Pico serial port on Jetson
PICO_SERIAL_PORT = "/dev/ttyACM0"
PICO_SERIAL_BAUD = 115200

# SO-ARM101 serial port on Jetson (Feetech servo bus)
ARM_SERIAL_PORT = "/dev/ttyACM1"  # Adjust based on actual connection
# Back support (kickstand) servos on the same Feetech bus.
# After the May 2026 hardware change, two servos (IDs 7 + 8) are wired in
# parallel for additional torque, both mounted in the SAME orientation so they
# share an identical target position. To invert one of them mechanically,
# add its index to BACK_SUPPORT_SERVO_INVERTED.
BACK_SUPPORT_SERVO_IDS = (7, 8)
BACK_SUPPORT_SERVO_INVERTED = (False, False)
# Legacy single-servo alias kept for code paths that reference one ID directly.
BACK_SUPPORT_SERVO_ID = BACK_SUPPORT_SERVO_IDS[0]
BACK_SUPPORT_DOWN = 150    # Stowed (safe mechanical minimum)
BACK_SUPPORT_UP = 1820     # Deployed (prevent tipping backward)
BACK_SUPPORT_STEP = 60     # Increment per button press (raise/lower, not jump to min/max)

# Leader arm on laptop (LeRobot SO-ARM leader, USB serial)
ARM_LEADER_SERIAL = "/dev/tty.usbmodem5AE60833341"  # macOS; on Linux often /dev/ttyUSB0 or /dev/ttyACM*
ARM_LEADER_RATE_HZ = 100  # Teleop send rate to Jetson

# =============================================================================
# CAMERA CONFIGURATION
# =============================================================================

# Camera device paths on Jetson (3 USB cameras: front, arm, backward)
# Swapped so front = former arm device, arm = former front device (physical camera IDs)
CAMERAS = {
    "front": "/dev/camera_front",
    "arm": "/dev/camera_arm",
    "backward": "/dev/camera_backward",
}

# Camera resolution and FPS (640x480 @ 15fps for USB cameras)
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 15

# JPEG quality for streaming (0-100)
JPEG_QUALITY = 50

# =============================================================================
# LIDAR CONFIGURATION  
# =============================================================================

# Unitree L2 Lidar (via unilidar_sdk2 ROS2)
LIDAR_SERIAL_PORT = "/dev/ttyUSB1"  # Fallback for direct serial (not used with ROS2)
LIDAR_BAUD = 230400

# ROS2 Topics (Point-LIO / unilidar_sdk2)
ROS2_TOPIC_CLOUD = "/unilidar/cloud"           # Raw point cloud from L2
ROS2_TOPIC_IMU = "/unilidar/imu"               # IMU from L2
ROS2_TOPIC_CLOUD_REGISTERED = "/cloud_registered"  # Registered cloud from Point-LIO
ROS2_TOPIC_ODOMETRY = "/Odometry"              # Pose from Point-LIO
ROS2_TOPIC_PATH = "/path"                      # Trajectory from Point-LIO

# =============================================================================
# SLAM CONFIGURATION
# =============================================================================

# Occupancy grid parameters
SLAM_GRID_RESOLUTION = 0.05  # meters per cell
SLAM_GRID_SIZE = 200         # grid size (200x200 = 10m x 10m at 0.05m resolution)
SLAM_GRID_ORIGIN_X = -5.0    # meters (center of grid in world frame)
SLAM_GRID_ORIGIN_Y = -5.0

# Point cloud to grid conversion
SLAM_Z_MIN = -0.5    # meters - ignore points below this (ground)
SLAM_Z_MAX = 2.0     # meters - ignore points above this (ceiling)
SLAM_GRID_UPDATE_HZ = 20  # How often to update occupancy grid (real-time display)

# =============================================================================
# CONTROL PARAMETERS
# =============================================================================

# Joystick deadzone (0.0 - 1.0)
JOYSTICK_DEADZONE = 0.05

# Max drive speed (0-100)
MAX_DRIVE_SPEED = 100

# Control loop rates (Hz)
DRIVE_CONTROL_HZ = 50
ARM_CONTROL_HZ = 30
CAMERA_STREAM_HZ = 30
STATUS_UPDATE_HZ = 10

# Watchdog timeout (ms)
WATCHDOG_TIMEOUT_MS = 500

# LiDAR soft obstacle stop distance (meters)
LIDAR_OBSTACLE_STOP_M = 0.15

# =============================================================================
# YOLO CONFIGURATION
# =============================================================================

# YOLOv8 model paths
YOLO_MODEL_HAZMAT = "models/hazmat_yolov8n.engine"
YOLO_MODEL_LANDOLT = "models/landolt_yolov8n.engine"
YOLO_MODEL_GENERAL = "models/yolov8n.engine"

# Detection confidence threshold
YOLO_CONFIDENCE = 0.5

# =============================================================================
# VISION SYSTEM - Jetson Orin Nano Super
# =============================================================================

# Jetson Orin Nano Super optimization
JETSON_VISION_CONFIG = {
    "power_mode": "15W",
    "tensorrt_precision": "FP16",
    "camera_resolution": "640x480",
    "max_concurrent_models": 2,
}

# 3 Jetson cameras: front, arm, backward — 2 with AI (front, arm)
VISION_CAMERA_CONFIG = {
    "jetson_cameras": {
        "front": {"resolution": "640x480", "fps": 30, "ai": True},
        "arm": {"resolution": "640x480", "fps": 30, "ai": True},
        "backward": {"resolution": "640x480", "fps": 30, "ai": False},
    },
    "pi_cameras": {},
}

# All 3 Jetson camera IDs (order for display)
JETSON_CAMERAS = ("front", "arm", "backward")

# AI/YOLO camera IDs — 2 of 4 run HAZMAT/Landolt inference
YOLO_CAMERAS = tuple(
    cid for cid, cfg in VISION_CAMERA_CONFIG.get("jetson_cameras", {}).items()
    if cfg.get("ai", False)
)
if not YOLO_CAMERAS:
    YOLO_CAMERAS = ("front", "arm")  # fallback

# Landolt-C 8 directions (relative to identifier/up)
LANDOLT_DIRECTIONS = ["Up", "Up-Right", "Right", "Down-Right", "Down", "Down-Left", "Left", "Up-Left"]

# GPU-offload: When True, QR decoding runs on laptop (not Jetson) to free CPU for SLAM/Autonomy
# Set False and use --qr-on-jetson if pyzbar not installed on laptop
QR_DECODE_ON_LAPTOP = True

# Decode QR every Nth frame on laptop (1=every frame, 3=every 3rd). Lower = more responsive, higher CPU
QR_DECODE_INTERVAL = 2

# QR codes output file (rulebook: "information from QR codes must be written to a file")
# Each unique QR is appended when first scanned. Also included in mission report PDF.
QR_CODES_OUTPUT_FILE = "mission_qr_codes.txt"

# =============================================================================
# COMPETITION SPECIFIC
# =============================================================================

# Mission duration (seconds) - RMRC rulebook
MISSION_DURATION = 300       # Preliminaries: 5 minutes
MISSION_DURATION_FINALS = 900  # Finals: 15 minutes (RMRC 2025 rulebook)

# QR code size variants (mm)
QR_SIZES = [1, 2, 5, 10, 20]

# Landolt-C gap sizes (mm)
LANDOLT_SIZES = [1.0, 1.4, 1.8, 2.5, 3.6]

# Hazmat labels
HAZMAT_LABELS = [
    "Explosive",
    "Flammable Gas",
    "Non-Flammable Gas",
    "Oxygen",
    "Flammable Liquid",
    "Flammable Solid",
    "Spontaneously Combustible",
    "Dangerous When Wet",
    "Oxidizer",
    "Organic Peroxide",
    "Poison",
    "Radioactive",
    "Corrosive",
    "Miscellaneous Dangerous Goods",
]

# Point values
POINTS_LAP_TELEOP = 1
POINTS_LAP_AUTO = 4
POINTS_QR_READ = 1
POINTS_QR_MAPPED = 4
POINTS_HAZMAT = 1
POINTS_LANDOLT = 1
POINTS_THERMAL = 1
POINTS_MAGNET = 1
POINTS_DOOR_OPEN = 1
POINTS_KEYPAD_5_CLEAN = 3
POINTS_KEYPAD_5_DIRTY = 1
POINTS_OBJECT_REMOVED = 1
POINTS_OBJECT_IN_CONTAINER = 2
# Drop test (finals only)
POINTS_DROP_15 = 1
POINTS_DROP_30 = 2

# Autonomy multiplier
AUTONOMY_MULTIPLIER = 4

# Dexterity autonomy 30 cm guard (rulebook changelog 2026-04-12).
# Robot must be at least this distance from the target object before an autonomous
# dexterity behavior is accepted, otherwise the 4x multiplier is voided.
DEXTERITY_AUTONOMY_MIN_DIST_M = 0.30

# Best-in-Class breadth target (rulebook): need positive non-zero score in >=80%
# of available tests (>=15 of 18) to be eligible for any Best-in-Class certificate.
BIC_BREADTH_TARGET = 0.80

# Tether mode default. Override via USE_TETHER_MODE=1 env var on Jetson startup.
TETHER_MODE_DEFAULT = False

# Snapshot zero-false-positive gating (Sensor Cabinet HAZMAT/Landolt autonomy).
# A claim is only made if the same class/direction is seen in N consecutive frames
# at >= SNAPSHOT_MIN_CONFIDENCE.
SNAPSHOT_FRAME_CONSISTENCY_N = 3
SNAPSHOT_MIN_CONFIDENCE = 0.75

# Stowed posture (RMRC: required for autonomy bonus)
# Operator must confirm robot is stowed before starting Sensor Cabinet / Keypad autonomy
STOWED_POSTURE_REQUIRED_FOR_AUTONOMY = True

# QR-on-map (Labyrinth): 4 extra points per QR if placement is accurate to within 30 cm of two closest fiducials
QR_MAP_VALIDATION_DISTANCE_M = 0.30
# Fiducial positions in world frame (meters). Set per-arena from competition photos; used for map display and validation.
# Pre-fill from competition arena photos. Example: [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]
FIDUCIAL_POSITIONS = []  # TODO: fill from competition arena layout photos

# GUI modes (OperatorStation) — rulebook: teleop vs autonomy vs snapshot_review
GUI_MODES = ("teleop", "autonomy", "snapshot_review")

# Test types for selector and per-test scoring (grouped: Mobility, Dexterity, Sensing, Other)
TEST_TYPES = [
    # Mobility
    "incline_horiz", "incline_inclined", "sand_gravel", "ramps_continuous", "ramps_pinwheel",
    "elevated_ramps", "krails_horiz", "krails_crossover", "hurdles_single", "hurdles_double",
    # Dexterity
    "keypad_omni", "linear_rail_pick", "linear_rail_inspect",
    # Sensing
    "sensor_cabinet", "labyrinth_flat", "labyrinth_krails",
    # Other
    "stairs", "align", "drop_test",
]

# Display names for test selector (grouped)
TEST_GROUPS = [
    ("Mobility", [
        ("Incline Horizontal", "incline_horiz"),
        ("Incline Inclined", "incline_inclined"),
        ("Sand & Gravel", "sand_gravel"),
        ("Ramps Continuous", "ramps_continuous"),
        ("Ramps Pinwheel", "ramps_pinwheel"),
        ("Elevated Ramps", "elevated_ramps"),
        ("K-Rails Horizontal", "krails_horiz"),
        ("K-Rails Crossover", "krails_crossover"),
        ("Hurdles Single", "hurdles_single"),
        ("Hurdles Double", "hurdles_double"),
    ]),
    ("Dexterity", [
        ("Keypad Omni", "keypad_omni"),
        ("Linear Rail Pick", "linear_rail_pick"),
        ("Linear Rail Inspect", "linear_rail_inspect"),
    ]),
    ("Sensing", [
        ("Sensor Cabinet", "sensor_cabinet"),
        ("Labyrinth Flat", "labyrinth_flat"),
        ("Labyrinth K-Rails", "labyrinth_krails"),
    ]),
    ("Other", [
        ("Stairs", "stairs"),
        ("Align", "align"),
        ("Drop Test", "drop_test"),
    ]),
]

# Linear Rail Pick modes (rulebook allowed)
LINEAR_RAIL_PICK_MODES = [
    ("Full Autonomy", "full_auto"),
    ("Pause for Base Teleop", "pause_teleop"),
    ("Full Teleop (arm only)", "full_teleop"),
]

# Nav2 / autonomy
NAV2_MAX_LINEAR_VEL_MPS = 0.3   # m/s — maps to ~100% motor when scaling cmd_vel
NAV2_MAX_ANGULAR_VEL_RPS = 1.0  # rad/s
NAV2_STUCK_TIMEOUT_S = 15       # Auto ABORT if Nav2 no progress
END_WALL_EXCEPTION_M = 0.30     # Rulebook: joystick allowed within 30 cm of end walls
