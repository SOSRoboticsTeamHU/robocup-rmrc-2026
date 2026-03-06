"""
RoboCupRescue RMRC 2026 - ZMQ Message Protocol
==============================================
Message definitions for communication between laptop and Jetson.
All messages are JSON encoded.
"""

import json
import time
from dataclasses import dataclass, asdict
from typing import Optional, List, Dict, Any
from enum import Enum

# =============================================================================
# MESSAGE TYPES
# =============================================================================

class MessageType(str, Enum):
    # Drive commands
    DRIVE_CMD = "drive_cmd"
    DRIVE_STATUS = "drive_status"
    
    # Arm commands
    ARM_CMD = "arm_cmd"
    ARM_STATUS = "arm_status"
    ARM_TELEOP = "arm_teleop"
    
    # Camera
    CAMERA_FRAME = "camera_frame"
    CAMERA_CONFIG = "camera_config"
    
    # Lidar/SLAM
    LIDAR_SCAN = "lidar_scan"
    SLAM_MAP = "slam_map"
    SLAM_POSE = "slam_pose"
    
    # Detection results
    YOLO_DETECTION = "yolo_detection"
    QR_DETECTION = "qr_detection"
    SNAPSHOT_RESULT = "snapshot_result"  # HAZMAT/Landolt still image result
    
    # Autonomy
    AUTONOMY_CMD = "autonomy_cmd"
    AUTONOMY_STATUS = "autonomy_status"
    
    # System
    HEARTBEAT = "heartbeat"
    ERROR = "error"
    STATUS = "status"

# =============================================================================
# BASE MESSAGE
# =============================================================================

@dataclass
class BaseMessage:
    """Base class for all messages."""
    msg_type: str
    timestamp: float = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()
    
    def to_json(self) -> str:
        return json.dumps(asdict(self))
    
    def to_bytes(self) -> bytes:
        return self.to_json().encode('utf-8')
    
    @classmethod
    def from_json(cls, json_str: str) -> 'BaseMessage':
        data = json.loads(json_str)
        return cls(**data)
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'BaseMessage':
        return cls.from_json(data.decode('utf-8'))

# =============================================================================
# DRIVE MESSAGES
# =============================================================================

@dataclass
class DriveCommand(BaseMessage):
    """Drive command from joystick to robot."""
    msg_type: str = MessageType.DRIVE_CMD
    y: int = 0      # Forward/backward (-100 to 100)
    x: int = 0      # Curved turn left/right (-100 to 100)
    z: int = 0      # Spot rotate left/right (-100 to 100)
    emergency_stop: bool = False

@dataclass
class DriveStatus(BaseMessage):
    """Drive status from robot."""
    msg_type: str = MessageType.DRIVE_STATUS
    left_speed: int = 0
    right_speed: int = 0
    left_target: int = 0
    right_target: int = 0
    watchdog_ok: bool = True

# =============================================================================
# ARM MESSAGES
# =============================================================================

@dataclass
class ArmCommand(BaseMessage):
    """Arm position command."""
    msg_type: str = MessageType.ARM_CMD
    positions: Dict[str, float] = None  # Joint name -> position (degrees or %)
    gripper: float = 0  # Gripper position (0-100%)

@dataclass
class ArmTeleopData(BaseMessage):
    """Leader arm teleoperation data."""
    msg_type: str = MessageType.ARM_TELEOP
    joint_positions: Dict[str, float] = None  # shoulder_pan, shoulder_lift, etc.

@dataclass
class ArmStatus(BaseMessage):
    """Arm status from robot."""
    msg_type: str = MessageType.ARM_STATUS
    current_positions: Dict[str, float] = None
    target_positions: Dict[str, float] = None
    gripper_position: float = 0
    is_moving: bool = False

# =============================================================================
# CAMERA MESSAGES
# =============================================================================

@dataclass
class CameraFrame(BaseMessage):
    """Camera frame with optional detection overlays."""
    msg_type: str = MessageType.CAMERA_FRAME
    camera_id: str = ""
    width: int = 0
    height: int = 0
    format: str = "jpeg"  # jpeg, raw
    # Frame data is sent as separate binary message part
    detections: List[Dict] = None  # YOLO detections
    qr_codes: List[Dict] = None    # QR code detections

@dataclass
class CameraConfig(BaseMessage):
    """Camera configuration."""
    msg_type: str = MessageType.CAMERA_CONFIG
    camera_id: str = ""
    width: int = 640
    height: int = 480
    fps: int = 30
    enable_yolo: bool = True
    enable_qr: bool = True

# =============================================================================
# DETECTION MESSAGES
# =============================================================================

@dataclass
class Detection:
    """Single object detection."""
    class_name: str
    confidence: float
    bbox: List[int]  # [x1, y1, x2, y2]
    center: List[int] = None  # [cx, cy]

@dataclass
class YoloDetectionResult(BaseMessage):
    """YOLO detection results."""
    msg_type: str = MessageType.YOLO_DETECTION
    camera_id: str = ""
    detections: List[Dict] = None
    inference_time_ms: float = 0

@dataclass 
class QRDetectionResult(BaseMessage):
    """QR code detection results."""
    msg_type: str = MessageType.QR_DETECTION
    camera_id: str = ""
    codes: List[Dict] = None  # [{data: str, bbox: [...], size_mm: int}]
    
# =============================================================================
# LIDAR/SLAM MESSAGES
# =============================================================================

@dataclass
class LidarScan(BaseMessage):
    """Lidar scan data."""
    msg_type: str = MessageType.LIDAR_SCAN
    ranges: List[float] = None  # Distance measurements
    angles: List[float] = None  # Angle for each measurement
    intensities: List[float] = None
    imu_data: Dict = None  # {roll, pitch, yaw, accel_x, accel_y, accel_z}

@dataclass
class SLAMPose(BaseMessage):
    """Robot pose from SLAM."""
    msg_type: str = MessageType.SLAM_POSE
    x: float = 0
    y: float = 0
    z: float = 0
    roll: float = 0
    pitch: float = 0
    yaw: float = 0
    confidence: float = 1.0

@dataclass
class SLAMMap(BaseMessage):
    """Occupancy grid map. Cell values: -1 unknown, 0 free (safe/green), 100 occupied (danger/red)."""
    msg_type: str = MessageType.SLAM_MAP
    width: int = 0
    height: int = 0
    resolution: float = 0.05  # meters per cell
    origin_x: float = 0
    origin_y: float = 0
    grid: List[List[int]] = None  # 2D: -1 unknown, 0 safe, 100 danger
    # Or map data as separate binary message part

@dataclass
class PointCloudRegistered(BaseMessage):
    """Registered point cloud (e.g. /cloud_registered from Cartographer/Point-LIO). For ZMQ: list of [x,y,z] or flattened."""
    msg_type: str = "point_cloud_registered"
    frame_id: str = "map"
    points: List[List[float]] = None  # [[x,y,z], ...] or None if sent as binary
    width: int = 0
    height: int = 0
    is_dense: bool = False

# =============================================================================
# AUTONOMY MESSAGES
# =============================================================================

class AutonomyMode(str, Enum):
    DISABLED = "disabled"
    MAPPING = "mapping"
    WAYPOINT = "waypoint"
    INSPECTION = "inspection"
    MANIPULATION = "manipulation"

@dataclass
class AutonomyCommand(BaseMessage):
    """Autonomy control command."""
    msg_type: str = MessageType.AUTONOMY_CMD
    mode: str = AutonomyMode.DISABLED
    target: Dict = None  # Mode-specific target (waypoint, object, etc.)
    params: Dict = None  # Mode-specific parameters

@dataclass
class AutonomyStatus(BaseMessage):
    """Autonomy system status (includes Nav2 overlay for SLAM view)."""
    msg_type: str = MessageType.AUTONOMY_STATUS
    mode: str = AutonomyMode.DISABLED
    state: str = "idle"  # idle, running, paused, completed, error, aborting
    progress: float = 0  # 0-100%
    message: str = ""
    # Nav2 overlay for ego-centric SLAM
    nav2_path: Optional[List[List[float]]] = None  # [[x,y], ...]
    nav2_goal: Optional[List[float]] = None  # [x, y]
    goal_distance_m: Optional[float] = None
    step: Optional[str] = None  # e.g. "2/5"
    step_label: Optional[str] = None  # e.g. "Navigating to B"
    laps_completed: int = 0
    battery_voltage: Optional[float] = None
    nav2_stuck: bool = False

# =============================================================================
# SYSTEM MESSAGES
# =============================================================================

@dataclass
class Heartbeat(BaseMessage):
    """Heartbeat message for connection monitoring."""
    msg_type: str = MessageType.HEARTBEAT
    source: str = ""  # "laptop" or "jetson"
    uptime: float = 0

@dataclass
class ErrorMessage(BaseMessage):
    """Error notification."""
    msg_type: str = MessageType.ERROR
    source: str = ""
    error_code: str = ""
    message: str = ""
    recoverable: bool = True

@dataclass
class RobotStatus(BaseMessage):
    """Overall robot status."""
    msg_type: str = MessageType.STATUS
    battery_voltage: float = 0
    battery_percent: float = 0
    cpu_temp: float = 0
    gpu_temp: float = 0
    cpu_usage: float = 0
    memory_usage: float = 0
    disk_usage: float = 0
    wifi_signal: int = 0
    ethernet_connected: bool = False
    pico_connected: bool = False
    arm_connected: bool = False
    lidar_connected: bool = False
    cameras_connected: List[str] = None

# =============================================================================
# MESSAGE PARSING
# =============================================================================

MESSAGE_CLASSES = {
    MessageType.DRIVE_CMD: DriveCommand,
    MessageType.DRIVE_STATUS: DriveStatus,
    MessageType.ARM_CMD: ArmCommand,
    MessageType.ARM_TELEOP: ArmTeleopData,
    MessageType.ARM_STATUS: ArmStatus,
    MessageType.CAMERA_FRAME: CameraFrame,
    MessageType.CAMERA_CONFIG: CameraConfig,
    MessageType.YOLO_DETECTION: YoloDetectionResult,
    MessageType.QR_DETECTION: QRDetectionResult,
    MessageType.LIDAR_SCAN: LidarScan,
    MessageType.SLAM_POSE: SLAMPose,
    MessageType.SLAM_MAP: SLAMMap,
    MessageType.AUTONOMY_CMD: AutonomyCommand,
    MessageType.AUTONOMY_STATUS: AutonomyStatus,
    MessageType.HEARTBEAT: Heartbeat,
    MessageType.ERROR: ErrorMessage,
    MessageType.STATUS: RobotStatus,
}

def parse_message(json_str: str) -> BaseMessage:
    """Parse a JSON message into the appropriate message class."""
    data = json.loads(json_str)
    msg_type = data.get("msg_type")
    
    if msg_type in MESSAGE_CLASSES:
        cls = MESSAGE_CLASSES[msg_type]
        return cls(**data)
    
    return BaseMessage(**data)

def parse_message_bytes(data: bytes) -> BaseMessage:
    """Parse bytes into a message."""
    return parse_message(data.decode('utf-8'))
