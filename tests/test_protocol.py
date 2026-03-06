#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Protocol Tests
========================================
Tests for ZMQ message protocol and serialization.
"""

import sys
import os
import json
import time
import unittest

# Add shared module to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'shared'))

from shared.protocol import (
    BaseMessage, DriveCommand, DriveStatus,
    ArmCommand, CameraFrame, RobotStatus,
    MessageType, parse_message, parse_message_bytes
)
from shared.constants import (
    JETSON_IP, LAPTOP_IP, ZMQ_PORT_DRIVE,
    JOYSTICK_DEADZONE, MAX_DRIVE_SPEED
)


class TestBaseMessage(unittest.TestCase):
    """Tests for BaseMessage class."""
    
    def test_timestamp_auto_set(self):
        """Test that timestamp is automatically set."""
        msg = BaseMessage(msg_type="test")
        self.assertIsNotNone(msg.timestamp)
        self.assertGreater(msg.timestamp, 0)
    
    def test_to_json(self):
        """Test JSON serialization."""
        msg = BaseMessage(msg_type="test", timestamp=12345.0)
        json_str = msg.to_json()
        data = json.loads(json_str)
        self.assertEqual(data["msg_type"], "test")
        self.assertEqual(data["timestamp"], 12345.0)
    
    def test_to_bytes(self):
        """Test bytes serialization."""
        msg = BaseMessage(msg_type="test")
        data = msg.to_bytes()
        self.assertIsInstance(data, bytes)
        self.assertIn(b"test", data)


class TestDriveCommand(unittest.TestCase):
    """Tests for DriveCommand message."""
    
    def test_default_values(self):
        """Test default values."""
        cmd = DriveCommand()
        self.assertEqual(cmd.y, 0)
        self.assertEqual(cmd.x, 0)
        self.assertEqual(cmd.z, 0)
        self.assertFalse(cmd.emergency_stop)
        self.assertEqual(cmd.msg_type, MessageType.DRIVE_CMD)
    
    def test_custom_values(self):
        """Test custom values."""
        cmd = DriveCommand(y=50, x=-30, z=10, emergency_stop=True)
        self.assertEqual(cmd.y, 50)
        self.assertEqual(cmd.x, -30)
        self.assertEqual(cmd.z, 10)
        self.assertTrue(cmd.emergency_stop)
    
    def test_serialization_roundtrip(self):
        """Test serialization and deserialization."""
        cmd = DriveCommand(y=75, x=25, z=-50)
        json_str = cmd.to_json()
        
        data = json.loads(json_str)
        self.assertEqual(data["y"], 75)
        self.assertEqual(data["x"], 25)
        self.assertEqual(data["z"], -50)


class TestDriveStatus(unittest.TestCase):
    """Tests for DriveStatus message."""
    
    def test_default_values(self):
        """Test default values."""
        status = DriveStatus()
        self.assertEqual(status.left_speed, 0)
        self.assertEqual(status.right_speed, 0)
        self.assertTrue(status.watchdog_ok)
    
    def test_motor_speeds(self):
        """Test motor speed values."""
        status = DriveStatus(
            left_speed=50,
            right_speed=-50,
            left_target=60,
            right_target=-60,
            watchdog_ok=True
        )
        self.assertEqual(status.left_speed, 50)
        self.assertEqual(status.right_speed, -50)


class TestCameraFrame(unittest.TestCase):
    """Tests for CameraFrame message."""
    
    def test_default_values(self):
        """Test default values."""
        frame = CameraFrame()
        self.assertEqual(frame.camera_id, "")
        self.assertEqual(frame.format, "jpeg")
    
    def test_with_detections(self):
        """Test with detection data."""
        detections = [
            {"class_name": "hazmat", "confidence": 0.95, "bbox": [10, 20, 100, 200]}
        ]
        frame = CameraFrame(
            camera_id="front",
            width=640,
            height=480,
            detections=detections
        )
        self.assertEqual(frame.camera_id, "front")
        self.assertEqual(len(frame.detections), 1)
        self.assertEqual(frame.detections[0]["class_name"], "hazmat")


class TestMessageParsing(unittest.TestCase):
    """Tests for message parsing functions."""
    
    def test_parse_drive_command(self):
        """Test parsing a drive command."""
        json_str = json.dumps({
            "msg_type": "drive_cmd",
            "timestamp": time.time(),
            "y": 50,
            "x": 25,
            "z": 0,
            "emergency_stop": False
        })
        
        msg = parse_message(json_str)
        self.assertIsInstance(msg, DriveCommand)
        self.assertEqual(msg.y, 50)
        self.assertEqual(msg.x, 25)
    
    def test_parse_drive_status(self):
        """Test parsing a drive status."""
        json_str = json.dumps({
            "msg_type": "drive_status",
            "timestamp": time.time(),
            "left_speed": 40,
            "right_speed": 40,
            "watchdog_ok": True
        })
        
        msg = parse_message(json_str)
        self.assertIsInstance(msg, DriveStatus)
        self.assertEqual(msg.left_speed, 40)
    
    def test_parse_bytes(self):
        """Test parsing from bytes."""
        data = b'{"msg_type": "drive_cmd", "y": 100, "x": 0, "z": 0}'
        msg = parse_message_bytes(data)
        self.assertEqual(msg.y, 100)
    
    def test_parse_unknown_type(self):
        """Test parsing unknown message type."""
        json_str = json.dumps({
            "msg_type": "unknown_type",
            "timestamp": time.time()
        })
        
        msg = parse_message(json_str)
        self.assertIsInstance(msg, BaseMessage)


class TestConstants(unittest.TestCase):
    """Tests for shared constants."""
    
    def test_ip_addresses(self):
        """Test IP address format."""
        import re
        ip_pattern = r'^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$'
        self.assertRegex(JETSON_IP, ip_pattern)
        self.assertRegex(LAPTOP_IP, ip_pattern)
    
    def test_port_ranges(self):
        """Test port numbers are valid."""
        self.assertGreater(ZMQ_PORT_DRIVE, 1024)
        self.assertLess(ZMQ_PORT_DRIVE, 65535)
    
    def test_control_parameters(self):
        """Test control parameters are sensible."""
        self.assertGreaterEqual(JOYSTICK_DEADZONE, 0)
        self.assertLessEqual(JOYSTICK_DEADZONE, 1)
        self.assertGreater(MAX_DRIVE_SPEED, 0)
        self.assertLessEqual(MAX_DRIVE_SPEED, 100)


class TestArcadeDriveComputation(unittest.TestCase):
    """Tests for arcade drive motor speed computation."""
    
    def compute_motor_speeds(self, y, x, z, deadzone=5):
        """Compute motor speeds matching drive_bridge.py mix().
        y=forward, x=curve turn, z=spot rotate.
        left = fwd - turn - rot, right = fwd + turn + rot.
        """
        y = y if abs(y) >= deadzone else 0
        x = x if abs(x) >= deadzone else 0
        z = z if abs(z) >= deadzone else 0
        left = y - x - z
        right = y + x + z
        
        # Normalize
        max_val = max(abs(left), abs(right))
        if max_val > 100:
            scale = 100.0 / max_val
            left = int(left * scale)
            right = int(right * scale)
        
        return int(left), int(right)
    
    def test_forward(self):
        """Test forward movement."""
        left, right = self.compute_motor_speeds(100, 0, 0)
        self.assertEqual(left, 100)
        self.assertEqual(right, 100)
    
    def test_backward(self):
        """Test backward movement."""
        left, right = self.compute_motor_speeds(-100, 0, 0)
        self.assertEqual(left, -100)
        self.assertEqual(right, -100)
    
    def test_turn_right(self):
        """Test curve turn (x>0): left slows, right speeds up (matches drive_bridge)."""
        left, right = self.compute_motor_speeds(50, 50, 0)
        self.assertEqual(left, 0)
        self.assertEqual(right, 100)
    
    def test_spot_turn_right(self):
        """Test spot turn (z>0): left reverses, right forwards (matches drive_bridge)."""
        left, right = self.compute_motor_speeds(0, 0, 100)
        self.assertEqual(left, -100)
        self.assertEqual(right, 100)
    
    def test_spot_turn_left(self):
        """Test spot turn (z<0): left forwards, right reverses (matches drive_bridge)."""
        left, right = self.compute_motor_speeds(0, 0, -100)
        self.assertEqual(left, 100)
        self.assertEqual(right, -100)
    
    def test_deadzone(self):
        """Test deadzone filtering."""
        left, right = self.compute_motor_speeds(3, 2, 1)
        self.assertEqual(left, 0)
        self.assertEqual(right, 0)
    
    def test_normalization(self):
        """Test speed normalization when exceeding 100."""
        left, right = self.compute_motor_speeds(100, 100, 0)
        self.assertLessEqual(abs(left), 100)
        self.assertLessEqual(abs(right), 100)


if __name__ == '__main__':
    unittest.main(verbosity=2)
