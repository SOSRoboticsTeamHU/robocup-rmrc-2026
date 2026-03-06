#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Camera pipeline tests
================================================
Tests for camera constants, protocol, and (when available) capture.
"""

import unittest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from shared.constants import CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS, ZMQ_PORT_CAMERA
from shared.protocol import CameraFrame, CameraConfig


class TestCameraConstants(unittest.TestCase):
    def test_resolution_positive(self):
        self.assertGreater(CAMERA_WIDTH, 0)
        self.assertGreater(CAMERA_HEIGHT, 0)

    def test_fps_positive(self):
        self.assertGreater(CAMERA_FPS, 0)

    def test_zmq_port_valid(self):
        self.assertGreater(ZMQ_PORT_CAMERA, 1024)


class TestCameraProtocol(unittest.TestCase):
    def test_camera_frame_defaults(self):
        f = CameraFrame()
        self.assertEqual(f.format, "jpeg")

    def test_camera_config_defaults(self):
        c = CameraConfig()
        self.assertEqual(c.width, 640)
        self.assertEqual(c.height, 480)


if __name__ == "__main__":
    unittest.main()
