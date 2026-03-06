#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Drive system tests
=============================================
Unit tests for arcade/tank mixing, protocol, and drive constants.
"""

import unittest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
# Use shared protocol/constants
from shared.constants import DRIVE_CONTROL_HZ, WATCHDOG_TIMEOUT_MS, PICO_SERIAL_PORT
from shared.protocol import DriveCommand, DriveStatus


class TestDriveConstants(unittest.TestCase):
    def test_drive_control_hz_positive(self):
        self.assertGreater(DRIVE_CONTROL_HZ, 0)

    def test_watchdog_timeout_positive(self):
        self.assertGreater(WATCHDOG_TIMEOUT_MS, 0)


class TestDriveCommand(unittest.TestCase):
    def test_drive_command_defaults(self):
        cmd = DriveCommand()
        self.assertEqual(cmd.y, 0)
        self.assertEqual(cmd.x, 0)
        self.assertEqual(cmd.z, 0)

    def test_drive_command_serialize(self):
        cmd = DriveCommand(y=50, x=-10, z=0)
        d = cmd.to_json()
        self.assertIn("50", d)
        self.assertIn("-10", d)


if __name__ == "__main__":
    unittest.main()
