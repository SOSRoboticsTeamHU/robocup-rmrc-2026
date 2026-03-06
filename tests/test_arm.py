#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Arm control tests
===========================================
Tests for arm protocol and constants (SO-ARM101 / LeRobot).
"""

import unittest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from shared.constants import ZMQ_PORT_ARM
from shared.protocol import ArmCommand, ArmStatus, MessageType


class TestArmConstants(unittest.TestCase):
    def test_arm_port_valid(self):
        self.assertGreater(ZMQ_PORT_ARM, 1024)


class TestArmProtocol(unittest.TestCase):
    def test_arm_command_type(self):
        cmd = ArmCommand()
        self.assertEqual(cmd.msg_type, MessageType.ARM_CMD)

    def test_arm_status_type(self):
        status = ArmStatus()
        self.assertEqual(status.msg_type, MessageType.ARM_STATUS)


if __name__ == "__main__":
    unittest.main()
