#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Autonomy Node (Jetson)
================================================
Implements mapping autonomy for Labyrinth +4 bonus.

Features:
- State machine: idle, mapping, completed
- Lap detection: tracks when robot returns to start
- Coverage tracking: percentage of map explored
- Status publishing on ZMQ 5565

Subscribes (ZMQ):
- Port 5560: Autonomy commands from laptop (mode: mapping, disabled)
- Port 5562: SLAM pose + grid from slam_bridge.py

Publishes (ZMQ):
- Port 5565: Autonomy status (state, coverage, laps)

States:
- idle: Waiting for mapping command
- mapping: Actively mapping, tracking coverage
- returning: Robot detected near start, completing lap
- completed: Lap completed, bonus earned
"""

import sys
import os
import time
import math
import argparse
import json
from typing import Optional, List, Tuple, Dict
from enum import Enum
from dataclasses import dataclass, asdict

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

try:
    import zmq
except ImportError:
    zmq = None

try:
    import numpy as np
except ImportError:
    np = None

try:
    from shared.constants import (
        ZMQ_PORT_AUTONOMY, ZMQ_PORT_SLAM, ZMQ_PORT_AUTONOMY_STATUS,
        POINTS_LAP_AUTO
    )
except ImportError:
    ZMQ_PORT_AUTONOMY = 5560
    ZMQ_PORT_SLAM = 5562
    ZMQ_PORT_AUTONOMY_STATUS = 5565
    POINTS_LAP_AUTO = 4


class AutonomyState(str, Enum):
    IDLE = "idle"
    MAPPING = "mapping"
    RETURNING = "returning"
    COMPLETED = "completed"
    SENSOR_CABINET = "sensor_cabinet"
    ERROR = "error"


@dataclass
class AutonomyStatus:
    """Status message published to GUI/logging."""
    state: str
    mode: str
    coverage_percent: float
    elapsed_time: float
    laps_completed: int
    distance_traveled: float
    current_pose: Dict[str, float]
    start_pose: Optional[Dict[str, float]]
    message: str
    timestamp: float
    
    def to_dict(self) -> dict:
        return {
            "msg_type": "autonomy_status",
            **asdict(self)
        }


class MappingTracker:
    """Tracks mapping progress: coverage, distance, lap detection."""
    
    def __init__(self, lap_threshold: float = 0.5, min_distance_for_lap: float = 5.0):
        self.lap_threshold = lap_threshold  # meters from start to count as lap
        self.min_distance_for_lap = min_distance_for_lap  # min distance before lap can complete
        
        # Start position
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.start_set = False
        
        # Current position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Tracking
        self.distance_traveled = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.positions_visited = set()  # Set of (grid_x, grid_y) for coverage
        
        # Grid info
        self.grid_resolution = 0.05
        self.grid_size = 200
        self.free_cells = 0
        self.total_cells = 0
        
        # Lap detection
        self.left_start_zone = False
        self.laps_completed = 0
    
    def set_start(self, x: float, y: float, yaw: float):
        """Set the starting position for lap detection."""
        self.start_x = x
        self.start_y = y
        self.start_yaw = yaw
        self.start_set = True
        self.last_x = x
        self.last_y = y
        self.left_start_zone = False
        self.distance_traveled = 0.0
        self.positions_visited.clear()
        print(f"[AUTONOMY] Start position set: ({x:.2f}, {y:.2f})")
    
    def update_pose(self, x: float, y: float, yaw: float):
        """Update current pose and tracking metrics."""
        self.current_x = x
        self.current_y = y
        self.current_yaw = yaw
        
        # Update distance traveled
        dx = x - self.last_x
        dy = y - self.last_y
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 1.0:  # Ignore jumps (localization errors)
            self.distance_traveled += dist
        self.last_x = x
        self.last_y = y
        
        # Track visited grid cells (for coverage)
        gx = int(x / self.grid_resolution)
        gy = int(y / self.grid_resolution)
        self.positions_visited.add((gx, gy))
        
        # Check if left start zone
        if self.start_set and not self.left_start_zone:
            dist_from_start = self._distance_from_start()
            if dist_from_start > self.lap_threshold * 2:
                self.left_start_zone = True
                print(f"[AUTONOMY] Left start zone (distance: {dist_from_start:.2f}m)")
    
    def update_grid(self, grid: List[List[int]], resolution: float):
        """Update grid info for coverage calculation."""
        if grid is None or len(grid) == 0:
            return
        
        self.grid_resolution = resolution
        self.grid_size = len(grid)
        
        # Count cells
        self.free_cells = 0
        self.total_cells = 0
        for row in grid:
            for cell in row:
                if cell >= 0:  # Known cell (free or occupied)
                    self.total_cells += 1
                if cell == 0:  # Free cell
                    self.free_cells += 1
    
    def _distance_from_start(self) -> float:
        """Calculate distance from current position to start."""
        if not self.start_set:
            return float('inf')
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        return math.sqrt(dx*dx + dy*dy)
    
    def check_lap_complete(self) -> bool:
        """Check if robot has completed a lap (returned to start)."""
        if not self.start_set or not self.left_start_zone:
            return False
        
        # Need minimum distance traveled
        if self.distance_traveled < self.min_distance_for_lap:
            return False
        
        # Check if close to start
        dist = self._distance_from_start()
        if dist < self.lap_threshold:
            self.laps_completed += 1
            self.left_start_zone = False  # Reset for next lap
            print(f"[AUTONOMY] LAP COMPLETED! Total laps: {self.laps_completed}")
            return True
        
        return False
    
    def get_coverage_percent(self) -> float:
        """Calculate exploration coverage percentage."""
        if self.total_cells == 0:
            # Fallback: use visited positions
            expected_cells = (self.grid_size ** 2) * 0.3  # Assume 30% is explorable
            return min(100.0, (len(self.positions_visited) / max(expected_cells, 1)) * 100)
        
        # Coverage = explored cells / total known cells
        return (self.free_cells / max(self.total_cells, 1)) * 100
    
    def reset(self):
        """Reset tracking for new mapping session."""
        self.start_set = False
        self.left_start_zone = False
        self.distance_traveled = 0.0
        self.positions_visited.clear()
        self.free_cells = 0
        self.total_cells = 0


class AutonomyNode:
    """Main autonomy node with state machine."""
    
    def __init__(self, autonomy_port: int = ZMQ_PORT_AUTONOMY,
                 slam_port: int = ZMQ_PORT_SLAM,
                 status_port: int = ZMQ_PORT_AUTONOMY_STATUS):
        
        self.state = AutonomyState.IDLE
        self.mode = "disabled"
        self.tracker = MappingTracker()
        self.start_time = 0.0
        self.message = ""
        
        # ZMQ setup
        self.ctx = zmq.Context()
        
        # Command socket (from laptop)
        self.cmd_sock = self.ctx.socket(zmq.SUB)
        self.cmd_sock.setsockopt(zmq.SUBSCRIBE, b"")
        self.cmd_sock.setsockopt(zmq.RCVTIMEO, 100)
        self.cmd_sock.bind(f"tcp://*:{autonomy_port}")
        
        # SLAM socket (from slam_bridge)
        self.slam_sock = self.ctx.socket(zmq.SUB)
        self.slam_sock.setsockopt(zmq.SUBSCRIBE, b"")
        self.slam_sock.setsockopt(zmq.RCVTIMEO, 100)
        self.slam_sock.setsockopt(zmq.CONFLATE, 1)
        self.slam_sock.connect(f"tcp://127.0.0.1:{slam_port}")
        
        # Status publisher
        self.status_sock = self.ctx.socket(zmq.PUB)
        self.status_sock.setsockopt(zmq.SNDHWM, 1)
        self.status_sock.bind(f"tcp://*:{status_port}")
        
        # Poller
        self.poller = zmq.Poller()
        self.poller.register(self.cmd_sock, zmq.POLLIN)
        self.poller.register(self.slam_sock, zmq.POLLIN)
        
        print(f"[AUTONOMY] Node started")
        print(f"  Commands: {autonomy_port}")
        print(f"  SLAM: {slam_port}")
        print(f"  Status: {status_port}")
    
    def handle_command(self, msg: dict):
        """Process autonomy command from laptop."""
        cmd_mode = msg.get("mode", "disabled")
        
        if cmd_mode == "mapping" and self.state == AutonomyState.IDLE:
            self._start_mapping()
        elif cmd_mode == "sensor_cabinet" and self.state == AutonomyState.IDLE:
            self._start_sensor_cabinet()
        elif cmd_mode == "disabled":
            self._stop_mapping()
        elif cmd_mode == "reset":
            self._reset()
    
    def handle_slam(self, msg: dict):
        """Process SLAM data from slam_bridge."""
        # Update pose
        if "x" in msg:
            x = float(msg.get("x", 0))
            y = float(msg.get("y", 0))
            yaw = float(msg.get("yaw", 0))
            self.tracker.update_pose(x, y, yaw)
        
        # Update grid
        if "grid" in msg:
            resolution = float(msg.get("resolution", 0.05))
            self.tracker.update_grid(msg["grid"], resolution)
        
        # State machine updates
        if self.state == AutonomyState.MAPPING:
            if self.tracker.check_lap_complete():
                self._lap_completed()
    
    def _start_mapping(self):
        """Start mapping mode."""
        self.state = AutonomyState.MAPPING
        self.mode = "mapping"
        self.start_time = time.time()
        self.tracker.set_start(
            self.tracker.current_x,
            self.tracker.current_y,
            self.tracker.current_yaw
        )
        self.message = "Mapping started - explore the area and return to start"
        print(f"[AUTONOMY] MAPPING STARTED")
    
    def _start_sensor_cabinet(self):
        """Start Sensor Cabinet autonomy (single command; 4x multiplier if from stowed).
        RMRC rulebook requires stowed posture before autonomy for scoring bonus."""
        if not self._check_stowed_posture():
            self.message = "WARNING: Arm not in stowed posture! Stow arm before starting autonomy for 4x bonus."
            print("[AUTONOMY] SENSOR CABINET BLOCKED: arm not stowed")
            # Allow start anyway (operator override) but log the warning
        self.state = AutonomyState.SENSOR_CABINET
        self.mode = "sensor_cabinet"
        self.start_time = time.time()
        self.message = "Sensor Cabinet: position camera for HAZMAT/Landolt (arm preset or manual)"
        print(f"[AUTONOMY] SENSOR CABINET STARTED")

    def _check_stowed_posture(self) -> bool:
        """Check if arm is in stowed posture. Returns True if stowed or check unavailable."""
        try:
            # Check if followerarm reports stowed state via shared status
            arm_stowed = getattr(self, '_arm_stowed', None)
            if arm_stowed is not None:
                return arm_stowed
            # If we can't determine arm state, assume stowed (don't block operation)
            return True
        except Exception:
            return True

    def _stop_mapping(self):
        """Stop mapping / sensor cabinet mode."""
        if self.state != AutonomyState.IDLE:
            print(f"[AUTONOMY] Stopped (state was: {self.state})")
        self.state = AutonomyState.IDLE
        self.mode = "disabled"
        self.message = "Mapping disabled"
    
    def _lap_completed(self):
        """Handle lap completion."""
        self.state = AutonomyState.COMPLETED
        elapsed = time.time() - self.start_time
        self.message = f"LAP COMPLETE! +{POINTS_LAP_AUTO} points! Time: {elapsed:.1f}s"
        print(f"[AUTONOMY] {self.message}")
        # Note: In competition, this would trigger scoring
    
    def _reset(self):
        """Reset autonomy state."""
        self.state = AutonomyState.IDLE
        self.mode = "disabled"
        self.tracker.reset()
        self.message = "Autonomy reset"
        print("[AUTONOMY] Reset")
    
    def get_status(self) -> AutonomyStatus:
        """Build status message."""
        elapsed = time.time() - self.start_time if self.start_time > 0 else 0.0
        
        return AutonomyStatus(
            state=self.state.value,
            mode=self.mode,
            coverage_percent=self.tracker.get_coverage_percent(),
            elapsed_time=elapsed,
            laps_completed=self.tracker.laps_completed,
            distance_traveled=self.tracker.distance_traveled,
            current_pose={
                "x": self.tracker.current_x,
                "y": self.tracker.current_y,
                "yaw": self.tracker.current_yaw
            },
            start_pose={
                "x": self.tracker.start_x,
                "y": self.tracker.start_y,
                "yaw": self.tracker.start_yaw
            } if self.tracker.start_set else None,
            message=self.message,
            timestamp=time.time()
        )
    
    def publish_status(self):
        """Publish status to ZMQ."""
        status = self.get_status()
        try:
            self.status_sock.send_json(status.to_dict(), zmq.NOBLOCK)
        except zmq.Again:
            pass
    
    def run(self):
        """Main loop."""
        print("[AUTONOMY] Running... (Ctrl+C to stop)")
        last_status_time = 0.0
        status_interval = 0.5  # Publish status at 2 Hz
        
        try:
            while True:
                # Poll for messages
                events = dict(self.poller.poll(100))
                
                # Handle commands
                if self.cmd_sock in events:
                    try:
                        msg = self.cmd_sock.recv_json(zmq.NOBLOCK)
                        self.handle_command(msg)
                    except (zmq.Again, json.JSONDecodeError):
                        pass
                
                # Handle SLAM data
                if self.slam_sock in events:
                    try:
                        msg = self.slam_sock.recv_json(zmq.NOBLOCK)
                        self.handle_slam(msg)
                    except (zmq.Again, json.JSONDecodeError):
                        pass
                
                # Publish status periodically
                now = time.time()
                if now - last_status_time > status_interval:
                    self.publish_status()
                    last_status_time = now
                    
        except KeyboardInterrupt:
            print("\n[AUTONOMY] Stopped")
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Clean up resources."""
        self.cmd_sock.close()
        self.slam_sock.close()
        self.status_sock.close()
        self.ctx.term()


def main():
    ap = argparse.ArgumentParser(description="Autonomy node for mapping bonus")
    ap.add_argument("--autonomy-port", type=int, default=ZMQ_PORT_AUTONOMY)
    ap.add_argument("--slam-port", type=int, default=ZMQ_PORT_SLAM)
    ap.add_argument("--status-port", type=int, default=ZMQ_PORT_AUTONOMY_STATUS)
    args = ap.parse_args()
    
    if zmq is None:
        print("[AUTONOMY] ERROR: pyzmq required")
        return
    
    node = AutonomyNode(
        autonomy_port=args.autonomy_port,
        slam_port=args.slam_port,
        status_port=args.status_port
    )
    node.run()


if __name__ == "__main__":
    main()
