#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Status Monitor Node
==============================================
Monitors and publishes robot system status.

Features:
- CPU/GPU temperature monitoring
- Memory and disk usage
- Battery status (if available)
- Connection status for all devices
- Periodic status publishing via ZMQ
"""

import json
import time
import threading
import subprocess
import os
import sys
from typing import Dict, Optional, List
from dataclasses import dataclass, asdict

try:
    import zmq
    ZMQ_AVAILABLE = True
except ImportError:
    ZMQ_AVAILABLE = False

try:
    import psutil
    PSUTIL_AVAILABLE = True
except ImportError:
    PSUTIL_AVAILABLE = False
    print("[STATUS] WARNING: psutil not available, limited monitoring")

# Add shared module to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', 'shared'))

try:
    from shared.constants import ZMQ_PORT_STATUS, STATUS_UPDATE_HZ
except ImportError:
    ZMQ_PORT_STATUS = 5559
    STATUS_UPDATE_HZ = 10


@dataclass
class SystemStatus:
    """System resource status."""
    cpu_percent: float = 0
    cpu_temp: float = 0
    gpu_temp: float = 0
    memory_percent: float = 0
    memory_used_mb: float = 0
    memory_total_mb: float = 0
    disk_percent: float = 0
    disk_used_gb: float = 0
    disk_total_gb: float = 0


@dataclass
class DeviceStatus:
    """Connected device status."""
    pico_connected: bool = False
    arm_connected: bool = False
    lidar_connected: bool = False
    cameras_connected: List[str] = None


@dataclass
class RobotStatus:
    """Complete robot status."""
    timestamp: float = 0
    uptime: float = 0
    system: SystemStatus = None
    devices: DeviceStatus = None
    battery_voltage: float = 0
    battery_percent: float = 0
    ethernet_connected: bool = False
    wifi_signal: int = 0


class StatusMonitor:
    """
    Monitors robot status and publishes via ZMQ.
    """
    
    def __init__(self):
        # ZMQ setup
        self.zmq_context: Optional[zmq.Context] = None
        self.pub_socket: Optional[zmq.Socket] = None
        
        # State
        self.running = False
        self.start_time = 0
        
        # Monitoring thread
        self.monitor_thread: Optional[threading.Thread] = None
        
        # Cached status
        self.current_status: Optional[RobotStatus] = None
    
    def start(self):
        """Start status monitoring."""
        print("[STATUS] Starting status monitor...")
        
        if ZMQ_AVAILABLE:
            self.zmq_context = zmq.Context()
            self.pub_socket = self.zmq_context.socket(zmq.PUB)
            
            try:
                bind_addr = f"tcp://*:{ZMQ_PORT_STATUS}"
                self.pub_socket.bind(bind_addr)
                print(f"[STATUS] Publishing on {bind_addr}")
            except zmq.ZMQError as e:
                print(f"[STATUS] ZMQ bind error: {e}")
        
        self.running = True
        self.start_time = time.time()
        
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        
        print("[STATUS] Status monitor started")
    
    def stop(self):
        """Stop status monitoring."""
        print("[STATUS] Stopping...")
        self.running = False
        
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)
        
        if self.pub_socket:
            self.pub_socket.close()
        
        if self.zmq_context:
            self.zmq_context.term()
        
        print("[STATUS] Status monitor stopped")
    
    def _monitor_loop(self):
        """Main monitoring loop."""
        update_period = 1.0 / STATUS_UPDATE_HZ
        
        while self.running:
            loop_start = time.time()
            
            # Collect status
            status = self._collect_status()
            self.current_status = status
            
            # Publish
            self._publish_status(status)
            
            # Maintain loop rate
            elapsed = time.time() - loop_start
            if elapsed < update_period:
                time.sleep(update_period - elapsed)
    
    def _collect_status(self) -> RobotStatus:
        """Collect all status information."""
        return RobotStatus(
            timestamp=time.time(),
            uptime=time.time() - self.start_time,
            system=self._get_system_status(),
            devices=self._get_device_status(),
            battery_voltage=self._get_battery_voltage(),
            battery_percent=self._get_battery_percent(),
            ethernet_connected=self._check_ethernet(),
            wifi_signal=self._get_wifi_signal()
        )
    
    def _get_system_status(self) -> SystemStatus:
        """Get system resource status."""
        status = SystemStatus()
        
        if PSUTIL_AVAILABLE:
            # CPU
            status.cpu_percent = psutil.cpu_percent(interval=None)
            
            # Memory
            mem = psutil.virtual_memory()
            status.memory_percent = mem.percent
            status.memory_used_mb = mem.used / (1024 * 1024)
            status.memory_total_mb = mem.total / (1024 * 1024)
            
            # Disk
            disk = psutil.disk_usage('/')
            status.disk_percent = disk.percent
            status.disk_used_gb = disk.used / (1024 * 1024 * 1024)
            status.disk_total_gb = disk.total / (1024 * 1024 * 1024)
        
        # Temperature (Jetson-specific)
        status.cpu_temp = self._get_jetson_temp('cpu')
        status.gpu_temp = self._get_jetson_temp('gpu')
        
        return status
    
    def _get_jetson_temp(self, sensor: str) -> float:
        """Get Jetson temperature from thermal zones."""
        try:
            # Try Jetson thermal zones
            thermal_paths = {
                'cpu': [
                    '/sys/devices/virtual/thermal/thermal_zone0/temp',
                    '/sys/class/thermal/thermal_zone0/temp'
                ],
                'gpu': [
                    '/sys/devices/virtual/thermal/thermal_zone1/temp',
                    '/sys/class/thermal/thermal_zone1/temp'
                ]
            }
            
            for path in thermal_paths.get(sensor, []):
                if os.path.exists(path):
                    with open(path, 'r') as f:
                        temp = int(f.read().strip()) / 1000.0
                        return temp
            
            return 0.0
            
        except Exception:
            return 0.0
    
    def _get_device_status(self) -> DeviceStatus:
        """Check connected devices."""
        status = DeviceStatus(cameras_connected=[])
        
        # Check Pico
        status.pico_connected = os.path.exists('/dev/pico') or os.path.exists('/dev/ttyACM0')
        
        # Check arm servo
        status.arm_connected = os.path.exists('/dev/arm_servo') or os.path.exists('/dev/ttyUSB0')
        
        # Check lidar
        status.lidar_connected = os.path.exists('/dev/lidar') or os.path.exists('/dev/ttyUSB1')
        
        # Check cameras
        for i in range(10):
            if os.path.exists(f'/dev/video{i}'):
                status.cameras_connected.append(f'video{i}')
        
        return status
    
    def _get_battery_voltage(self) -> float:
        """Get battery voltage (if INA219 or similar is connected)."""
        # Placeholder - implement based on actual battery monitoring hardware
        return 0.0
    
    def _get_battery_percent(self) -> float:
        """Get battery percentage."""
        # Placeholder - implement based on actual battery monitoring
        return 0.0
    
    def _check_ethernet(self) -> bool:
        """Check if ethernet is connected."""
        try:
            if PSUTIL_AVAILABLE:
                stats = psutil.net_if_stats()
                return stats.get('eth0', stats.get('enp1s0', None)) is not None
            return False
        except Exception:
            return False
    
    def _get_wifi_signal(self) -> int:
        """Get WiFi signal strength (0-100)."""
        try:
            result = subprocess.run(
                ['iwconfig', 'wlan0'],
                capture_output=True, text=True, timeout=1
            )
            if 'Signal level' in result.stdout:
                # Parse signal level
                for part in result.stdout.split():
                    if 'level=' in part:
                        level = int(part.split('=')[1])
                        # Convert dBm to percentage (approximate)
                        return max(0, min(100, (level + 100) * 2))
            return 0
        except Exception:
            return 0
    
    def _publish_status(self, status: RobotStatus):
        """Publish status via ZMQ."""
        if not self.pub_socket:
            return
        
        msg = {
            "msg_type": "status",
            "timestamp": status.timestamp,
            "uptime": status.uptime,
            "cpu_percent": status.system.cpu_percent if status.system else 0,
            "cpu_temp": status.system.cpu_temp if status.system else 0,
            "gpu_temp": status.system.gpu_temp if status.system else 0,
            "memory_percent": status.system.memory_percent if status.system else 0,
            "disk_percent": status.system.disk_percent if status.system else 0,
            "pico_connected": status.devices.pico_connected if status.devices else False,
            "arm_connected": status.devices.arm_connected if status.devices else False,
            "lidar_connected": status.devices.lidar_connected if status.devices else False,
            "cameras_connected": status.devices.cameras_connected if status.devices else [],
            "battery_voltage": status.battery_voltage,
            "battery_percent": status.battery_percent,
            "ethernet_connected": status.ethernet_connected,
            "wifi_signal": status.wifi_signal
        }
        
        try:
            self.pub_socket.send_json(msg, zmq.NOBLOCK)
        except zmq.Again:
            pass
        except Exception as e:
            print(f"[STATUS] Publish error: {e}")
    
    def get_status(self) -> Optional[RobotStatus]:
        """Get current cached status."""
        return self.current_status


def main():
    """Main entry point."""
    monitor = StatusMonitor()
    
    try:
        monitor.start()
        
        # Keep running and print status periodically
        while True:
            time.sleep(5)
            status = monitor.get_status()
            if status and status.system:
                print(f"[STATUS] CPU: {status.system.cpu_percent:.1f}% @ {status.system.cpu_temp:.1f}°C, "
                      f"MEM: {status.system.memory_percent:.1f}%, "
                      f"Pico: {'✓' if status.devices.pico_connected else '✗'}")
            
    except KeyboardInterrupt:
        print("\n[STATUS] Interrupted")
    finally:
        monitor.stop()


if __name__ == "__main__":
    main()
