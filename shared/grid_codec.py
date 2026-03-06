"""
RoboCupRescue RMRC 2026 - Grid Codec (RLE + base64)
=====================================================
Encodes 200×200 int8 occupancy grid from ~400KB JSON to ~2-5KB base64.
Used by slam_bridge.py (encoder) and slam_view.py (decoder).
"""

import base64
import struct
from typing import List, Tuple

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False


def encode_grid(grid: "np.ndarray") -> str:
    """RLE + base64 encode an int8 occupancy grid.
    
    Grid values: -1=unknown, 0=free, 100=occupied.
    RLE format: pairs of (value_byte, count_uint16) packed big-endian.
    Returns base64 string suitable for JSON.
    """
    flat = grid.flatten().astype(np.int8)
    runs = []
    i = 0
    n = len(flat)
    while i < n:
        val = flat[i]
        count = 1
        while i + count < n and flat[i + count] == val and count < 65535:
            count += 1
        # Pack: signed byte for value, unsigned short for count
        runs.append(struct.pack(">bH", int(val), count))
        i += count
    raw = b"".join(runs)
    return base64.b64encode(raw).decode("ascii")


def decode_grid(encoded: str, size: int) -> "np.ndarray":
    """Decode RLE + base64 string back to int8 grid of shape (size, size).
    
    Returns np.ndarray of shape (size, size) with dtype int8.
    """
    raw = base64.b64decode(encoded)
    flat = []
    i = 0
    while i + 2 < len(raw):
        val = struct.unpack(">b", raw[i:i+1])[0]
        count = struct.unpack(">H", raw[i+1:i+3])[0]
        flat.extend([val] * count)
        i += 3
    arr = np.array(flat[:size*size], dtype=np.int8)
    if len(arr) < size * size:
        arr = np.pad(arr, (0, size * size - len(arr)), constant_values=-1)
    return arr.reshape(size, size)


def encode_cloud(points: "np.ndarray") -> str:
    """Compress point cloud (N×3 float32) to base64 string."""
    if points is None or len(points) == 0:
        return ""
    arr = np.asarray(points, dtype=np.float32)
    return base64.b64encode(arr.tobytes()).decode("ascii")


def decode_cloud(encoded: str) -> "np.ndarray":
    """Decode base64 point cloud back to N×3 float32 array."""
    if not encoded:
        return np.empty((0, 3), dtype=np.float32)
    raw = base64.b64decode(encoded)
    arr = np.frombuffer(raw, dtype=np.float32)
    return arr.reshape(-1, 3)
