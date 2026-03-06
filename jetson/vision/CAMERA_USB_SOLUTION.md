# USB Camera Bandwidth Solution

## Problem
All 4 cameras are connected to the SAME USB hub (Bus 01, USB 2.0 @ 480 Mbps).
This causes only 2 cameras to work at a time due to bandwidth limitations.

## Root Cause
From `lsusb -t`:
- **Bus 01** (USB 2.0, 480 Mbps): ALL 4 CAMERAS HERE ❌
- **Bus 02** (USB 3.0, 10 Gbps): EMPTY ✓

All cameras share 480 Mbps bandwidth, which is NOT enough for 4 cameras.

## SOLUTION: Use Both USB Controllers

### Step 1: Physically Reconnect Cameras
You need to connect cameras to DIFFERENT physical USB ports that use Bus 02.

**Current Setup (all on Bus 01):**
- Port 1.2.4: Camera (video15)
- Port 1.2.3.3: Camera (video20) 
- Port 1.2.3.4: Camera (video16)
- Port 1.2.3.2: Camera (video19)

**Target Setup:**
- **Bus 01**: Keep 2 cameras (e.g., left + right)
- **Bus 02**: Move 2 cameras (e.g., front + arm)

### How to Find Bus 02 Ports:
1. Unplug all cameras
2. Plug one camera into each physical USB port on your Jetson
3. Run `lsusb -t` after each connection
4. Find which ports show up under "Bus 02"
5. Use those ports for front/arm cameras (they need more bandwidth)

### Step 2: Optimize Camera Settings
Once cameras are split across 2 buses:

**Front + Arm (Bus 02 - USB 3.0):**
- Use Full GPU pipeline
- 640x480@15fps
- ~3 Mbps each = 6 Mbps total

**Left + Right (Bus 01 - USB 2.0):**
- Use Light MJPEG  
- 320x240@25fps
- ~0.6 Mbps each = 1.2 Mbps total

**Total: ~7 Mbps** - easily fits within limits!

### Step 3: Test
```bash
# After reconnecting cameras, verify:
lsusb -t

# You should see 2 cameras under Bus 02 now
# Then run:
python vision_node.py
```

## Alternative Solutions (if above doesn't work)

### Option A: Powered USB 3.0 Hub
Buy a powered USB 3.0 hub with its own power supply.
This provides independent bandwidth for each device.

### Option B: Lower All Resolutions
If you must use current setup, reduce all cameras to:
- 320x240@10fps (~0.3 Mbps each)
- Total: 1.2 Mbps (might work)

### Option C: Use Only 3 Cameras
Accept the limitation and use only front, arm, and one side camera.

## Quick Test
To verify Bus 02 works, try:
```bash
# Connect ONE camera to a Bus 02 port
# Run this to test at high bandwidth:
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  image/jpeg,width=1920,height=1080,framerate=30/1 ! \
  fakesink
```

If this works without errors, Bus 02 can handle high bandwidth!

## Current Status
✓ USB autosuspend disabled
✓ Front/Arm using Full GPU pipeline (640x480@15fps)
✓ Left/Right using Light MJPEG (320x240@25fps)
❌ All cameras on same USB controller (Bus 01)

**NEXT STEP: Physically move 2 cameras to Bus 02 ports!**
