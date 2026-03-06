# RMRC Competition Checklist

Short reference for operator and judge handover at competition.

---

## QR codes for judges (rulebook requirement)

**Rule:** *"The information from the QR codes must be written to a file and given to the judges at the end of the run."*

### Where the files are

1. **During the run (live log)**  
   - Path: `reports/mission_qr_codes.txt` (in the project repo on the laptop).  
   - Each new QR is appended when first read; the file is cleared when you click **Start Mission**.

2. **With the PDF report (after the run)**  
   - When you click **Report**, the GUI generates:
     - `~/mission_report.pdf` – mission summary (laps, QR count, HAZMAT/Landolt, points).
     - `~/mission_report_qr_codes.txt` – same QR list in plain text (next to the PDF).  
   - Give the judge **both** the PDF and the `mission_report_qr_codes.txt` (or the live `reports/mission_qr_codes.txt` if you did not generate a report).

### What to give the judge

- **Option A:** After the run, click **Report** and give the judge:
  - `~/mission_report.pdf`
  - `~/mission_report_qr_codes.txt` (same folder as the PDF)
- **Option B:** Give the judge the live log: `reports/mission_qr_codes.txt` from the project folder (e.g. `robocup_rescue_2026/reports/mission_qr_codes.txt`).

### Operator reminder

- During the run, when a new QR is read, the status bar shows: **"New QR read – point to judge and declare"**.  
- Rulebook: *"During the run, the operator shall also declare each time a QR code is read and point it out on the screen to the judge."*

---

## Mission timer

- **Start Mission** starts the 5-minute countdown (preliminaries).  
- The judge starts the official run timer when the team declares ready; the GUI timer is for operator awareness.

---

## Stowed posture and autonomy (4× multiplier)

- Check **Stowed** before using **Mapping** or **Sensor Cab** so autonomy bonus applies.  
- **Sensor Cab**: use the **Sensor Cab** button for autonomous sensing (HAZMAT/Landolt); Landolt directions appear on the camera overlay.

---

## QR-on-map (4 extra points per QR, Labyrinth)

- When a QR is read **and** SLAM pose is available, the QR is placed on the 2D map at the robot’s position (orange circle + short label).  
- **Report** includes **QR mapped** count and a table of map positions (map_x, map_y) and distances to the two closest fiducials (if `FIDUCIAL_POSITIONS` is set in `shared/constants.py`).  
- Rulebook: 4 extra points per QR if placement is accurate to within 30 cm of the two closest fiducials. Set `FIDUCIAL_POSITIONS = [(x1,y1), (x2,y2), ...]` (meters, world frame) for your arena to show fiducials on the map and distances in the report.

## Landolt-C directions (for judge)

- When Landolt-C is detected, direction and confidence are shown on the **camera image** (bottom line: e.g. `Landolt: Up 92%, Right 85%`).  
- The judge can use this to verify Sensor Cabinet / Linear Rail Inspect readings.

---

## Autonomy laps in the report

- If the robot completes autonomous laps (mapping), the GUI receives autonomy status from the Jetson and updates the autonomy lap count.  
- The **Report** PDF includes **Laps (autonomy)** so the judge can cross-check.

---

## Fiducial positions (QR-on-map validation)

- Set `FIDUCIAL_POSITIONS` in `shared/constants.py` from competition arena layout (meters, world frame).
- Or use **Set Fiducial** in the GUI (teleop): places current SLAM pose into the fiducial list (saved to `reports/fiducials.json`). Load that file at startup to set constants, or paste values into `constants.py`.
- Rulebook: 4 extra points per QR if placement is within 30 cm of two closest fiducials.

## Robot configuration photos

- Before competition, take: front, side, top, publicity, close-up (stowed + operating).
- Store in `docs/robot_photos/` for judge and documentation.

## Designated container (Linear Rail Pick)

- Bring a container ≤30×30 cm, ≤10 cm high for placed objects (rulebook).
- Pack it with the operator station gear.

---

## Quick file locations (laptop)

| Item | Location |
|------|----------|
| Live QR log | `reports/mission_qr_codes.txt` (in repo) |
| Mission report PDF | `~/mission_report.pdf` |
| QR list for judge (with report) | `~/mission_report_qr_codes.txt` |
| Fiducials (optional) | `reports/fiducials.json` |
