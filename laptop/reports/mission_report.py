#!/usr/bin/env python3
"""
RoboCupRescue RMRC 2026 - Mission Report Generator
==================================================
Generates PDF report after missions: laps, QR counts, detections, points.
Uses reportlab for PDF output.
"""

import os
import sys
from datetime import datetime
from typing import Dict, List, Any, Optional

# Add parent for shared
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

try:
    from shared.constants import (
        MISSION_DURATION,
        POINTS_LAP_TELEOP, POINTS_LAP_AUTO,
        POINTS_QR_READ, POINTS_QR_MAPPED,
        POINTS_HAZMAT, POINTS_LANDOLT, POINTS_OBJECT_REMOVED, POINTS_OBJECT_IN_CONTAINER,
        POINTS_KEYPAD_5_CLEAN, POINTS_KEYPAD_5_DIRTY,
        AUTONOMY_MULTIPLIER,
    )
except ImportError:
    MISSION_DURATION = 300
    POINTS_LAP_TELEOP, POINTS_LAP_AUTO = 1, 4
    POINTS_QR_READ, POINTS_QR_MAPPED = 1, 4
    POINTS_HAZMAT, POINTS_LANDOLT = 1, 1
    POINTS_OBJECT_REMOVED, POINTS_OBJECT_IN_CONTAINER = 1, 2
    POINTS_KEYPAD_5_CLEAN, POINTS_KEYPAD_5_DIRTY = 3, 1
    AUTONOMY_MULTIPLIER = 4

try:
    from reportlab.lib import colors
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
    from reportlab.lib.units import cm
    from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak
    from reportlab.lib.enums import TA_CENTER, TA_LEFT
    REPORTLAB_AVAILABLE = True
except ImportError:
    REPORTLAB_AVAILABLE = False


def _default_mission_data() -> Dict[str, Any]:
    return {
        "mission_id": datetime.now().strftime("%Y%m%d_%H%M%S"),
        "start_time": datetime.now().isoformat(),
        "duration_sec": 0,
        "test_type": "",
        "laps_teleop": 0,
        "laps_auto": 0,
        "qr_read_count": 0,
        "qr_mapped_count": 0,
        "hazmat_count": 0,
        "landolt_count": 0,
        "objects_removed": 0,
        "objects_in_container": 0,
        "keypad_clean": 0,
        "keypad_dirty": 0,
        "stairs_teleop": 0,
        "stairs_auto": 0,
        "align_teleop": 0,
        "align_auto": 0,
        "drop_15": 0,
        "drop_30": 0,
        "inspect_readings": [],
        "detection_count": 0,
        "notes": "",
        "qr_entries": [],
    }


def _compute_points_by_test_type(test_type: str, data: Dict[str, Any]) -> tuple:
    """Per-test scoring: returns (teleop_pts, auto_pts). Auto pts at final value (4× already baked in); total = teleop + auto."""
    teleop = 0.0
    auto = 0.0
    # Mobility (laps)
    if test_type in ("incline_horiz", "incline_inclined", "sand_gravel", "ramps_continuous", "ramps_pinwheel",
                     "elevated_ramps", "krails_horiz", "krails_crossover", "hurdles_single", "hurdles_double"):
        teleop = data.get("laps_teleop", 0) * POINTS_LAP_TELEOP
        auto = data.get("laps_auto", 0) * POINTS_LAP_AUTO
    # Stairs
    elif test_type == "stairs":
        teleop = data.get("stairs_teleop", 0) * POINTS_LAP_TELEOP
        auto = data.get("stairs_auto", 0) * POINTS_LAP_AUTO
    # Align
    elif test_type == "align":
        teleop = data.get("align_teleop", 0) * POINTS_LAP_TELEOP
        auto = data.get("align_auto", 0) * POINTS_LAP_AUTO
    # Keypad
    elif test_type == "keypad_omni":
        teleop = data.get("keypad_clean", 0) * POINTS_KEYPAD_5_CLEAN + data.get("keypad_dirty", 0) * POINTS_KEYPAD_5_DIRTY
        auto = teleop * (AUTONOMY_MULTIPLIER - 1)  # auto bonus at final value
    # Sensor Cabinet
    elif test_type == "sensor_cabinet":
        teleop = (data.get("hazmat_count", 0) * POINTS_HAZMAT + data.get("landolt_count", 0) * POINTS_LANDOLT)
        auto = teleop * (AUTONOMY_MULTIPLIER - 1)  # auto bonus at final value
    # Linear Rail Inspect (per Landolt × 4)
    elif test_type == "linear_rail_inspect":
        readings = data.get("inspect_readings", [])
        auto = len(readings) * POINTS_LANDOLT * AUTONOMY_MULTIPLIER
    # Linear Rail Pick: removed(1) + container(2) × 4
    elif test_type == "linear_rail_pick":
        teleop = data.get("objects_removed", 0) * POINTS_OBJECT_REMOVED
        auto = data.get("objects_in_container", 0) * POINTS_OBJECT_IN_CONTAINER
    # Labyrinth: QR + QR mapped
    elif test_type in ("labyrinth_flat", "labyrinth_krails"):
        teleop = data.get("qr_read_count", 0) * POINTS_QR_READ
        auto = data.get("qr_mapped_count", 0) * POINTS_QR_MAPPED
    # Drop Test
    elif test_type == "drop_test":
        teleop = data.get("drop_15", 0) * 1 + data.get("drop_30", 0) * 2
    else:
        teleop = (
            data.get("laps_teleop", 0) * POINTS_LAP_TELEOP
            + data.get("qr_read_count", 0) * POINTS_QR_READ
            + data.get("hazmat_count", 0) * POINTS_HAZMAT
            + data.get("landolt_count", 0) * POINTS_LANDOLT
            + data.get("objects_removed", 0) * POINTS_OBJECT_REMOVED
        )
        auto = (
            data.get("laps_auto", 0) * POINTS_LAP_AUTO
            + data.get("qr_mapped_count", 0) * POINTS_QR_MAPPED
            + data.get("objects_in_container", 0) * POINTS_OBJECT_IN_CONTAINER
        )
    return teleop, auto


def _compute_points(data: Dict[str, Any]) -> tuple:
    """Compute teleop and autonomy points. Returns (teleop_total, auto_total, total).
    POINTS_LAP_AUTO/POINTS_QR_MAPPED already include the 4× autonomy multiplier,
    so total = teleop + auto (no extra multiplier).
    """
    teleop = (
        data.get("laps_teleop", 0) * POINTS_LAP_TELEOP
        + data.get("qr_read_count", 0) * POINTS_QR_READ
        + data.get("hazmat_count", 0) * POINTS_HAZMAT
        + data.get("landolt_count", 0) * POINTS_LANDOLT
        + data.get("objects_removed", 0) * POINTS_OBJECT_REMOVED
    )
    auto = (
        data.get("laps_auto", 0) * POINTS_LAP_AUTO
        + data.get("qr_mapped_count", 0) * POINTS_QR_MAPPED
        + data.get("objects_in_container", 0) * POINTS_OBJECT_IN_CONTAINER
    )
    total = teleop + auto
    return teleop, auto, total


def generate_mission_report(
    output_path: str = "mission_report.pdf",
    mission_data: Optional[Dict[str, Any]] = None,
    **kwargs
) -> bool:
    """
    Generate PDF mission report.
    mission_data can contain: laps_teleop, laps_auto, qr_read_count, qr_mapped_count,
    hazmat_count, landolt_count, objects_removed, objects_in_container, duration_sec, notes.
    Returns True on success.
    """
    if not REPORTLAB_AVAILABLE:
        print("[REPORT] reportlab not installed; pip install reportlab")
        return False

    data = _default_mission_data()
    if mission_data:
        data.update(mission_data)
    data.update(kwargs)

    try:
        doc = SimpleDocTemplate(
            output_path,
            pagesize=A4,
            rightMargin=1.5 * cm,
            leftMargin=1.5 * cm,
            topMargin=1.5 * cm,
            bottomMargin=1.5 * cm,
        )
        styles = getSampleStyleSheet()
        title_style = ParagraphStyle(
            "Title",
            parent=styles["Heading1"],
            fontSize=18,
            alignment=TA_CENTER,
            spaceAfter=12,
        )
        h2_style = ParagraphStyle(
            "H2",
            parent=styles["Heading2"],
            fontSize=14,
            spaceAfter=8,
        )

        story = []

        # Title
        story.append(Paragraph("RoboCupRescue RMRC 2026 – Mission Report", title_style))
        story.append(Spacer(1, 12))
        story.append(Paragraph(f"Mission ID: {data['mission_id']}", styles["Normal"]))
        story.append(Paragraph(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}", styles["Normal"]))
        story.append(Spacer(1, 20))

        # Summary table (per-test scoring when test_type set)
        test_type = data.get("test_type") or ""
        if test_type:
            teleop_pts, auto_pts = _compute_points_by_test_type(test_type, data)
            total_pts = teleop_pts + auto_pts
        else:
            teleop_pts, auto_pts, total_pts = _compute_points(data)
        summary_data = [
            ["Metric", "Value"],
            ["Test type", test_type or "—"],
            ["Duration (s)", str(data["duration_sec"])],
            ["Laps (teleop)", str(data["laps_teleop"])],
            ["Laps (autonomy)", str(data["laps_auto"])],
            ["QR read", str(data["qr_read_count"])],
            ["QR mapped", str(data["qr_mapped_count"])],
            ["Hazmat detections", str(data["hazmat_count"])],
            ["Landolt-C", str(data["landolt_count"])],
            ["Objects removed", str(data["objects_removed"])],
            ["Objects in container", str(data["objects_in_container"])],
            ["Total detections (log)", str(data["detection_count"])],
            ["Points (teleop)", str(teleop_pts)],
            ["Points (autonomy)", str(auto_pts)],
            ["Total points", str(total_pts)],
        ]
        t = Table(summary_data, colWidths=[5 * cm, 5 * cm])
        t.setStyle(TableStyle([
            ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#0f3460")),
            ("TEXTCOLOR", (0, 0), (-1, 0), colors.whitesmoke),
            ("ALIGN", (0, 0), (-1, -1), "LEFT"),
            ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
            ("FONTSIZE", (0, 0), (-1, 0), 10),
            ("BOTTOMPADDING", (0, 0), (-1, 0), 8),
            ("BACKGROUND", (0, 1), (-1, -1), colors.HexColor("#1a1a2e")),
            ("TEXTCOLOR", (0, 1), (-1, -1), colors.HexColor("#e0e0e0")),
            ("GRID", (0, 0), (-1, -1), 0.5, colors.HexColor("#0f3460")),
            ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.HexColor("#1a1a2e"), colors.HexColor("#16213e")]),
        ]))
        story.append(Paragraph("Summary", h2_style))
        story.append(t)
        story.append(Spacer(1, 16))

        # Linear Rail Inspect readings (tube + direction)
        inspect_readings = data.get("inspect_readings") or []
        if inspect_readings:
            story.append(Paragraph("Linear Rail Inspect – Recorded readings", h2_style))
            in_rows = [["#", "Tube", "Direction"]]
            for i, r in enumerate(inspect_readings, 1):
                row = r if isinstance(r, dict) else {}
                in_rows.append([str(i), str(row.get("tube", "—")), str(row.get("direction", "—"))])
            in_table = Table(in_rows, colWidths=[1.5 * cm, 2 * cm, 4 * cm])
            in_table.setStyle(TableStyle([
                ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#0f3460")),
                ("TEXTCOLOR", (0, 0), (-1, 0), colors.whitesmoke),
                ("FONTSIZE", (0, 0), (-1, -1), 9),
                ("GRID", (0, 0), (-1, -1), 0.5, colors.HexColor("#0f3460")),
            ]))
            story.append(in_table)
            story.append(Spacer(1, 12))

        # Scanned QR codes (full content)
        qr_entries = data.get("qr_entries") or []
        if qr_entries:
            story.append(Paragraph("Scanned QR codes", h2_style))
            qr_table_data = [["#", "Time", "Camera", "Content"]]
            for i, entry in enumerate(qr_entries, 1):
                row = entry if isinstance(entry, dict) else {}
                ts = row.get("timestamp", "")
                cam = row.get("camera_id", "")
                raw = row.get("data", str(entry) if not isinstance(entry, dict) else "")
                # Escape for ReportLab and allow wrapping; full content in PDF
                content_para = Paragraph(
                    raw.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace("\n", "<br/>"),
                    ParagraphStyle("QRContent", parent=styles["Normal"], fontSize=9, wordWrap="CJK"),
                )
                qr_table_data.append([str(i), ts, cam, content_para])
            qr_table = Table(qr_table_data, colWidths=[1.2 * cm, 2 * cm, 2.2 * cm, 12 * cm])
            qr_table.setStyle(TableStyle([
                ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#0f3460")),
                ("TEXTCOLOR", (0, 0), (-1, 0), colors.whitesmoke),
                ("ALIGN", (0, 0), (-1, -1), "LEFT"),
                ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
                ("FONTSIZE", (0, 0), (-1, 0), 10),
                ("FONTSIZE", (0, 1), (-1, -1), 9),
                ("BOTTOMPADDING", (0, 0), (-1, 0), 8),
                ("BACKGROUND", (0, 1), (-1, -1), colors.HexColor("#1a1a2e")),
                ("TEXTCOLOR", (0, 1), (-1, -1), colors.HexColor("#e0e0e0")),
                ("GRID", (0, 0), (-1, -1), 0.5, colors.HexColor("#0f3460")),
                ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.HexColor("#1a1a2e"), colors.HexColor("#16213e")]),
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
            ]))
            story.append(qr_table)
            story.append(Spacer(1, 16))

        # QR-on-map: positions and optional 30 cm validation note (rulebook)
        qr_mapped = data.get("qr_mapped_count", 0)
        if qr_mapped > 0:
            story.append(Paragraph("QR-on-map (4 pts each if within 30 cm of two closest fiducials)", h2_style))
            story.append(Paragraph(
                f"Count: {qr_mapped}. Positions (robot pose when read) and fiducial distances below for judge verification.",
                styles["Normal"],
            ))
            try:
                from shared.constants import FIDUCIAL_POSITIONS
                fiducials = FIDUCIAL_POSITIONS
            except ImportError:
                fiducials = []
            qr_with_pos = [e for e in qr_entries if isinstance(e, dict) and "map_x" in e and "map_y" in e]
            if qr_with_pos:
                map_rows = [["#", "QR (short)", "map_x (m)", "map_y (m)", "d1 (m)", "d2 (m)"]]
                for i, row in enumerate(qr_with_pos, 1):
                    mx = row.get("map_x", 0)
                    my = row.get("map_y", 0)
                    d1 = d2 = ""
                    if fiducials and len(fiducials) >= 2:
                        dists = sorted(((mx - fx) ** 2 + (my - fy) ** 2) ** 0.5 for fx, fy in fiducials)
                        d1 = f"{dists[0]:.2f}" if dists else ""
                        d2 = f"{dists[1]:.2f}" if len(dists) > 1 else ""
                    map_rows.append([str(i), (row.get("data") or "")[:12], f"{mx:.2f}", f"{my:.2f}", d1, d2])
                map_table = Table(map_rows, colWidths=[1 * cm, 3 * cm, 2 * cm, 2 * cm, 1.5 * cm, 1.5 * cm])
                map_table.setStyle(TableStyle([
                    ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#0f3460")),
                    ("TEXTCOLOR", (0, 0), (-1, 0), colors.whitesmoke),
                    ("FONTSIZE", (0, 0), (-1, -1), 8),
                    ("GRID", (0, 0), (-1, -1), 0.5, colors.HexColor("#0f3460")),
                ]))
                story.append(map_table)
            story.append(Spacer(1, 16))

        if data.get("notes"):
            story.append(Paragraph("Notes", h2_style))
            story.append(Paragraph(data["notes"], styles["Normal"]))

        doc.build(story)

        # Write QR codes to companion .txt file (rulebook: given to judge)
        qr_entries = data.get("qr_entries") or []
        if qr_entries:
            txt_path = output_path.rsplit(".", 1)[0] + "_qr_codes.txt"
            try:
                with open(txt_path, "w", encoding="utf-8") as f:
                    f.write("RoboCupRescue RMRC 2026 - Scanned QR Codes\n")
                    f.write(f"Mission: {data.get('mission_id', '')} | Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                    f.write("-" * 60 + "\n")
                    for i, entry in enumerate(qr_entries, 1):
                        row = entry if isinstance(entry, dict) else {}
                        mx = row.get("map_x")
                        my = row.get("map_y")
                        pos = f"\t{mx:.3f}\t{my:.3f}" if mx is not None and my is not None else ""
                        f.write(f"{i}\t{row.get('timestamp', '')}\t{row.get('camera_id', '')}\t{row.get('data', '')}{pos}\n")
            except Exception as e:
                print(f"[REPORT] Could not write QR .txt: {e}")

        return True
    except Exception as e:
        print(f"[REPORT] Error: {e}")
        return False


def main():
    """CLI: generate a sample report."""
    import argparse
    ap = argparse.ArgumentParser(description="Generate mission report PDF")
    ap.add_argument("-o", "--output", default="mission_report.pdf", help="Output PDF path")
    ap.add_argument("--laps-teleop", type=int, default=0)
    ap.add_argument("--laps-auto", type=int, default=0)
    ap.add_argument("--qr-read", type=int, default=0)
    ap.add_argument("--qr-mapped", type=int, default=0)
    ap.add_argument("--duration", type=int, default=300)
    args = ap.parse_args()

    ok = generate_mission_report(
        args.output,
        mission_data={
            "laps_teleop": args.laps_teleop,
            "laps_auto": args.laps_auto,
            "qr_read_count": args.qr_read,
            "qr_mapped_count": args.qr_mapped,
            "duration_sec": args.duration,
        },
    )
    if ok:
        print(f"Report saved to {args.output}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
