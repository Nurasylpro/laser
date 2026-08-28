"""
test_fixed_points.py — ground-truth accuracy test.

Imitates the real weed_zap.py flow's MOVEMENT (home, sweep both rows stop by
stop, same X-advance tiling) but never opens the camera or runs YOLO — the
scan phase is pure motion, purely to reproduce realistic timing/behavior.
Instead of firing on whatever detection would have found, it fires on a
hardcoded list of KNOWN real crown positions (measured by hand, already
relative to the laser beam — i.e. these are exactly what abs_x/abs_y/Z need
to be for the beam to land dead-on). This isolates motion + calibration
accuracy from detection accuracy entirely: if the beam lands correctly on
these known points, the aiming math is fine and any error in a real run is
coming from detection/pixel conversion, not the gantry/laser side.

Run:
    python scripts/test_fixed_points.py --port COM7 --home --seconds 2

Skip the simulated scan (just home, attach laser, visit the fixed points,
fire) if you don't need the movement realism:
    python scripts/test_fixed_points.py --port COM7 --home --seconds 2 --skip-tour
"""

import argparse
import sys
import time
from pathlib import Path

SCRIPT_DIR   = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(SCRIPT_DIR))

import path as path_mod
from calibration_loader import load_all_calibrations
from weed_zap import (
    open_serial, send_and_wait, get_gantry_position,
    compute_row_scan_advance_mm,
    SCAN_Z_MM, ROW_X_START_MM, ROW_X_END_MM, ROWS_Y_MM,
    ROW_SCAN_OVERLAP_FRACTION,
)
import photo as photo_mod

# ── Known real crown positions (relative to the laser beam) ──────
# Measured by hand, given directly as absolute gantry targets — the exact
# values the beam needs to move to so it lands on the real crown.
FIXED_POINTS = [
    {"abs_x": 370.00, "abs_y": 295.00, "z_mm": 120.00},
    {"abs_x": 460.00, "abs_y": 300.00, "z_mm": 120.00},
    {"abs_x": 600.00, "abs_y": 285.00, "z_mm": 120.00},
    {"abs_x": 600.00, "abs_y": 205.00, "z_mm": 120.00},
    {"abs_x": 685.00, "abs_y": 100.00, "z_mm": 120.00},
    {"abs_x": 426.00, "abs_y": 110.00, "z_mm": 120.00},
]


def simulate_tour(ser, dry_run: bool, calibrations: dict):
    """
    Move through the same rows/stops a real tour would, WITHOUT ever opening
    the camera or running detection — pure motion, for realistic timing.
    """
    advance_mm = compute_row_scan_advance_mm(
        SCAN_Z_MM, calibrations, photo_mod.CAPTURE_WIDTH, photo_mod.CAPTURE_HEIGHT,
        ROW_SCAN_OVERLAP_FRACTION)
    print(f"[SIM-TOUR] X-advance per stop: {advance_mm:.0f}mm "
          f"(rows span X={ROW_X_START_MM:.0f}-{ROW_X_END_MM:.0f}mm)\n")

    for row_index, row_y in enumerate(ROWS_Y_MM):
        row_num = row_index + 1
        print(f"[SIM-TOUR ROW {row_num}/{len(ROWS_Y_MM)}] Y={row_y:.1f}mm\n")

        stop_x, stop_index = ROW_X_END_MM, 0
        while stop_x > ROW_X_START_MM:
            stop_index += 1
            print(f"  [STOP {stop_index}] commanded X={stop_x:.1f}  Y={row_y:.1f}  Z={SCAN_Z_MM:.1f}")

            if dry_run:
                print(f"    (dry-run) Would send:  m {stop_x:.2f} {row_y:.2f} {SCAN_Z_MM:.2f}")
            else:
                cmd = f"m {stop_x:.2f} {row_y:.2f} {SCAN_Z_MM:.2f}"
                send_and_wait(ser, cmd, keyword="Move complete")
                real_x, real_y, real_z = get_gantry_position(ser)
                print(f"    Actual position: X={real_x:.1f}  Y={real_y:.1f}  Z={real_z:.1f}")

            print("    (no photo taken — motion-only simulation)")
            stop_x -= advance_mm

        print(f"[SIM-TOUR ROW {row_num}/{len(ROWS_Y_MM)}] Complete.\n")

    print("[SIM-TOUR] Complete.\n")


def visit_fixed_points(ser, dry_run: bool, laser_power: int, seconds: float,
                       confirm_each: bool):
    print(f"[FIXED-POINTS] Visiting {len(FIXED_POINTS)} known point(s), "
          f"laser power={laser_power}, {seconds:.1f}s each ...\n")

    for i, pt in enumerate(FIXED_POINTS, start=1):
        x, y, z = pt["abs_x"], pt["abs_y"], pt["z_mm"]
        print(f"  Point #{i}  ->  X={x:.2f} Y={y:.2f} Z={z:.2f}")

        if dry_run:
            print(f"  (dry-run) Would send:  m {x:.2f} {y:.2f} {z:.2f}")
            continue

        cmd = f"m {x:.2f} {y:.2f} {z:.2f}"
        send_and_wait(ser, cmd, keyword="Move complete")

        if confirm_each:
            input(f"  Point #{i} in position. Press Enter to fire ...")

        print(f"  Firing laser at power {laser_power} for {seconds:.1f}s ...")
        try:
            send_and_wait(ser, f"laser {laser_power}", keyword="Laser ->")
            time.sleep(seconds)
        finally:
            send_and_wait(ser, "laser 0", keyword="Laser ->")
        print(f"  Point #{i} done.\n")

    print("[FIXED-POINTS] All points visited.")


def run(port: str, baud: int, do_home: bool, dry_run: bool, skip_tour: bool,
        laser_power: int, seconds: float, confirm_each: bool, do_tool_put: bool):

    print(f"\n{'='*60}")
    print("  WeedZap — fixed-points accuracy test (no camera)")
    print(f"  Port: {port if not dry_run else '(dry-run)'}")
    print(f"{'='*60}\n")

    ser = None
    if not dry_run:
        ser = open_serial(port, baud)

    if do_home and not dry_run:
        print("[STEP 1] Homing all axes ...")
        send_and_wait(ser, "h", keyword="HOMING COMPLETE")
        print("[INFO] Homed.\n")
    else:
        print("[STEP 1] Skipping home (add --home to home first)\n")

    if not skip_tour:
        calibrations = load_all_calibrations(str(path_mod.CALIB_FOLDER))
        print("[STEP 2] Simulating tour scan (motion only, no camera) ...\n")
        simulate_tour(ser, dry_run, calibrations)
    else:
        print("[STEP 2] Skipping simulated tour (--skip-tour)\n")

    if dry_run:
        print("[STEP 3] (dry-run) Would attach laser (tool take 1).\n")
    else:
        print("[STEP 3] Attaching laser tool ...")
        send_and_wait(ser, "tool take 1", keyword="Tool 1 ready")
        print("[INFO] Laser tool attached.\n")

    visit_fixed_points(ser, dry_run, laser_power, seconds, confirm_each)

    if do_tool_put:
        if dry_run:
            print("\n[STEP 4] (dry-run) Would return laser tool (tool put).")
        else:
            print("\n[STEP 4] Returning laser tool ...")
            send_and_wait(ser, "tool put", keyword="returned")
            print("[INFO] Tool returned.")

    print("\n[INFO] Fixed-points test complete.")
    if ser:
        ser.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Fire on known real crown positions instead of detected "
                    "ones, to isolate motion/calibration accuracy from "
                    "detection accuracy. Never opens the camera.")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--home", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--skip-tour", action="store_true",
                        help="Skip the simulated row-scan motion, go straight to the fixed points.")
    parser.add_argument("--laser-power", type=int, default=255, metavar="0-255")
    parser.add_argument("--seconds", type=float, required=True, metavar="SECONDS",
                        help="Laser dwell time per point, in seconds.")
    parser.add_argument("--confirm-each", action="store_true",
                        help="Pause for Enter before firing at each point "
                             "(default: fully automatic).")
    parser.add_argument("--no-tool-put", action="store_true")
    args = parser.parse_args()

    if args.port is None and not args.dry_run:
        import serial.tools.list_ports
        print("Available serial ports:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device:12s}  {p.description}")
        print("\nRe-run with  --port <port>  or add  --dry-run  to test without hardware.")
        sys.exit(0)

    run(
        port         = args.port or "DRYRUN",
        baud         = args.baud,
        do_home      = args.home,
        dry_run      = args.dry_run,
        skip_tour    = args.skip_tour,
        laser_power  = args.laser_power,
        seconds      = args.seconds,
        confirm_each = args.confirm_each,
        do_tool_put  = not args.no_tool_put,
    )
