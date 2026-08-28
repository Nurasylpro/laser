"""
weed_zap.py — tour-then-zap pipeline.

Why "tour then zap" instead of the old "attach laser, scan, zap as you go":
mounting the laser tool physically blocks/occludes the camera's view. So
this pipeline instead:

  1. Homes the gantry.
  2. TOUR phase — laser tool NOT attached, camera unobstructed. Sweeps both
     rows, taking a fresh photo at every camera-FOV-tiled stop, detecting
     weeds, and converting each to an absolute (X, Y) gantry position. Every
     position uses a FRESH 'p' query of the real gantry position taken right
     after that stop's move confirms complete — never the commanded/assumed
     stop coordinates — since trusting "Move complete" alone was the likely
     source of weeds being zapped at the wrong spot. Weeds are accumulated
     into one global list, de-duplicated across overlapping stops.
  3. Attaches the laser tool once ("tool take 1").
  4. VISIT phase — moves to every recorded weed position in turn and fires.

Run:
    python scripts/weed_zap.py --port COM3 --home
    python scripts/weed_zap.py --dry-run --image path\to\test.jpg
"""

import argparse
import json
import math
import re
import sys
import threading
import time
from pathlib import Path

import cv2

SCRIPT_DIR   = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(SCRIPT_DIR))

import photo as photo_mod
import path  as path_mod
import save_review
from calibration_loader import load_all_calibrations, pixel_to_world_at_height

try:
    import serial
except ImportError:
    print("[ERROR] pyserial not installed.  Run:  pip install pyserial")
    sys.exit(1)


# ── Row / tour settings ──────────────────────────────────────────
# Mirrors firmware/test_sketch/test_sketch.ino's tool-1 rack geometry
# (TOOL1_X=10, TOOL1_X_OFFSET=110 — 10mm wider than the shared
# TOOL_X_OFFSET=100, added 2026-08-04: the coarse move from the tool-1
# rack toward row 1 was passing too close to the mech tool's rack) and
# weeding-row Y positions (WEED_ROW1_Y_MM=120, WEED_ROW2_Y_MM=330 — same
# bed, same rows, laser tool instead of the mechanical one).
TOOL1_X_MM        = 10.0
TOOL1_X_OFFSET_MM = 110.0
ROW_X_START_MM    = TOOL1_X_MM + TOOL1_X_OFFSET_MM   # = 120, clear of the laser's own rack
ROW_X_END_MM      = 680.0    # 800mm bed - 120mm safety margin
ROWS_Y_MM         = [120.0, 330.0]
ROW_SCAN_OVERLAP_FRACTION = 0.90   # advance = FOV * this; a gap is worse than mild overlap

# Two Z heights were freshly calibrated (z_0, z_90 — see scripts/calibrate.py):
#   SCAN_Z_MM: used for the tour phase. Higher up (further from soil) means a
#     wider field of view per photo and, critically, no tool mounted to
#     block the camera.
#   FIRE_Z_MM: used for the visit/fire phase, once the laser tool is
#     attached — closer to the target for effective firing.
# Both are PHYSICAL Z values sent straight to the gantry; the new
# calibration was done directly at these physical heights (no soil-offset
# correction needed, unlike the old dauren/ pipeline).
SCAN_Z_MM = 0.0
FIRE_Z_MM = 130.0

# Two weeds recorded by different tour stops are treated as the same plant
# if their computed absolute positions are within this distance — adjacent
# stops' camera FOVs overlap (see ROW_SCAN_OVERLAP_FRACTION), so a weed near
# a stop boundary can otherwise get queued (and fired) twice.
DEDUP_DISTANCE_MM = 15.0


# ── Background camera thread / visualizer (unchanged pattern from the
#    previous pipeline; used only for optional live monitoring) ─────

class LiveCamera:
    def __init__(self, camera_index: int):
        self._cap   = cv2.VideoCapture(camera_index, photo_mod.CAMERA_BACKEND)
        self._frame = None
        self._lock  = threading.Lock()
        self._stop  = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop.is_set():
            ret, frame = self._cap.read()
            if ret:
                with self._lock:
                    self._frame = frame

    def get_frame(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def stop(self):
        self._stop.set()
        self._cap.release()


def compute_row_scan_advance_mm(z_mm: float, calibrations: dict,
                                img_w: int, img_h: int,
                                overlap_fraction: float) -> float:
    """
    Real-world X-direction (row-sweep direction) distance one photo covers,
    so consecutive tour stops tile edge-to-edge. Image HEIGHT maps to
    gantry X (see path.py's axis-swap in convert_weeds_to_mm), so this
    measures the vertical extent of the frame.
    """
    top    = pixel_to_world_at_height((img_w / 2.0, 0.0),        z_mm, calibrations)
    bottom = pixel_to_world_at_height((img_w / 2.0, img_h - 1.0), z_mm, calibrations)
    vert_fov_mm = math.hypot(bottom[0] - top[0], bottom[1] - top[1])
    return vert_fov_mm * overlap_fraction


# ── Serial helpers ────────────────────────────────────────────────

def open_serial(port: str, baud: int) -> "serial.Serial":
    try:
        ser = serial.Serial()
        ser.port     = port
        ser.baudrate = baud
        ser.timeout  = 1.0
        ser.dtr      = False   # no reset on connect — keeps homed state across script runs
        ser.open()
        print(f"[INFO] Opened {port} at {baud} baud (no reset).")
        time.sleep(0.5)
        ser.write(b"\n")
        time.sleep(0.2)
        ser.reset_input_buffer()
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open {port}: {e}")
        sys.exit(1)


def send_and_wait(ser: "serial.Serial", cmd: str, keyword: str) -> str:
    """Send cmd, block until a response line contains keyword. No timeout —
    physical operations can legitimately take a while; Ctrl+C if stuck."""
    print(f"  -> {cmd}")
    ser.write((cmd + "\n").encode("utf-8"))
    while True:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if line:
            print(f"    Arduino: {line}")
        if keyword.lower() in line.lower():
            return line


# Matches "X=12.50mm" / "Y=-3.0mm" / "Z=90mm" anywhere in the line,
# independent of field order or exact spacing — more robust than the old
# split()-by-position parsing, which silently broke if the firmware's
# format ever shifted.
_POS_RE = re.compile(r"([XYZ])=(-?\d+(?:\.\d+)?)mm", re.IGNORECASE)


def get_gantry_position(ser: "serial.Serial") -> tuple:
    """Query 'p', block until a line with X=/Y=/Z=<mm> can be parsed. Returns (x, y, z)."""
    print("  -> p")
    ser.write(b"p\n")
    while True:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue
        matches = dict((m.group(1).upper(), float(m.group(2))) for m in _POS_RE.finditer(line))
        if "X" in matches and "Y" in matches and "Z" in matches:
            print(f"    Arduino: {line}")
            return matches["X"], matches["Y"], matches["Z"]


# ── Tour phase ────────────────────────────────────────────────────

def dedupe_against(new_weed, existing, dist_mm=DEDUP_DISTANCE_MM):
    """True if new_weed is within dist_mm of any weed already in `existing`."""
    for w in existing:
        if math.dist((new_weed["abs_x"], new_weed["abs_y"]),
                      (w["abs_x"], w["abs_y"])) < dist_mm:
            return True
    return False


def run_tour(ser, dry_run: bool, camera_index: int, conf_thresh: float,
             model_path: str, weed_height_mm: float,
             image_path_override: str, calibrations: dict) -> list:
    """
    Sweep both rows with the camera unobstructed (no tool attached). At
    every FOV-tiled stop: move, re-query the real position, photo, detect,
    convert to absolute mm, and merge into one global de-duplicated list.
    Returns that global list (not yet ordered for firing).
    """
    advance_mm = compute_row_scan_advance_mm(
        SCAN_Z_MM, calibrations, photo_mod.CAPTURE_WIDTH, photo_mod.CAPTURE_HEIGHT,
        ROW_SCAN_OVERLAP_FRACTION)
    print(f"[TOUR] X-advance per stop: {advance_mm:.0f}mm "
          f"(rows span X={ROW_X_START_MM:.0f}-{ROW_X_END_MM:.0f}mm)\n")

    global_weeds = []
    next_id = 0

    for row_index, row_y in enumerate(ROWS_Y_MM):
        row_num = row_index + 1
        print(f"[TOUR ROW {row_num}/{len(ROWS_Y_MM)}] Y={row_y:.1f}mm\n")

        stop_x, stop_index = ROW_X_END_MM, 0
        while stop_x > ROW_X_START_MM:
            stop_index += 1
            print(f"  [STOP {stop_index}] commanded X={stop_x:.1f}  Y={row_y:.1f}  Z={SCAN_Z_MM:.1f}")

            if dry_run:
                print(f"    (dry-run) Would send:  m {stop_x:.2f} {row_y:.2f} {SCAN_Z_MM:.2f}")
                real_x, real_y, real_z = stop_x, row_y, SCAN_Z_MM
            else:
                cmd = f"m {stop_x:.2f} {row_y:.2f} {SCAN_Z_MM:.2f}"
                send_and_wait(ser, cmd, keyword="Move complete")
                # Fresh position query — never trust the commanded stop
                # coordinates alone, even after "Move complete".
                real_x, real_y, real_z = get_gantry_position(ser)
                print(f"    Actual position: X={real_x:.1f}  Y={real_y:.1f}  Z={real_z:.1f}")

            if image_path_override:
                image_path = image_path_override
            else:
                image_path = photo_mod.capture_from_camera(camera_index, preview=False)

            detection_json = photo_mod.run(
                image_path=image_path, z_mm=SCAN_Z_MM,
                model_path=model_path, conf_thresh=conf_thresh)

            stop_weeds = []
            if detection_json is not None:
                path_json = path_mod.run(
                    detection_json=str(detection_json),
                    gantry_x=real_x, gantry_y=real_y,
                    apply_filter=True, weed_height_mm=weed_height_mm,
                    save_visualization=False)
                with open(path_json) as f:
                    stop_weeds = json.load(f)["path"]

            added = 0
            for w in stop_weeds:
                if dedupe_against(w, global_weeds):
                    continue
                w["global_id"] = next_id
                next_id += 1
                global_weeds.append(w)
                added += 1

            print(f"    {len(stop_weeds)} weed(s) this stop, {added} new "
                  f"(total so far: {len(global_weeds)})")

            # Save this stop's photo annotated with every weed's crown
            # point, box, and computed absolute position — no blocking
            # window, just a file to scroll through afterward in
            # runs/review/ to check computed positions against reality.
            save_review.save_annotated(
                image_path, stop_weeds,
                label=f"row{row_num}_stop{stop_index}")

            next_stop_x = stop_x - advance_mm
            stop_x = next_stop_x

        print(f"[TOUR ROW {row_num}/{len(ROWS_Y_MM)}] Complete.\n")

    print(f"[TOUR] Complete. {len(global_weeds)} unique weed(s) recorded.\n")
    return global_weeds


# ── Visit/fire phase ────────────────────────────────────────────

def run_visit(ser, dry_run: bool, weeds: list, laser_power: int):
    """Attach the laser tool once, then visit every recorded weed and fire."""
    if dry_run:
        print("[VISIT] (dry-run) Would attach laser (tool take 1).\n")
    else:
        print("[VISIT] Attaching laser tool ...")
        send_and_wait(ser, "tool take 1", keyword="Tool 1 ready")
        print("[INFO] Laser tool attached.\n")

    if not weeds:
        print("[VISIT] No weeds recorded during the tour. Nothing to fire.")
        return

    # Order the accumulated global list into one coherent visit path,
    # starting from wherever the gantry is right after tool-take.
    ordered = path_mod.build_path(weeds, ROW_X_START_MM, ROWS_Y_MM[0])

    print(f"[VISIT] Visiting {len(ordered)} weed(s) at Z={FIRE_Z_MM:.1f}mm ...\n")

    for w in ordered:
        num   = w["visit_order"]
        abs_x = w["abs_x"]
        abs_y = w["abs_y"]

        print(f"  Weed #{num}  ->  X={abs_x:+.1f} mm  Y={abs_y:+.1f} mm  Z={FIRE_Z_MM:.1f} mm")

        if dry_run:
            print(f"  (dry-run) Would send:  m {abs_x:.2f} {abs_y:.2f} {FIRE_Z_MM:.2f}")
        else:
            cmd = f"m {abs_x:.2f} {abs_y:.2f} {FIRE_Z_MM:.2f}"
            send_and_wait(ser, cmd, keyword="Move complete")

            # Head is now sitting on the target — pause here so you can note
            # down / photograph exactly where the beam is about to land
            # before it actually fires.
            input(f"  Weed #{num} in position. Press Enter to fire ...")

            print(f"  Firing laser at power {laser_power} ...")
            try:
                send_and_wait(ser, f"laser {laser_power}", keyword="Laser ->")
                # Stays on until you say stop — Ctrl+C also lands in the
                # finally below, so the laser never gets left on by accident.
                input("  Laser ON. Press Enter to turn it off ...")
            finally:
                send_and_wait(ser, "laser 0", keyword="Laser ->")
            print("  Laser off.")

        print(f"  Weed #{num} done.\n")

    print("[VISIT] All weeds visited.")


# ── Main pipeline ───────────────────────────────────────────────

def run(port: str, baud: int, camera_index: int, image_path_override: str,
        do_home: bool, dry_run: bool, conf_thresh: float, model_path: str,
        weed_height_mm: float, laser_power: int,
        do_tool_put: bool):

    print(f"\n{'='*60}")
    print("  WeedZap — tour then zap")
    print(f"  Port: {port if not dry_run else '(dry-run)'}")
    print(f"{'='*60}\n")

    ser = None
    if not dry_run:
        ser = open_serial(port, baud)

    if do_home and not dry_run:
        print("\n[STEP 1] Homing all axes ...")
        send_and_wait(ser, "h", keyword="HOMING COMPLETE")
        print("[INFO] Homed.\n")
    else:
        print("[STEP 1] Skipping home (add --home to home first)\n")

    calibrations = load_all_calibrations(str(path_mod.CALIB_FOLDER))
    print(f"[INFO] Calibrations loaded: {sorted(calibrations.keys())}\n")

    print("[STEP 2] Tour — scanning rows without the laser tool attached ...\n")
    weeds = run_tour(ser, dry_run, camera_index, conf_thresh, model_path,
                     weed_height_mm, image_path_override, calibrations)

    # Persist the recorded positions to disk — revisit_and_photo.py reads
    # this afterward to drive the (tool-free) camera back to each weed for
    # a verification photo, since the laser tool blocks the camera view at
    # fire time and there's no way to visually confirm accuracy in the moment.
    tour_json = PROJECT_ROOT / "runs" / "review" / "tour_weeds.json"
    tour_json.parent.mkdir(parents=True, exist_ok=True)
    with open(tour_json, "w") as f:
        json.dump({"weeds": weeds}, f, indent=2)
    print(f"[INFO] Tour positions saved -> {tour_json}\n")

    print("[STEP 3] Visit — attaching laser tool and firing on every recorded weed ...\n")
    run_visit(ser, dry_run, weeds, laser_power)

    if do_tool_put:
        if dry_run:
            print("\n[STEP 4] (dry-run) Would return laser tool (tool put).")
        elif weeds:
            print("\n[STEP 4] Returning laser tool ...")
            send_and_wait(ser, "tool put", keyword="returned")
            print("[INFO] Tool returned.")

    print("\n[INFO] Pipeline complete.")
    if ser:
        ser.close()


# ── Entry point ─────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="WeedZap tour-then-zap pipeline: scan all rows (no tool "
                    "attached), then attach the laser and visit every weed.")
    parser.add_argument("--port", default=None,
                        help="Arduino serial port  e.g. COM3  or  /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--camera", type=int, default=0, metavar="INDEX",
                        help="Camera index (default 0). Ignored if --image is given.")
    parser.add_argument("--image", metavar="PATH", default=None,
                        help="Reuse one static image at every tour stop instead of "
                             "capturing live — for --dry-run control-flow testing.")
    parser.add_argument("--home", action="store_true",
                        help="Home all axes before starting")
    parser.add_argument("--dry-run", action="store_true",
                        help="Run full pipeline but do not open serial or move motors")
    parser.add_argument("--conf", type=float, default=0.38,
                        help="YOLO confidence threshold (default 0.38)")
    parser.add_argument("--model",
                        default=str(photo_mod.DEFAULT_MODEL),
                        help="Path to YOLO model weights")
    parser.add_argument("--weed-height", type=float, default=path_mod.WEED_HEIGHT_MM,
                        help=f"Weed crown height above soil in mm "
                             f"(default {path_mod.WEED_HEIGHT_MM}). Corrects parallax.")
    parser.add_argument("--no-parallax", action="store_true",
                        help="Disable parallax correction (treat plants as flat).")
    parser.add_argument("--laser-power", type=int, default=80, metavar="0-255",
                        help="Laser PWM power per weed (default 80 — low test power). "
                             "Fires until you press Enter, then turns off.")
    parser.add_argument("--no-tool-put", action="store_true",
                        help="Skip returning the laser tool to its rack at the end.")

    args = parser.parse_args()

    if args.port is None and not args.dry_run:
        import serial.tools.list_ports
        print("Available serial ports:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device:12s}  {p.description}")
        print("\nRe-run with  --port <port>  or add  --dry-run  to test without hardware.")
        sys.exit(0)

    wh = 0.0 if args.no_parallax else args.weed_height

    run(
        port                 = args.port or "DRYRUN",
        baud                 = args.baud,
        camera_index         = args.camera,
        image_path_override  = args.image,
        do_home              = args.home,
        dry_run              = args.dry_run,
        conf_thresh          = args.conf,
        model_path           = args.model,
        weed_height_mm       = wh,
        laser_power          = args.laser_power,
        do_tool_put          = not args.no_tool_put,
    )
