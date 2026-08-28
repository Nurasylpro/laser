"""
weed_zap.py — full pipeline: position → photo → detect → path → move laser

Run:
    python scripts/weed_zap.py --port COM3 --z 50

What it does:
    1. Connects to Arduino, reads current gantry X/Y position
    2. Captures photo from USB camera
    3. Runs YOLO weed detection on that photo
    4. Converts pixel → mm using current gantry position as reference
    5. Builds optimised visit path
    6. Moves laser to each weed in order

Optional flags:
    --home          Home all axes before starting
    --camera N      Camera index (default 0)
    --preview       Show camera preview before capture
    --dry-run       Full pipeline but no serial / no movement
    --conf 0.38     YOLO confidence threshold
    --row-scan      Work both rows end-to-end with a fresh photo at every
                     camera-FOV-width stop, instead of a single photo
"""

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

import cv2

# ── Project paths ───────────────────────────────────────────────
SCRIPT_DIR   = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(SCRIPT_DIR))

# ── Import pipeline modules ─────────────────────────────────────
import photo as photo_mod
import path  as path_mod
from calibration_loader import pixel_to_world_at_height

try:
    import serial
except ImportError:
    print("[ERROR] pyserial not installed.  Run:  pip install pyserial")
    sys.exit(1)


# ── Row-scan settings (laser tool, row-by-row multi-photo pass) ──
# Mirrors the mechanical tool's row/tool-rack conventions in
# firmware/test_sketch/test_sketch.ino (WEED_ROW1/2_Y_MM, WEED_X_END_MM,
# TOOL1_X, TOOL_X_OFFSET) — same physical bed and rows, different tool.
LASER_VANTAGE_Z_MM = 0.0     # physical Z for both the row-scan photo and firing (2026-08-02: was 90)
TOOL1_X_MM         = 10.0    # laser tool rack X (mirrors firmware TOOL1_X)
TOOL_X_OFFSET_MM   = 130.0   # approach/retreat clearance (mirrors firmware TOOL_X_OFFSET; 2026-08-02: was 100, +30mm)
LASER_X_START_MM   = TOOL1_X_MM + TOOL_X_OFFSET_MM   # = 140, clear of the laser's own rack
LASER_X_END_MM     = 680.0   # 800mm bed - 100mm safety margin (bed geometry, not tool-specific)
LASER_ROW1_Y_MM    = 120.0
LASER_ROW2_Y_MM    = 330.0
LASER_ROWS_Y_MM     = [LASER_ROW1_Y_MM, LASER_ROW2_Y_MM]
LASER_ROW_SCAN_OVERLAP_FRACTION = 0.90   # advance = FOV * this; a gap is worse than mild overlap


def to_calib_z(z_physical: float) -> float:
    """
    Physical/commanded gantry Z -> the Z value calibration math should use.
    calibration2/ was captured before soil was added to the bed; the soil
    raised the working floor 88mm closer to the camera, so calibration
    lookups need +88 applied. NEVER pass this result as an actual move
    target — only into photo_mod.run()'s z_mm and Visualizer's z_mm.
    """
    return z_physical + path_mod.Z_CALIB_OFFSET_MM


# ── Background camera thread ────────────────────────────────────

class LiveCamera:
    """
    Reads camera frames in a background thread.
    Call get_frame() anytime to get the latest frame without blocking.
    """
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


# ── Live visualizer ─────────────────────────────────────────────

class Visualizer:
    """
    Opens a cv2 window showing the captured image with:
      - Blue  cross  = laser beam home position (abs 0,0)
      - Red   dot    = weed not yet visited
      - Yellow dot   = weed currently being moved to
      - Green  dot   = weed already visited
      - Grey  arrows = planned path order
      - Status bar   = current action
    """

    WIN = "WeedZap"

    def __init__(self, image_path: str, weeds: list,
                 calibrations: dict, z_mm: float,
                 live_cam: LiveCamera | None = None):
        self.weeds    = weeds
        self.live_cam = live_cam

        img = cv2.imread(image_path)
        if img is None:
            raise FileNotFoundError(f"Cannot open image: {image_path}")
        self.base = img
        h, w = img.shape[:2]
        self.cx, self.cy = w / 2.0, h / 2.0

        # mm-per-pixel scale sampled at frame centre.
        # Horizontal pixels correspond to gantry Y axis,
        # vertical pixels correspond to gantry X axis (axis swap).
        p0 = pixel_to_world_at_height((self.cx,     self.cy),     z_mm, calibrations)
        p1 = pixel_to_world_at_height((self.cx + 1, self.cy),     z_mm, calibrations)
        p2 = pixel_to_world_at_height((self.cx,     self.cy + 1), z_mm, calibrations)
        self.mpp_h = p1[0] - p0[0]   # mm per horizontal pixel (gantry Y direction)
        self.mpp_v = p2[1] - p0[1]   # mm per vertical pixel  (gantry X direction)

        # Laser beam pixel position.
        # Laser sits at world offset (-CAM_TO_LASER_X, -CAM_TO_LASER_Y) from camera,
        # in gantry coords (X=forward, Y=left).
        # Horizontal pixel uses CAM_TO_LASER_Y, vertical pixel uses CAM_TO_LASER_X.
        # MINUS, matching test_laser_point.py's already-validated formula —
        # confirmed 2026-08-02 (this was PLUS and drew the crosshair in the
        # wrong spot; do not flip back without re-validating against reality).
        self.laser_px = int(self.cx - path_mod.CAM_TO_LASER_Y / self.mpp_h)
        self.laser_py = int(self.cy - path_mod.CAM_TO_LASER_X / self.mpp_v)

        # Display size — image is resized proportionally inside draw()
        self.display_w = 1280
        self.display_h = 720
        cv2.namedWindow(self.WIN, cv2.WINDOW_AUTOSIZE)

    def draw(self, current_order: int | None, visited: set[int], status: str = ""):
        """Redraw the window. Call before and after each move."""
        frame = self.base.copy()

        # ── grey path arrows ──────────────────────────────────────
        prev = (self.laser_px, self.laser_py)
        for w in self.weeds:
            curr = (int(w["crown_px"]), int(w["crown_py"]))
            cv2.arrowedLine(frame, prev, curr,
                            (160, 160, 160), 1, tipLength=0.04)
            prev = curr

        # ── laser beam home position ──────────────────────────────
        cv2.drawMarker(frame, (self.laser_px, self.laser_py),
                       (255, 120, 0), cv2.MARKER_CROSS, 28, 2)
        cv2.putText(frame, "laser (0,0)",
                    (self.laser_px + 10, self.laser_py - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 180, 0), 1, cv2.LINE_AA)

        # ── weed dots ─────────────────────────────────────────────
        for w in self.weeds:
            order = w["visit_order"]
            px    = int(w["crown_px"])
            py    = int(w["crown_py"])

            if order in visited:
                color      = (0, 210, 0)     # green  — done
                radius     = 8
                thickness  = -1
            elif order == current_order:
                color      = (0, 230, 255)   # yellow — current target
                radius     = 14
                thickness  = -1
                # pulsing ring
                cv2.circle(frame, (px, py), 22, (0, 200, 255), 2)
            else:
                color      = (0, 0, 220)     # red    — pending
                radius     = 10
                thickness  = -1

            cv2.circle(frame, (px, py), radius, color, thickness)

            # distance label from laser
            rel_x   = w["x_mm"] - path_mod.CAM_TO_LASER_X
            rel_y   = w["y_mm"] - path_mod.CAM_TO_LASER_Y
            dist_mm = math.hypot(rel_x, rel_y)
            label   = f"#{order}  {dist_mm:.0f}mm"
            cv2.putText(frame, label, (px + 12, py - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        (255, 255, 255), 1, cv2.LINE_AA)

        # ── status bar at top ─────────────────────────────────────
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 34), (30, 30, 30), -1)
        cv2.putText(frame, status, (10, 23),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (200, 255, 200), 1, cv2.LINE_AA)

        # ── live camera inset (bottom-right corner) ───────────────
        if self.live_cam is not None:
            live = self.live_cam.get_frame()
            if live is not None:
                fh, fw   = frame.shape[:2]
                inset_w  = fw // 3          # inset is 1/3 of main frame width
                inset_h  = int(inset_w * live.shape[0] / live.shape[1])
                live_sm  = cv2.resize(live, (inset_w, inset_h))

                # draw laser crosshair on live feed (always at cam center + offset)
                lh, lw = live_sm.shape[:2]
                # laser position in live feed coordinates
                scale_x  = lw / live.shape[1]
                scale_y  = lh / live.shape[0]
                lx_live  = int(self.laser_px * scale_x)
                ly_live  = int(self.laser_py * scale_y)
                cv2.drawMarker(live_sm, (lx_live, ly_live),
                               (255, 120, 0), cv2.MARKER_CROSS, 20, 2)

                # border around inset
                cv2.rectangle(live_sm, (0, 0), (lw - 1, lh - 1),
                              (255, 120, 0), 2)
                cv2.putText(live_sm, "LIVE", (6, 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 220, 255), 1, cv2.LINE_AA)

                # paste into bottom-right of frame
                margin = 10
                y1 = fh - inset_h - margin
                x1 = fw - inset_w - margin
                frame[y1:y1 + inset_h, x1:x1 + inset_w] = live_sm

        # Resize proportionally to fit display size
        h, w  = frame.shape[:2]
        scale = min(self.display_w / w, self.display_h / h)
        frame = cv2.resize(frame, (int(w * scale), int(h * scale)))

        cv2.imshow(self.WIN, frame)
        cv2.waitKey(1)   # non-blocking — keeps window alive

    def close(self):
        try:
            cv2.destroyWindow(self.WIN)
        except cv2.error:
            pass   # window was already closed manually — that's fine


def compute_row_scan_advance_mm(z_calib: float, calibrations: dict,
                                img_w: int, img_h: int,
                                overlap_fraction: float) -> float:
    """
    Real-world X-direction (row-sweep direction) distance one photo covers
    at the given calibration Z, so consecutive row-scan stops can tile
    edge-to-edge. Image HEIGHT maps to gantry X (not width — see the axis
    swap noted in Visualizer.__init__ and path.py's convert_weeds_to_mm),
    so this measures the vertical extent of the frame, not horizontal.
    """
    top    = pixel_to_world_at_height((img_w / 2.0, 0.0),        z_calib, calibrations)
    bottom = pixel_to_world_at_height((img_w / 2.0, img_h - 1.0), z_calib, calibrations)
    vert_fov_mm = math.hypot(bottom[0] - top[0], bottom[1] - top[1])
    return vert_fov_mm * overlap_fraction


# ── Serial helpers (self-contained so no import from move.py) ───

def open_serial(port: str, baud: int) -> serial.Serial:
    try:
        # dtr=False prevents Arduino from resetting when Python opens the port.
        # Without this, Arduino resets every run → loses homed state → refuses moves.
        ser = serial.Serial()
        ser.port     = port
        ser.baudrate = baud
        ser.timeout  = 1.0
        ser.dtr      = False   # ← key line: no reset on connect
        ser.open()
        print(f"[INFO] Opened {port} at {baud} baud (no reset).")
        time.sleep(0.5)
        # Flush any partial command left in Arduino's RX buffer from a previous run.
        # Sending "\n" terminates whatever incomplete bytes were waiting there,
        # Arduino processes it (may print "Unknown command"), we discard everything.
        ser.write(b"\n")
        time.sleep(0.2)
        ser.reset_input_buffer()
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open {port}: {e}")
        sys.exit(1)


def send_and_wait(ser: serial.Serial, cmd: str, keyword: str) -> str:
    """
    Send cmd, block until a response line contains keyword. No timeout —
    physical operations (homing, tool changes, moves) can legitimately take
    longer than any fixed ceiling would allow; a timeout here just crashes
    the pipeline mid-operation instead of letting it finish. If something is
    actually stuck, Ctrl+C.
    """
    print(f"  → {cmd}")
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


def get_gantry_position(ser: serial.Serial) -> tuple[float, float, float]:
    """Query 'p' command, block until X/Y/Z mm can be parsed. Returns (x, y, z)."""
    print("  → p")
    ser.write(b"p\n")
    while True:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if "X=" in line and "Y=" in line and "Z=" in line:
            print(f"    Arduino: {line}")
            try:
                # "X=12.50mm  Y=34.00mm  Z=50.00mm  ..."
                parts = line.split()
                x = float(parts[0].replace("X=", "").replace("mm", ""))
                y = float(parts[1].replace("Y=", "").replace("mm", ""))
                z = float(parts[2].replace("Z=", "").replace("mm", ""))
                return x, y, z
            except (IndexError, ValueError):
                pass


# ── Main pipeline ───────────────────────────────────────────────

def run(port: str, baud: int, z_mm_override: float | None,
        camera_index: int, preview: bool,
        image_path_override: str | None,
        do_home: bool, z_move: float | None,
        dry_run: bool,
        conf_thresh: float, model_path: str,
        apply_filter: bool = True,
        weed_height_mm: float = path_mod.WEED_HEIGHT_MM,
        laser_power: int = 80, laser_time: float = 3.0,
        row_scan: bool = False):

    print(f"\n{'='*60}")
    print(f"  WeedZap — full pipeline")
    print(f"  Port: {port if not dry_run else '(dry-run)'}")
    print(f"{'='*60}\n")

    # ── 1. Serial connection ────────────────────────────────────
    ser = None
    if not dry_run:
        ser = open_serial(port, baud)

    # ── 2. Optional home ────────────────────────────────────────
    if do_home and not dry_run:
        print("\n[STEP 1] Homing all axes …")
        send_and_wait(ser, "h", keyword="HOMING COMPLETE")
        print("[INFO] Homed.\n")
    else:
        print("[STEP 1] Skipping home (add --home to home first)\n")

    # ── 2b. Attach the laser tool ────────────────────────────────
    # Requires the gantry to already be homed (either via --home above, or
    # from a previous run) — the firmware refuses "tool take" otherwise.
    # If a DIFFERENT tool (e.g. tool 2, the mech weeder) is still attached
    # from earlier testing, the firmware also refuses ("Put current tool
    # first: tool put") and this will time out — run "tool put" by hand
    # (e.g. via jog.py) first if that happens.
    if dry_run:
        print("[STEP 1b] (dry-run) Would attach laser (tool take 1).\n")
    else:
        print("[STEP 1b] Attaching laser tool …")
        send_and_wait(ser, "tool take 1", keyword="Tool 1 ready")
        print("[INFO] Laser tool attached.\n")

    if row_scan:
        run_row_scan(ser=ser, dry_run=dry_run, camera_index=camera_index,
                     conf_thresh=conf_thresh, model_path=model_path,
                     apply_filter=apply_filter, weed_height_mm=weed_height_mm,
                     laser_power=laser_power, laser_time=laser_time,
                     image_path_override=image_path_override)
        if ser:
            ser.close()
        return

    # ── 3. Read current gantry position ─────────────────────────
    print("[STEP 2] Reading current gantry position …")
    if dry_run:
        gantry_x, gantry_y, gantry_z = 0.0, 0.0, z_mm_override or 0.0
        print(f"  (dry-run) Assuming position X=0  Y=0  Z={gantry_z}")
    else:
        gantry_x, gantry_y, gantry_z = get_gantry_position(ser)

    # z_physical is the real gantry Z — the only thing ever sent as a move
    # target. z_calib is z_physical + the soil-raise offset (see
    # to_calib_z()) and is the only thing ever fed into calibration/pixel
    # math (photo_mod.run's z_mm, Visualizer's z_mm). Never cross the two —
    # see the header comment on to_calib_z() for why.
    # --z overrides z_calib directly (its docs already say "for calibration
    # lookup"), bypassing the +88 math entirely.
    z_physical = gantry_z
    z_calib    = z_mm_override if z_mm_override is not None else to_calib_z(z_physical)

    print(f"  Gantry at  X={gantry_x:.1f} mm  Y={gantry_y:.1f} mm  Z={gantry_z:.1f} mm")
    if z_mm_override is not None:
        print(f"  Z override → using Z={z_calib:.1f} mm for calibration")
    else:
        print(f"  Calibration Z = {z_physical:.1f} + {path_mod.Z_CALIB_OFFSET_MM:.0f} (soil offset) = {z_calib:.1f} mm")
    print()

    # ── 4. Capture photo (or use existing image) ────────────────
    if image_path_override:
        image_path = image_path_override
        print(f"[STEP 3] Using existing image: {image_path}\n")
    else:
        print("[STEP 3] Capturing photo …")
        image_path = photo_mod.capture_from_camera(camera_index, preview=preview)
        print()

    # ── 5. YOLO detection ───────────────────────────────────────
    print("[STEP 4] Running weed detection …")
    detection_json = photo_mod.run(
        image_path  = image_path,
        z_mm        = z_calib,
        model_path  = model_path,
        conf_thresh = conf_thresh,
    )

    if detection_json is None:
        print("[INFO] No weeds detected. Done.")
        if ser:
            ser.close()
        return
    print()

    # ── 6. Build path with current gantry position ──────────────
    print("[STEP 5] Computing weed path …")
    path_mod.run(
        detection_json = str(detection_json),
        gantry_x       = gantry_x,
        gantry_y       = gantry_y,
        apply_filter   = apply_filter,
        weed_height_mm = weed_height_mm,
    )

    # Reload path JSON that path.py just saved
    stem      = Path(detection_json).stem
    path_json = PROJECT_ROOT / "runs" / "paths" / f"{stem}_path.json"

    with open(path_json) as f:
        path_data = json.load(f)

    weeds = path_data["path"]
    print(f"  {len(weeds)} weeds in path.\n")

    if not weeds:
        print("[INFO] No weeds inside gantry bounds. Done.")
        if ser:
            ser.close()
        return

    # ── 7. Open visualizer + live camera ───────────────────────
    calibrations = path_mod.load_all_calibrations(str(path_mod.CALIB_FOLDER))
    time.sleep(1.0)          # let camera settle after capture_from_camera released it
    live_cam     = LiveCamera(camera_index)
    time.sleep(0.5)          # give thread time to grab first frame
    viz          = Visualizer(image_path, weeds, calibrations, z_calib,
                              live_cam=live_cam)
    visited      = set()

    # Show full path before moving
    viz.draw(current_order=None, visited=visited,
             status=f"Path ready — {len(weeds)} weeds")
    print("[INFO] Visualization window opened.\n")

    # ── 7b. Drop Z to working height before visiting weeds ──────
    if z_move is not None:
        if dry_run:
            print(f"[STEP 6a] (dry-run) Would drop Z to {z_move:.1f} mm.\n")
        else:
            print(f"[STEP 6a] Dropping Z to working height {z_move:.1f} mm …")
            send_and_wait(ser, f"m 0 0 {z_move:.2f}", keyword="Move complete")
            print(f"[INFO] Z at {z_move:.1f} mm. Ready to zap.\n")

    # ── 8. Move laser to each weed ──────────────────────────────
    print(f"[STEP 6] Moving laser to {len(weeds)} weeds …\n")

    for w in weeds:
        num    = w["visit_order"]
        abs_x  = w["abs_x"]
        abs_y  = w["abs_y"]
        move_z = z_move if z_move is not None else z_physical   # use --z-move if given; NEVER z_calib here

        print(f"  Weed #{num}  →  X={abs_x:+.1f} mm  Y={abs_y:+.1f} mm  Z={move_z:.1f} mm")

        # Highlight current target before moving
        viz.draw(current_order=num, visited=visited,
                 status=f"Moving to weed #{num}  |  X={abs_x:+.0f}  Y={abs_y:+.0f} mm")

        if dry_run:
            print(f"  (dry-run) Would send:  m {abs_x:.2f} {abs_y:.2f} {move_z:.2f}")
            time.sleep(0.8)   # simulate travel time so you can see the window update
        else:
            cmd = f"m {abs_x:.2f} {abs_y:.2f} {move_z:.2f}"
            send_and_wait(ser, cmd, keyword="Move complete")

            # ── fire laser ────────────────────────────────────────
            # try/finally guarantees "laser 0" is sent even if the dwell is
            # interrupted (Ctrl+C, exception) — never leave the beam on.
            print(f"  Firing laser at power {laser_power} for {laser_time:.1f}s …")
            try:
                send_and_wait(ser, f"laser {laser_power}", keyword="Laser ->")
                time.sleep(laser_time)
            finally:
                send_and_wait(ser, "laser 0", keyword="Laser ->")
            print(f"  Laser off.")

        # Mark as visited — dot turns green
        visited.add(num)
        viz.draw(current_order=None, visited=visited,
                 status=f"Weed #{num} done  ({len(visited)}/{len(weeds)}) — press Enter for next")

        # Wait for user to press Enter before moving to next weed
        if num < weeds[-1]["visit_order"]:   # skip wait after the last weed
            input(f"  ✓ Weed #{num} done. Press Enter to move to next weed …")
        print(f"  ✓ Weed #{num} done.\n")

    viz.draw(current_order=None, visited=visited,
             status=f"All {len(weeds)} weeds done!  Press any key to close.")
    print("[INFO] All weeds visited. Pipeline complete.")
    print("[INFO] Press any key in the visualizer window to exit.")
    cv2.waitKey(0)
    live_cam.stop()
    viz.close()

    if ser:
        ser.close()


def run_row_scan(ser, dry_run: bool, camera_index: int,
                 conf_thresh: float, model_path: str,
                 apply_filter: bool, weed_height_mm: float,
                 laser_power: int, laser_time: float,
                 image_path_override: str | None = None):
    """
    Row-by-row laser pass: at each row, take repeated photos stepping in X
    from LASER_X_END_MM (far end, away from home) back toward
    LASER_X_START_MM (near home) — tiled by the camera's real field-of-view,
    so consecutive photos cover the row edge-to-edge — firing every weed
    found at each stop, until the row is covered, then move to the next row.

    Photo/vantage and firing both happen at the same physical
    LASER_VANTAGE_Z_MM — no separate descend step (see plan for reasoning).
    Reuses the exact same fire block as the single-shot pipeline; only the
    outer stop/row looping and per-stop photo/detect/path calls are new.
    """
    z_physical_vantage = LASER_VANTAGE_Z_MM
    z_calib_vantage    = to_calib_z(z_physical_vantage)

    print(f"[ROW-SCAN] vantage Z: physical={z_physical_vantage:.1f}  "
          f"calib={z_calib_vantage:.1f}  "
          f"({'within' if z_calib_vantage <= 165 else 'EXTRAPOLATED beyond'} calibration2's sampled 0-165 range)")

    calibrations = path_mod.load_all_calibrations(str(path_mod.CALIB_FOLDER))
    advance_mm = compute_row_scan_advance_mm(
        z_calib_vantage, calibrations,
        photo_mod.CAPTURE_WIDTH, photo_mod.CAPTURE_HEIGHT,
        LASER_ROW_SCAN_OVERLAP_FRACTION)
    print(f"[ROW-SCAN] X-advance per stop: {advance_mm:.0f}mm "
          f"(rows span X={LASER_X_START_MM:.0f}-{LASER_X_END_MM:.0f}mm)\n")

    live_cam = None

    for row_index, row_y in enumerate(LASER_ROWS_Y_MM):
        row_num     = row_index + 1
        is_last_row = (row_index == len(LASER_ROWS_Y_MM) - 1)
        print(f"[ROW {row_num}/{len(LASER_ROWS_Y_MM)}] Y={row_y:.1f}mm\n")

        # Sweep starts at the FAR end (away from home) and steps back toward
        # home — reversed from a naive near-to-far sweep, per 2026-08-02 request.
        stop_x, stop_index = LASER_X_END_MM, 0

        while stop_x > LASER_X_START_MM:
            stop_index += 1
            next_stop_x = stop_x - advance_mm
            is_last_stop_in_row = next_stop_x <= LASER_X_START_MM

            print(f"  [STOP {stop_index}] X={stop_x:.1f}  Y={row_y:.1f}  "
                  f"Z={z_physical_vantage:.1f} (physical)")

            if dry_run:
                print(f"    (dry-run) Would send:  m {stop_x:.2f} {row_y:.2f} {z_physical_vantage:.2f}")
            else:
                cmd = f"m {stop_x:.2f} {row_y:.2f} {z_physical_vantage:.2f}"
                send_and_wait(ser, cmd, keyword="Move complete")

            # ── capture (skip real camera entirely if a static test image was given) ──
            if image_path_override:
                image_path = image_path_override
            else:
                if live_cam is not None:
                    live_cam.stop()
                    live_cam = None
                image_path = photo_mod.capture_from_camera(camera_index, preview=False)

            # ── detect ──────────────────────────────────────────────
            detection_json = photo_mod.run(
                image_path  = image_path,
                z_mm        = z_calib_vantage,
                model_path  = model_path,
                conf_thresh = conf_thresh,
            )

            weeds = []
            if detection_json is not None:
                # ── path — anchor on the commanded stop position, which the
                # firmware's "Move complete" echo already confirmed we reached ──
                path_mod.run(
                    detection_json = str(detection_json),
                    gantry_x       = stop_x,
                    gantry_y       = row_y,
                    apply_filter   = apply_filter,
                    weed_height_mm = weed_height_mm,
                )
                stem      = Path(detection_json).stem
                path_json = PROJECT_ROOT / "runs" / "paths" / f"{stem}_path.json"
                if path_json.exists():
                    with open(path_json) as f:
                        weeds = json.load(f)["path"]

            print(f"    {len(weeds)} weed(s) this stop.")

            # ── fresh visualizer every stop — new photo, new weeds ────
            if not dry_run:
                if live_cam is None:
                    time.sleep(1.0)
                    live_cam = LiveCamera(camera_index)
                    time.sleep(0.5)
            viz     = Visualizer(image_path, weeds, calibrations, z_calib_vantage,
                                 live_cam=live_cam)
            visited = set()
            viz.draw(current_order=None, visited=visited,
                     status=f"Row {row_num} Stop {stop_index} — {len(weeds)} weed(s)")

            # ── fire loop — identical to the single-shot fire block ──
            for w in weeds:
                num   = w["visit_order"]
                abs_x = w["abs_x"]
                abs_y = w["abs_y"]
                move_z = z_physical_vantage   # same vantage height, NEVER z_calib
                is_last_weed_in_stop = (num == weeds[-1]["visit_order"])
                is_globally_last = is_last_row and is_last_stop_in_row and is_last_weed_in_stop

                print(f"    Weed #{num}  →  X={abs_x:+.1f} mm  Y={abs_y:+.1f} mm  Z={move_z:.1f} mm")

                viz.draw(current_order=num, visited=visited,
                         status=f"Row {row_num} Stop {stop_index} — moving to weed #{num}")

                if dry_run:
                    print(f"    (dry-run) Would send:  m {abs_x:.2f} {abs_y:.2f} {move_z:.2f}")
                    time.sleep(0.8)
                else:
                    cmd = f"m {abs_x:.2f} {abs_y:.2f} {move_z:.2f}"
                    send_and_wait(ser, cmd, keyword="Move complete")

                    print(f"    Firing laser at power {laser_power} for {laser_time:.1f}s …")
                    try:
                        send_and_wait(ser, f"laser {laser_power}", keyword="Laser ->")
                        time.sleep(laser_time)
                    finally:
                        send_and_wait(ser, "laser 0", keyword="Laser ->")
                    print(f"    Laser off.")

                visited.add(num)
                viz.draw(current_order=None, visited=visited,
                         status=f"Row {row_num} Stop {stop_index} — weed #{num} done")

                if not is_globally_last:
                    input(f"    ✓ Weed #{num} done. Press Enter to continue …")
                print(f"    ✓ Weed #{num} done.\n")

            viz.close()
            stop_x = next_stop_x   # always advance, even on a zero-weed stop

        print(f"[ROW {row_num}/{len(LASER_ROWS_Y_MM)}] Complete.\n")

    if live_cam is not None:
        live_cam.stop()
    print("[ROW-SCAN] All rows complete.")


# ── Entry point ─────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="WeedZap full pipeline: position → photo → detect → move laser."
    )
    parser.add_argument("--port", default=None,
                        help="Arduino serial port  e.g. COM3  or  /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--z", type=float, default=None,
                        help="Override Z height in mm for calibration lookup. "
                             "If omitted, Z is read from the Arduino automatically.")
    # Image source — camera (default) or an existing saved image
    src = parser.add_mutually_exclusive_group()
    src.add_argument("--camera", type=int, default=None, metavar="INDEX",
                     help="Camera index (default 0). Cannot be used with --image.")
    src.add_argument("--image", metavar="PATH",
                     help="Use an existing image file instead of capturing. "
                          "Good for dry-run testing.")
    parser.add_argument("--preview", action="store_true",
                        help="Show camera preview before capture (only with --camera)")
    parser.add_argument("--home", action="store_true",
                        help="Home all axes before starting")
    parser.add_argument("--z-move", type=float, default=None, metavar="MM",
                        help="After homing, move Z down to this height in mm "
                             "before taking the photo.  e.g. --z-move 100")
    parser.add_argument("--dry-run", action="store_true",
                        help="Run full pipeline but do not open serial or move motors")
    parser.add_argument("--no-filter", action="store_true",
                        help="Skip gantry bounds filtering — show all detected weeds")
    parser.add_argument("--conf", type=float, default=0.38,
                        help="YOLO confidence threshold (default 0.38)")
    parser.add_argument("--model",
                        default=str(PROJECT_ROOT / "model" / "best1.pt"),
                        help="Path to YOLO model weights")
    parser.add_argument("--weed-height", type=float,
                        default=path_mod.WEED_HEIGHT_MM,
                        help=f"Weed crown height above soil in mm "
                             f"(default {path_mod.WEED_HEIGHT_MM}). "
                             "Corrects parallax error for tall plants.")
    parser.add_argument("--no-parallax", action="store_true",
                        help="Disable parallax correction (treat plants as flat).")
    parser.add_argument("--laser-power", type=int, default=80, metavar="0-255",
                        help="Laser PWM power per weed (default 80 — low test power). "
                             "Firmware clamps to 0-255.")
    parser.add_argument("--laser-time", type=float, default=3.0, metavar="SECONDS",
                        help="Laser dwell time per weed in seconds (default 3.0)")
    parser.add_argument("--row-scan", action="store_true",
                        help="Work both rows end-to-end, taking a fresh photo every "
                             "time the camera's field of view has been covered, "
                             "instead of a single photo from wherever the gantry "
                             "happens to be. --camera still selects which camera to "
                             "use; --preview and --z-move are ignored; --image (if "
                             "given) is reused as a static test photo at every stop "
                             "instead of capturing live — useful for --dry-run "
                             "control-flow smoke-testing.")

    args = parser.parse_args()

    if args.port is None and not args.dry_run:
        import serial.tools.list_ports
        print("Available serial ports:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device:12s}  {p.description}")
        print("\nRe-run with  --port <port>  or add  --dry-run  to test without hardware.")
        sys.exit(0)

    # Default camera index to 0 if neither --camera nor --image was given
    camera_idx = args.camera if args.camera is not None else 0

    wh = 0.0 if args.no_parallax else args.weed_height

    run(
        port                = args.port or "DRYRUN",
        baud                = args.baud,
        z_mm_override       = args.z,
        camera_index        = camera_idx,
        preview             = args.preview,
        image_path_override = args.image,
        do_home             = args.home,
        z_move              = args.z_move,
        dry_run             = args.dry_run,
        conf_thresh         = args.conf,
        model_path          = args.model,
        apply_filter        = not args.no_filter,
        weed_height_mm      = wh,
        laser_power         = args.laser_power,
        laser_time          = args.laser_time,
        row_scan            = args.row_scan,
    )
