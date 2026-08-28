"""
path.py — convert crown pixels to mm, filter to gantry bounds, build a
visit-order path, visualize.

Pixel -> mm conversion is offset-from-frame-center, added to the CURRENT
gantry position at capture time (never a raw lookup into the calibration
photo's own coordinate frame) — see convert_weeds_to_mm() below. This is
the formula validated against the hand-derived sketch/math in this
project's planning session.
"""

import json
import math
import sys
from pathlib import Path
import cv2

SCRIPT_DIR   = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(SCRIPT_DIR))

from calibration_loader import load_all_calibrations, pixel_to_world_at_height

# ── Settings ───────────────────────────────────────────────────
CALIB_FOLDER = PROJECT_ROOT / "calibration"
OUTPUT_DIR   = PROJECT_ROOT / "runs" / "paths"

BAND_WIDTH_MM = 30    # forward (X) band size for grouping weeds when ordering visits

# Gantry safe working range (mm) — weeds outside this box are skipped,
# gantry cannot reach them. From firmware soft limits (test_sketch.ino):
# X travel 800mm, Y travel 500mm, with a small margin held back from the
# switches at the far ends.
GANTRY_X_MIN =   0.0
GANTRY_X_MAX = 755.0
GANTRY_Y_MIN =   0.0
GANTRY_Y_MAX = 490.0

# Physical offset: camera center relative to laser mount center (mm).
# Positive X = camera is further forward (toward X+) than laser mount.
# Positive Y = camera is further left   (toward Y+) than laser mount.
# Measured with scripts/measure_laser_offset.py (click the laser beam's
# actual pixel in a photo) and auto-loaded from calibration/cam_to_laser.json
# below if that file exists. The values here are only the fallback used
# before that measurement has ever been taken — re-run measure_laser_offset.py
# any time the camera/laser mount physically changes.
CAM_TO_LASER_X = 75.8
CAM_TO_LASER_Y = 47.87

_offset_path = CALIB_FOLDER / "cam_to_laser.json"
if _offset_path.exists():
    try:
        with open(_offset_path) as _f:
            _measured = json.load(_f)
        CAM_TO_LASER_X = _measured["cam_to_laser_x"]
        CAM_TO_LASER_Y = _measured["cam_to_laser_y"]
        print(f"[INFO] Loaded measured CAM_TO_LASER offset from {_offset_path.name}: "
              f"X={CAM_TO_LASER_X}  Y={CAM_TO_LASER_Y}")
    except Exception as _e:
        print(f"[WARNING] Could not load {_offset_path.name}: {_e} — using hardcoded fallback.")

# Camera height above the ground (soil) when gantry is at Z=0 (home).
# As Z increases the camera moves DOWN, so:  H_cam(Z) = CAMERA_HEIGHT_AT_Z0 - Z
# Measure once with a ruler; this is the only physical input parallax needs.
CAMERA_HEIGHT_AT_Z0 = 670.0   # mm — re-verify against the rebuilt bed/mount

# Weed crown height above soil surface (mm). Used to correct parallax:
# camera sees the crown, but the laser must hit the root. Override per-run
# with --weed-height.
WEED_HEIGHT_MM = 60.0

# NOTE: unlike the old dauren/ pipeline, there is no Z_CALIB_OFFSET_MM soil
# correction here — the laser/calibration z_0.npz / z_90.npz photos are
# taken directly at the real physical Z the gantry will command (per the
# 2026-08-03 calibration redo), so calibration Z == physical Z, no offset.


def camera_height_at(z_mm: float) -> float:
    """Actual camera height above the soil at gantry Z position."""
    return CAMERA_HEIGHT_AT_Z0 - z_mm


# ── Step 1: pixel -> mm ───────────────────────────────────────

def convert_weeds_to_mm(weeds, z_mm, calibrations, img_w, img_h,
                        gantry_x=0.0, gantry_y=0.0,
                        weed_height_mm: float = WEED_HEIGHT_MM):
    """
    Convert each weed's crown pixel to an absolute gantry target.

    Parallax correction
    -------------------
    The YOLO model detects the weed *crown* (top of plant), but the laser
    must hit the *base* (root at soil level). Because the camera looks down
    from height H_cam, a crown at height h_weed appears shifted away from
    the image centre compared to where the root is on the ground.

        scale        = (H_cam - h_weed) / H_cam
        corrected_px = cx + (crown_px - cx) * scale
        corrected_py = cy + (crown_py - cy) * scale

    Axis mapping (image -> gantry)
    -------------------------------
    x_mm = wy - cy_raw    (image "up"/forward -> gantry +X, away from home)
    y_mm = -(wx - cx_raw) (image "left" -> gantry +Y)
    This matches the hand-derived sketch: home is at the low-X end, X grows
    toward the far end of the bed; Y=0 is on the near side (home side), Y
    grows toward the far/left side.

    abs_x/abs_y are anchored on the CURRENT gantry position at the moment
    this photo was taken (gantry_x, gantry_y) — always pass a freshly
    queried position, never a commanded/assumed one, or every weed computed
    from that photo will be silently wrong.
    """
    cx = img_w / 2.0
    cy = img_h / 2.0
    center_px = (cx, cy)

    H_cam = camera_height_at(z_mm)

    if H_cam <= weed_height_mm or weed_height_mm <= 0:
        parallax_scale = 1.0
        print(f"  [PARALLAX] Disabled  (H_cam={H_cam:.1f} mm, weed_h={weed_height_mm:.1f} mm)")
    else:
        parallax_scale = (H_cam - weed_height_mm) / H_cam
        shift_pct      = (1.0 - parallax_scale) * 100.0
        print(f"  [PARALLAX] H_cam={H_cam:.1f} mm  weed_h={weed_height_mm:.1f} mm  "
              f"scale={parallax_scale:.4f}  ({shift_pct:.1f}% shift toward image centre)")

    cx_raw, cy_raw = pixel_to_world_at_height(center_px, z_mm, calibrations)

    result = []
    for w in weeds:
        px = w["crown_px"]
        py = w["crown_py"]

        corr_px = cx + (px - cx) * parallax_scale
        corr_py = cy + (py - cy) * parallax_scale

        wx, wy = pixel_to_world_at_height((corr_px, corr_py), z_mm, calibrations)

        x_mm = (wy - cy_raw)
        y_mm = -(wx - cx_raw)

        abs_x = gantry_x - CAM_TO_LASER_X + x_mm
        abs_y = gantry_y - CAM_TO_LASER_Y + y_mm

        result.append({
            **w,
            "x_mm":  round(x_mm,  2),
            "y_mm":  round(y_mm,  2),
            "abs_x": round(abs_x, 2),
            "abs_y": round(abs_y, 2),
        })

    return result


# ── Step 2: filter out-of-range weeds ─────────────────────────

def filter_weeds(weeds):
    """Remove weeds whose absolute gantry target is outside the safe working area."""
    kept    = []
    skipped = []

    for w in weeds:
        x, y = w["abs_x"], w["abs_y"]
        reason = None

        if x < GANTRY_X_MIN:
            reason = f"abs_x={x:+.1f} < min {GANTRY_X_MIN}"
        elif x > GANTRY_X_MAX:
            reason = f"abs_x={x:+.1f} > max {GANTRY_X_MAX}"
        elif y < GANTRY_Y_MIN:
            reason = f"abs_y={y:+.1f} < min {GANTRY_Y_MIN}"
        elif y > GANTRY_Y_MAX:
            reason = f"abs_y={y:+.1f} > max {GANTRY_Y_MAX}"

        if reason:
            skipped.append((w, reason))
        else:
            kept.append(w)

    if skipped:
        print(f"  [FILTER] Skipped {len(skipped)} weed(s) outside gantry bounds:")
        for w, reason in skipped:
            print(f"    weed id={w['id']}  {reason}")

    print(f"  [FILTER] {len(kept)} of {len(kept)+len(skipped)} weeds in range.")
    return kept


# ── Step 3: build path ─────────────────────────────────────────

def build_path(weeds, start_x: float = 0.0, start_y: float = 0.0):
    """
    Build visit order using banded nearest-neighbor.

    1. Assign each weed to a forward band (X // BAND_WIDTH)
    2. Process bands from lowest X (back) to highest X (front)
    3. Within each band, always go to nearest unvisited weed
    4. Current position updates after each visit
    """
    if not weeds:
        return []

    for w in weeds:
        w["band"] = int(w["abs_x"] // BAND_WIDTH_MM)

    bands     = sorted(set(w["band"] for w in weeds))
    current_x = start_x
    current_y = start_y
    path      = []
    remaining = weeds.copy()

    for band_index in bands:
        band_weeds = [w for w in remaining if w["band"] == band_index]

        while band_weeds:
            nearest = min(
                band_weeds,
                key=lambda w: math.dist((current_x, current_y), (w["abs_x"], w["abs_y"]))
            )
            path.append(nearest)
            band_weeds.remove(nearest)
            remaining.remove(nearest)
            current_x = nearest["abs_x"]
            current_y = nearest["abs_y"]

    for i, w in enumerate(path):
        w["visit_order"] = i

    return path


# ── Step 4: visualize ──────────────────────────────────────────

def visualize_path(image_path, path, img_w, img_h, output_path, calibrations, z_mm):
    """Draw the path on the original image."""
    image = cv2.imread(image_path)
    if image is None:
        print(f"[ERROR] Cannot read image for visualization: {image_path}")
        return

    cx = img_w / 2.0
    cy = img_h / 2.0

    p0 = pixel_to_world_at_height((cx,     cy),     z_mm, calibrations)
    p1 = pixel_to_world_at_height((cx + 1, cy),     z_mm, calibrations)
    p2 = pixel_to_world_at_height((cx,     cy + 1), z_mm, calibrations)
    mpp_h = p1[0] - p0[0]
    mpp_v = p2[1] - p0[1]

    laser_px = int(round(cx - CAM_TO_LASER_Y / mpp_h))
    laser_py = int(round(cy - CAM_TO_LASER_X / mpp_v))

    cv2.drawMarker(image, (int(cx), int(cy)), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
    cv2.putText(image, "camera", (int(cx) + 8, int(cy) - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)

    cv2.drawMarker(image, (laser_px, laser_py), (255, 100, 0), cv2.MARKER_CROSS, 24, 2)
    cv2.putText(image, "laser(0,0)", (laser_px + 8, laser_py - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 100, 0), 1, cv2.LINE_AA)

    prev_px, prev_py = laser_px, laser_py
    for w in path:
        curr_px, curr_py = int(w["crown_px"]), int(w["crown_py"])
        cv2.arrowedLine(image, (prev_px, prev_py), (curr_px, curr_py),
                        (255, 200, 0), 2, tipLength=0.03)
        prev_px, prev_py = curr_px, curr_py

    for w in path:
        px, py = int(w["crown_px"]), int(w["crown_py"])
        num = w["visit_order"]

        rel_x   = w["x_mm"] - CAM_TO_LASER_X
        rel_y   = w["y_mm"] - CAM_TO_LASER_Y
        dist_mm = math.hypot(rel_x, rel_y)

        cv2.circle(image, (px, py), 6, (0, 0, 255), -1)
        label = f"#{num} ({rel_x:+.0f},{rel_y:+.0f})mm d={dist_mm:.0f}mm"
        cv2.putText(image, label, (px + 8, py - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

    cv2.imwrite(str(output_path), image)
    print(f"[INFO] Path image saved -> {output_path}")


# ── Main ───────────────────────────────────────────────────────

def run(detection_json: str, gantry_x: float = 0.0, gantry_y: float = 0.0,
        apply_filter: bool = True, weed_height_mm: float = WEED_HEIGHT_MM,
        save_visualization: bool = True):

    with open(detection_json) as f:
        data = json.load(f)

    image_path = data["image"]
    z_mm       = data["z_mm"]
    img_w      = data["img_width"]
    img_h      = data["img_height"]
    weeds      = data["weeds"]

    print(f"[INFO] Loaded {len(weeds)} detections from {detection_json}")
    print(f"[INFO] Z = {z_mm} mm  |  gantry = ({gantry_x}, {gantry_y}) mm  |  "
          f"cam-to-laser = ({CAM_TO_LASER_X}, {CAM_TO_LASER_Y}) mm  |  "
          f"weed height = {weed_height_mm} mm")

    calibrations = load_all_calibrations(str(CALIB_FOLDER))
    print(f"[INFO] Calibrations loaded: {sorted(calibrations.keys())}")

    weeds_mm = convert_weeds_to_mm(weeds, z_mm, calibrations, img_w, img_h,
                                   gantry_x, gantry_y, weed_height_mm=weed_height_mm)

    if apply_filter:
        weeds_mm = filter_weeds(weeds_mm)
        if not weeds_mm:
            print("[INFO] No weeds inside gantry bounds after filtering.")
    else:
        print(f"  [FILTER] Disabled — keeping all {len(weeds_mm)} weeds.")

    path = build_path(weeds_mm, gantry_x, gantry_y) if weeds_mm else []

    print(f"\n{'='*74}")
    print(f"  VISIT ORDER  (Z={z_mm}mm,  gantry=({gantry_x},{gantry_y}),  "
          f"band={BAND_WIDTH_MM}mm,  weed_h={weed_height_mm}mm)")
    print(f"{'='*74}")
    print(f"  {'#':>3}  {'abs_X':>8}  {'abs_Y':>8}  {'cam_dx':>8}  {'cam_dy':>8}  {'Band':>5}  Src")
    print(f"  {'-'*62}")
    for w in path:
        print(f"  {w['visit_order']:>3}  "
              f"{w['abs_x']:>+8.1f}  {w['abs_y']:>+8.1f}  "
              f"{w['x_mm']:>+8.1f}  {w['y_mm']:>+8.1f}  "
              f"{w['band']:>5}  {w['source']}")
    print(f"{'='*74}\n")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    stem = Path(detection_json).stem

    if save_visualization:
        out_image = OUTPUT_DIR / f"{stem}_path.jpg"
        visualize_path(image_path, path, img_w, img_h, out_image, calibrations, z_mm)

    path_json = OUTPUT_DIR / f"{stem}_path.json"
    with open(path_json, "w") as f:
        json.dump({
            "z_mm":           z_mm,
            "gantry_x":       gantry_x,
            "gantry_y":       gantry_y,
            "cam_to_laser_x": CAM_TO_LASER_X,
            "cam_to_laser_y": CAM_TO_LASER_Y,
            "weed_height_mm": weed_height_mm,
            "path":           path,
        }, f, indent=2)
    print(f"[INFO] Path data saved -> {path_json}")

    return path_json
