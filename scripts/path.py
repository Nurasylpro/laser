"""
path.py — convert crown pixels to mm, build path, visualize

Run:
    python scripts/path.py --detection runs/detections/weed1_z-9.json
    python scripts/path.py --detection runs/detections/weed1_z-9.json --weed-height 60
"""

import argparse
import json
import math
import sys
from pathlib import Path
import cv2

# ── Paths ──────────────────────────────────────────────────────
SCRIPT_DIR   = Path(__file__).resolve().parent   # .../DAUREN/scripts
PROJECT_ROOT = SCRIPT_DIR.parent                 # .../DAUREN
sys.path.insert(0, str(SCRIPT_DIR))

from calibration_loader import load_all_calibrations, pixel_to_world_at_height

# ── Settings ───────────────────────────────────────────────────
CALIB_FOLDER  = PROJECT_ROOT / "calibration2"
OUTPUT_DIR    = PROJECT_ROOT / "runs" / "paths"

BAND_WIDTH_MM = 30    # forward (X) band size for grouping weeds

# Gantry safe working range (mm) — from Arduino sketch:
#   X: 0  →  X_MAX_MM - X_HOME_OFFSET_MM  (800 - 45 = 755)
#   Y: 0  →  Y_MAX_MM - Y_HOME_OFFSET_MM  (500 - 10 = 490)
# Weeds outside this box are skipped — gantry cannot reach them.
GANTRY_X_MIN =   0.0
GANTRY_X_MAX = 755.0
GANTRY_Y_MIN =   0.0
GANTRY_Y_MAX = 490.0

# Physical offset: camera center relative to laser mount center (mm).
# Positive X = camera is further forward (toward X+) than laser mount.
# Positive Y = camera is further left   (toward Y+) than laser mount.
# Measure on the aluminum profile and set these before running.
CAM_TO_LASER_X = -65.7
CAM_TO_LASER_Y = 15

# Camera height above the ground (soil) when gantry is at Z=0 (home).
# As Z increases the camera moves DOWN, so:  H_cam(Z) = CAMERA_HEIGHT_AT_Z0 - Z
# Measure once with a ruler; this is the only physical input parallax needs.
CAMERA_HEIGHT_AT_Z0 = 670.0   # mm

# calibration2/ was captured before soil was added to the bed. Adding soil
# raised the working floor 88mm closer to the camera, so any Z value used
# for calibration lookups (homography + parallax) needs this added first —
# NEVER add this to a value that's actually sent to the gantry as a move
# target, only to values feeding calibration math (see weed_zap.py's
# z_physical/z_calib split and to_calib_z()).
Z_CALIB_OFFSET_MM = 88.0   # mm

# Weed crown height above soil surface (mm).
# Used to correct parallax: camera sees the crown, but laser must hit the root.
# Measure from a side photo with a ruler.  Override per-run with --weed-height.
WEED_HEIGHT_MM = 60.0


def camera_height_at(z_mm: float) -> float:
    """Actual camera height above the soil at gantry Z position."""
    return CAMERA_HEIGHT_AT_Z0 - z_mm


# ── Step 1: pixel → mm ─────────────────────────────────────────

def convert_weeds_to_mm(weeds, z_mm, calibrations, img_w, img_h,
                        gantry_x=0.0, gantry_y=0.0,
                        weed_height_mm: float = WEED_HEIGHT_MM):
    """
    Convert each weed's crown pixel to absolute gantry target coordinates.

    Parallax correction
    -------------------
    The YOLO model detects the weed *crown* (top of plant), but the laser
    must hit the *base* (root at soil level).  Because the camera looks down
    from height H_cam, a crown at height h_weed appears shifted away from
    the image centre compared to where the root is on the ground.

    Correction (similar-triangles geometry):
        scale        = (H_cam - h_weed) / H_cam
        corrected_px = cx + (crown_px - cx) * scale
        corrected_py = cy + (crown_py - cy) * scale

    The corrected pixel then maps to the ground-plane root position.

    Output fields per weed
    ----------------------
    x_mm / y_mm   — mm offset from camera frame centre (debug)
    abs_x / abs_y — absolute gantry target the laser mount must move to
    """
    cx = img_w / 2.0
    cy = img_h / 2.0
    center_px = (cx, cy)

    # ── parallax correction factor ──────────────────────────────
    # Camera height above ground at the current Z position
    H_cam = camera_height_at(z_mm)

    if H_cam <= weed_height_mm or weed_height_mm <= 0:
        # Camera is below crown level — correction undefined, disable
        parallax_scale = 1.0
        print(f"  [PARALLAX] Disabled  (H_cam={H_cam:.1f} mm, weed_h={weed_height_mm:.1f} mm)")
    else:
        parallax_scale = (H_cam - weed_height_mm) / H_cam
        shift_pct      = (1.0 - parallax_scale) * 100.0
        print(f"  [PARALLAX] H_cam={H_cam:.1f} mm  weed_h={weed_height_mm:.1f} mm  "
              f"scale={parallax_scale:.4f}  ({shift_pct:.1f}% shift toward image centre)")

    # ── frame centre in world mm ────────────────────────────────
    cx_raw, cy_raw = pixel_to_world_at_height(center_px, z_mm, calibrations)

    result = []
    for w in weeds:
        px = w["crown_px"]
        py = w["crown_py"]

        # Apply parallax correction: shift crown pixel toward image centre
        # to find where the weed root projects on the ground plane.
        corr_px = cx + (px - cx) * parallax_scale
        corr_py = cy + (py - cy) * parallax_scale

        wx, wy = pixel_to_world_at_height((corr_px, corr_py), z_mm, calibrations)

        # X axis = forward/backward (bottom to top in image, away from home = +X)
        # Y axis = left/right       (left to right in image, left = +Y)
        # wx and wy from homography are swapped relative to gantry axes.
        # X sign confirmed empirically 2026-08-02 via validate_click_points.py
        # (clicking a point closer to home was reading positive, backwards) —
        # do not reintroduce the negation without re-confirming on the bench.
        x_mm = (wy - cy_raw)    # forward (away from home) is +X
        y_mm = -(wx - cx_raw)   # left is +Y

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


# ── Step 2b: filter out-of-range weeds ────────────────────────

def filter_weeds(weeds):
    """
    Remove weeds whose absolute gantry target is outside the safe working area.
    Prints a reason for every skipped weed.
    """
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

def build_path(weeds):
    """
    Build visit order using banded nearest-neighbor.

    Algorithm:
        1. Assign each weed to a forward band (X // BAND_WIDTH)
        2. Process bands from lowest X (back) to highest X (front)
        3. Within each band, always go to nearest unvisited weed
        4. Current position updates after each visit

    Returns weeds in visit order with visit_order and band fields added.
    """
    if not weeds:
        return []

    for w in weeds:
        w["band"] = int(w["abs_x"] // BAND_WIDTH_MM)

    bands     = sorted(set(w["band"] for w in weeds))
    current_x = 0.0
    current_y = 0.0
    path      = []
    remaining = weeds.copy()

    for band_index in bands:
        band_weeds = [w for w in remaining if w["band"] == band_index]

        while band_weeds:
            nearest = min(
                band_weeds,
                key=lambda w: math.dist(
                    (current_x, current_y),
                    (w["abs_x"], w["abs_y"])
                )
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

def visualize_path(image_path, path, img_w, img_h, output_path,
                   calibrations, z_mm):
    """
    Draw the path on the original image.
    Crown dots are shown at detected pixel; root dots (corrected) overlay in cyan.
    """
    image = cv2.imread(image_path)
    if image is None:
        print(f"[ERROR] Cannot read image for visualization: {image_path}")
        return

    cx = img_w / 2.0
    cy = img_h / 2.0

    # Horizontal pixels = gantry-Y axis, vertical pixels = gantry-X axis (axis swap).
    p0 = pixel_to_world_at_height((cx,     cy),     z_mm, calibrations)
    p1 = pixel_to_world_at_height((cx + 1, cy),     z_mm, calibrations)
    p2 = pixel_to_world_at_height((cx,     cy + 1), z_mm, calibrations)
    mpp_h = p1[0] - p0[0]   # mm per horizontal pixel (gantry Y direction)
    mpp_v = p2[1] - p0[1]   # mm per vertical pixel  (gantry X direction)

    # Laser pixel position: laser sits at world offset (-CAM_TO_LASER_X, -CAM_TO_LASER_Y)
    # from the camera, in gantry coordinates (X=forward, Y=left).
    # Horizontal pixel uses CAM_TO_LASER_Y, vertical pixel uses CAM_TO_LASER_X.
    # MINUS, matching test_laser_point.py's already-validated formula —
    # confirmed 2026-08-02 (this was PLUS and drew the crosshair in the
    # wrong spot; do not flip back without re-validating against reality).
    laser_px = int(round(cx - CAM_TO_LASER_Y / mpp_h))
    laser_py = int(round(cy - CAM_TO_LASER_X / mpp_v))

    # ── reference markers ───────────────────────────────────────
    cv2.drawMarker(image, (int(cx), int(cy)), (0, 255, 0),
                   cv2.MARKER_CROSS, 20, 2)
    cv2.putText(image, "camera", (int(cx) + 8, int(cy) - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)

    cv2.drawMarker(image, (laser_px, laser_py), (255, 100, 0),
                   cv2.MARKER_CROSS, 24, 2)
    cv2.putText(image, "laser(0,0)",
                (laser_px + 8, laser_py - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 100, 0), 1, cv2.LINE_AA)

    # ── arrows (from laser, visiting crown pixels) ──────────────
    prev_px = laser_px
    prev_py = laser_py
    for w in path:
        curr_px = int(w["crown_px"])
        curr_py = int(w["crown_py"])
        cv2.arrowedLine(image, (prev_px, prev_py), (curr_px, curr_py),
                        (255, 200, 0), 2, tipLength=0.03)
        prev_px = curr_px
        prev_py = curr_py

    # ── weed dots and labels ────────────────────────────────────
    for w in path:
        px  = int(w["crown_px"])
        py  = int(w["crown_py"])
        num = w["visit_order"]

        rel_x   = w["x_mm"] - CAM_TO_LASER_X
        rel_y   = w["y_mm"] - CAM_TO_LASER_Y
        dist_mm = math.hypot(rel_x, rel_y)

        # red dot = detected crown position
        cv2.circle(image, (px, py), 6, (0, 0, 255), -1)

        label = f"#{num} ({rel_x:+.0f},{rel_y:+.0f})mm d={dist_mm:.0f}mm"
        cv2.putText(image, label, (px + 8, py - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

    cv2.imwrite(str(output_path), image)
    print(f"[INFO] Path image saved → {output_path}")


# ── Main ───────────────────────────────────────────────────────

def run(detection_json: str, gantry_x: float = 0.0, gantry_y: float = 0.0,
        apply_filter: bool = True, weed_height_mm: float = WEED_HEIGHT_MM):

    # 1. Load detection results saved by photo.py
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

    # 2. Load calibration (also loads poly_fit.npz if present)
    calibrations = load_all_calibrations(str(CALIB_FOLDER))
    print(f"[INFO] Calibrations loaded: {sorted(calibrations.keys())}")

    # 3. Convert pixels → mm (includes parallax correction)
    weeds_mm = convert_weeds_to_mm(weeds, z_mm, calibrations, img_w, img_h,
                                   gantry_x, gantry_y,
                                   weed_height_mm=weed_height_mm)

    # 4. Filter weeds outside gantry working area
    if apply_filter:
        weeds_mm = filter_weeds(weeds_mm)
        if not weeds_mm:
            print("[INFO] No weeds inside gantry bounds after filtering.")
    else:
        print(f"  [FILTER] Disabled — keeping all {len(weeds_mm)} weeds.")

    # 5. Build path (empty weeds_mm -> empty path, handled below rather than
    # bailing out early, so callers can always rely on path_json existing —
    # a stop with zero weeds is a normal outcome for a multi-stop row-scan)
    path = build_path(weeds_mm) if weeds_mm else []

    # 6. Print path table
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

    # 7. Save visualization
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    stem      = Path(detection_json).stem
    out_image = OUTPUT_DIR / f"{stem}_path.jpg"
    visualize_path(image_path, path, img_w, img_h, out_image,
                   calibrations, z_mm)

    # 8. Save path JSON — always, even when path is empty, so any caller
    # that reads this file back (e.g. weed_zap.py) never has to special-case
    # a missing file just because a stop/photo had no weeds in bounds.
    path_json = OUTPUT_DIR / f"{stem}_path.json"
    with open(path_json, "w") as f:
        json.dump({
            "z_mm":            z_mm,
            "gantry_x":        gantry_x,
            "gantry_y":        gantry_y,
            "cam_to_laser_x":  CAM_TO_LASER_X,
            "cam_to_laser_y":  CAM_TO_LASER_Y,
            "weed_height_mm":  weed_height_mm,
            "path":            path,
        }, f, indent=2)
    print(f"[INFO] Path data saved → {path_json}")


# ── Entry point ────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Convert weed pixels to mm, build gantry path, visualize."
    )
    parser.add_argument("--detection", required=True,
                        help="JSON file from photo.py  "
                             "e.g. runs/detections/weed1_z-9.json")
    parser.add_argument("--gantry-x", type=float, default=0.0,
                        help="Gantry X position (mm) when photo was taken.")
    parser.add_argument("--gantry-y", type=float, default=0.0,
                        help="Gantry Y position (mm) when photo was taken.")
    parser.add_argument("--weed-height", type=float, default=WEED_HEIGHT_MM,
                        help=f"Weed crown height above soil surface in mm "
                             f"(default {WEED_HEIGHT_MM}). "
                             "Corrects parallax: camera sees crown, laser targets root.")
    parser.add_argument("--no-parallax", action="store_true",
                        help="Disable parallax correction (treat plants as flat).")
    args = parser.parse_args()

    wh = 0.0 if args.no_parallax else args.weed_height
    run(args.detection, args.gantry_x, args.gantry_y,
        weed_height_mm=wh)
