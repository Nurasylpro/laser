"""
log_fire_error.py — measure and log the real beam-to-crown error after a
test fire, to find the actual pattern (grows with distance from frame
center? consistent direction? random?) instead of guessing.

How to use, per test weed:
    1. Fire the laser at a weed as usual (leaves a visible mark/scorch).
    2. Take (or reuse) a photo showing BOTH the weed's crown and the burn
       mark, at the same Z the pipeline fires from (FIRE_Z_MM, 90 by
       default).
    3. Run:
        python scripts/log_fire_error.py --z 90 --image runs/captures/xyz.jpg
    4. Click the CROWN first, then the BURN MARK. Press 'q' when both are
       placed ('r' to reset).

Each run appends one row to runs/review/fire_error_log.csv:
    image, crown_px, crown_py, beam_px, beam_py,
    dist_from_center_px (of the crown — for checking lens-distortion
        correlation: does error grow with this?),
    error_x_mm, error_y_mm, error_mag_mm (in gantry X/Y — same axis-swap
        convention as path.py, so error_x_mm/error_y_mm are directly
        comparable to abs_x/abs_y bias)

After a handful of test fires, run with --summary to see whether error
magnitude correlates with distance from center (-> lens distortion,
uncorrected by the plain 4-point homography) or is roughly constant
everywhere (-> a calibration/offset bias instead), and whether the error
vector points the same direction each time (systematic, fixable by
nudging CAM_TO_LASER) or scatters (imprecision/backlash/noise).
"""

import argparse
import csv
import math
import sys
from pathlib import Path

import cv2

SCRIPT_DIR   = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(SCRIPT_DIR))

from calibration_loader import load_all_calibrations, pixel_to_world_at_height
import photo as photo_mod
import path  as path_mod

LOG_PATH = PROJECT_ROOT / "runs" / "review" / "fire_error_log.csv"

clicks = []   # [(crown_px, crown_py)] then [..., (beam_px, beam_py)]
img_show = None
LABELS = ["CROWN", "BURN MARK"]


def mouse_callback(event, x, y, flags, param):
    global clicks, img_show
    if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < 2:
        clicks.append((x, y))
        color = (0, 0, 255) if len(clicks) == 1 else (255, 120, 0)
        cv2.drawMarker(img_show, (x, y), color, cv2.MARKER_CROSS, 22, 2)
        cv2.putText(img_show, LABELS[len(clicks) - 1], (x + 10, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
        cv2.imshow(WIN, img_show)


WIN = "Click CROWN then BURN MARK  (r=reset, q=done)"


def run(image_path: str, z_mm: float):
    global img_show, clicks

    img = cv2.imread(image_path)
    if img is None:
        raise FileNotFoundError(f"Cannot open image: {image_path}")

    h, w = img.shape[:2]
    cx, cy = w / 2.0, h / 2.0

    calibrations = load_all_calibrations(str(path_mod.CALIB_FOLDER))

    img_show = img.copy()
    cv2.drawMarker(img_show, (int(cx), int(cy)), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WIN, mouse_callback)

    print("Click 1) the weed's CROWN, then 2) the BURN MARK left by the beam.")
    print("'r' to reset, 'q' once both are placed.\n")

    while True:
        cv2.imshow(WIN, img_show)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('r'):
            clicks = []
            img_show = img.copy()
            cv2.drawMarker(img_show, (int(cx), int(cy)), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
            print("Reset.")
        if key == ord('q'):
            if len(clicks) == 2:
                break
            print(f"Need both points, have {len(clicks)}.")

    cv2.destroyAllWindows()

    (crown_px, crown_py), (beam_px, beam_py) = clicks

    cx_raw, cy_raw = pixel_to_world_at_height((cx, cy), z_mm, calibrations)
    crown_wx, crown_wy = pixel_to_world_at_height((crown_px, crown_py), z_mm, calibrations)
    beam_wx,  beam_wy  = pixel_to_world_at_height((beam_px,  beam_py),  z_mm, calibrations)

    # Same axis-swap convention as path.py's convert_weeds_to_mm, so these
    # are directly comparable to abs_x/abs_y bias.
    crown_x_mm = crown_wy - cy_raw
    crown_y_mm = -(crown_wx - cx_raw)
    beam_x_mm  = beam_wy - cy_raw
    beam_y_mm  = -(beam_wx - cx_raw)

    error_x_mm = crown_x_mm - beam_x_mm
    error_y_mm = crown_y_mm - beam_y_mm
    error_mag_mm = math.hypot(error_x_mm, error_y_mm)

    dist_from_center_px = math.hypot(crown_px - cx, crown_py - cy)

    print(f"\n{'='*50}")
    print(f"  Crown pixel: ({crown_px}, {crown_py})   dist from center: {dist_from_center_px:.0f}px")
    print(f"  Beam  pixel: ({beam_px}, {beam_py})")
    print(f"  Error (crown - beam):  X={error_x_mm:+.1f}mm  Y={error_y_mm:+.1f}mm  "
          f"|error|={error_mag_mm:.1f}mm")
    print(f"{'='*50}")

    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    is_new = not LOG_PATH.exists()
    with open(LOG_PATH, "a", newline="") as f:
        writer = csv.writer(f)
        if is_new:
            writer.writerow(["image", "z_mm", "crown_px", "crown_py", "beam_px", "beam_py",
                             "dist_from_center_px", "error_x_mm", "error_y_mm", "error_mag_mm"])
        writer.writerow([image_path, z_mm, crown_px, crown_py, beam_px, beam_py,
                         round(dist_from_center_px, 1), round(error_x_mm, 2),
                         round(error_y_mm, 2), round(error_mag_mm, 2)])
    print(f"[OK] Logged -> {LOG_PATH}")


def summary():
    if not LOG_PATH.exists():
        print(f"No log yet at {LOG_PATH} — run a few measurements first.")
        return

    with open(LOG_PATH) as f:
        rows = list(csv.DictReader(f))

    if not rows:
        print("Log is empty.")
        return

    n = len(rows)
    mags  = [float(r["error_mag_mm"]) for r in rows]
    dists = [float(r["dist_from_center_px"]) for r in rows]
    exs   = [float(r["error_x_mm"]) for r in rows]
    eys   = [float(r["error_y_mm"]) for r in rows]

    mean_mag  = sum(mags) / n
    mean_ex   = sum(exs) / n
    mean_ey   = sum(eys) / n

    # crude correlation between radial pixel distance and error magnitude
    def corr(a, b):
        n = len(a)
        ma, mb = sum(a) / n, sum(b) / n
        num = sum((x - ma) * (y - mb) for x, y in zip(a, b))
        da  = math.sqrt(sum((x - ma) ** 2 for x in a))
        db  = math.sqrt(sum((y - mb) ** 2 for y in b))
        return num / (da * db) if da > 0 and db > 0 else 0.0

    r = corr(dists, mags)

    print(f"\n{'='*60}")
    print(f"  {n} measurement(s) logged")
    print(f"  Mean |error|:          {mean_mag:.1f} mm")
    print(f"  Mean error vector:     X={mean_ex:+.1f}mm  Y={mean_ey:+.1f}mm")
    print(f"  Correlation(radial px distance, |error|):  r={r:+.2f}")
    print(f"{'='*60}")
    if r > 0.5:
        print("  -> Error grows with distance from frame center: likely UNCORRECTED "
              "LENS DISTORTION. Fix: add lens distortion correction to calibration, "
              "or keep weeds closer to frame center during the tour.")
    elif abs(mean_ex) > 5 or abs(mean_ey) > 5:
        print("  -> Consistent directional bias regardless of position: likely a "
              "CAM_TO_LASER offset that's slightly off. Fix: re-run "
              "measure_laser_offset.py more carefully, or manually nudge "
              f"CAM_TO_LASER_X by {mean_ex:+.1f} and CAM_TO_LASER_Y by {mean_ey:+.1f} "
              "in calibration/cam_to_laser.json.")
    else:
        print("  -> No clear pattern yet (low radial correlation, small mean bias): "
              "likely click/measurement noise, backlash, or per-weed height variation. "
              "Log more samples, or check mechanical repeatability.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--summary", action="store_true",
                        help="Print aggregate stats from the existing log instead of "
                             "logging a new measurement.")
    parser.add_argument("--z", type=float, help="Z the photo was taken at (mm)")
    src = parser.add_mutually_exclusive_group()
    src.add_argument("--image", metavar="PATH")
    src.add_argument("--camera", type=int, nargs="?", const=0, metavar="INDEX")
    args = parser.parse_args()

    if args.summary:
        summary()
        sys.exit(0)

    if args.z is None or (args.image is None and args.camera is None):
        parser.error("--z and one of --image/--camera are required unless --summary is given")

    if args.camera is not None:
        image_path = photo_mod.capture_from_camera(args.camera, preview=False)
    else:
        image_path = args.image

    run(image_path, args.z)
