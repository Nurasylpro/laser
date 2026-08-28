"""
measure_laser_offset.py — measure CAM_TO_LASER_X/Y by clicking the laser
beam's actual pixel position in a photo.

Why this replaces "jog until it lines up": the camera and laser are
rigidly mounted on the same head, so jogging the gantry moves both
together and can't change their relative offset — the only way to
measure that fixed mechanical offset is to see where the beam actually
lands in the camera's own image and convert that pixel to mm using the
calibration, exactly like a weed detection would.

How to use:
    1. Home the gantry, move to a safe open spot over the bed, put
       something the beam will visibly mark (paper, cardboard) under it.
    2. Fire the laser at LOW power for a moment (by hand, e.g. via
       jog.py or the serial monitor: "laser 40" ... "laser 0") so its
       spot is visible in a photo, OR simply photograph the physical
       mark it left.
    3. Run this script against that photo and click exactly on the
       beam's spot / burn mark:

        python scripts/measure_laser_offset.py --z 90 --image runs/captures/capture_xyz.jpg

       or capture fresh from the camera while the beam is on:

        python scripts/measure_laser_offset.py --z 90 --camera 0

    4. The script prints CAM_TO_LASER_X / CAM_TO_LASER_Y and saves them to
       calibration/cam_to_laser.json. path.py automatically loads that
       file if present (overriding its hardcoded defaults), so no manual
       copy-paste into path.py is required — but the values are also
       printed for you to copy/paste anywhere else that needs them.

Math: this is the exact same pixel -> mm offset-from-frame-center formula
path.py uses for weeds (see convert_weeds_to_mm). If you clicked the
laser's own beam point, that offset IS CAM_TO_LASER_X/Y by definition —
plugging it back into path.py's  abs_x = gantry_x - CAM_TO_LASER_X + x_mm
makes abs_x/abs_y come out anchored on the BEAM's position instead of the
camera's, which is what actually matters for aiming.
"""

import argparse
import json
import sys
from pathlib import Path

import cv2

SCRIPT_DIR   = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(SCRIPT_DIR))

from calibration_loader import load_all_calibrations, pixel_to_world_at_height
import photo as photo_mod
import path  as path_mod

OUT_PATH = path_mod.CALIB_FOLDER / "cam_to_laser.json"

clicked_point = None
img_show = None


def mouse_callback(event, x, y, flags, param):
    global clicked_point, img_show
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_point = (x, y)
        img_show = param["base"].copy()
        cv2.drawMarker(img_show, (x, y), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
        cv2.circle(img_show, (x, y), 18, (0, 0, 255), 2)
        cv2.imshow("Click the laser beam point", img_show)


def main():
    global img_show, clicked_point

    parser = argparse.ArgumentParser()
    parser.add_argument("--z", required=True, type=float,
                        help="Gantry Z (mm) the photo was taken at — use the "
                             "same height you'll actually fire from (weed_zap.py's FIRE_Z_MM).")
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--image", metavar="PATH", help="Use an already-saved photo.")
    src.add_argument("--camera", type=int, nargs="?", const=0, metavar="INDEX",
                     help="Capture fresh from the camera (laser should be visibly on).")
    args = parser.parse_args()

    if args.camera is not None:
        image_path = photo_mod.capture_from_camera(args.camera, preview=False)
    else:
        image_path = args.image

    img = cv2.imread(image_path)
    if img is None:
        raise FileNotFoundError(f"Cannot open image: {image_path}")

    h, w = img.shape[:2]
    cx, cy = w / 2.0, h / 2.0

    calibrations = load_all_calibrations(str(path_mod.CALIB_FOLDER))
    print(f"[INFO] Calibrations loaded: {sorted(calibrations.keys())}")
    print(f"[INFO] Image: {w}x{h}  Z={args.z}mm\n")

    img_show = img.copy()
    cv2.drawMarker(img_show, (int(cx), int(cy)), (0, 200, 0), cv2.MARKER_CROSS, 24, 2)
    cv2.putText(img_show, "frame center", (int(cx) + 8, int(cy) - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 230, 0), 1, cv2.LINE_AA)

    cv2.namedWindow("Click the laser beam point", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Click the laser beam point", mouse_callback, {"base": img_show.copy()})

    print("Click exactly on the laser beam's spot / burn mark.")
    print("Press 'r' to reset, 'q' to accept the click and finish.\n")

    while True:
        cv2.imshow("Click the laser beam point", img_show)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('r'):
            clicked_point = None
            img_show = img.copy()
            cv2.drawMarker(img_show, (int(cx), int(cy)), (0, 200, 0), cv2.MARKER_CROSS, 24, 2)
            print("Reset.")

        if key == ord('q'):
            if clicked_point is not None:
                break
            print("Click the beam point first.")

    cv2.destroyAllWindows()

    px, py = clicked_point

    cx_raw, cy_raw = pixel_to_world_at_height((cx, cy), args.z, calibrations)
    wx, wy = pixel_to_world_at_height((px, py), args.z, calibrations)

    # Same axis-swap formula as path.py's convert_weeds_to_mm — no parallax
    # correction here, since the beam hits the actual ground plane the
    # homography was calibrated against (weed_height_mm=0 equivalent).
    cam_to_laser_x = (wy - cy_raw)
    cam_to_laser_y = -(wx - cx_raw)

    print(f"\n{'='*45}")
    print("  Measured offset (camera center -> laser beam):")
    print(f"  CAM_TO_LASER_X = {cam_to_laser_x:.2f}")
    print(f"  CAM_TO_LASER_Y = {cam_to_laser_y:.2f}")
    print(f"{'='*45}")
    print(f"  Copy these into path.py if you'd rather set them by hand.")
    print(f"  Also auto-saved to {OUT_PATH} — path.py loads it automatically.")

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(OUT_PATH, "w") as f:
        json.dump({
            "cam_to_laser_x": round(cam_to_laser_x, 2),
            "cam_to_laser_y": round(cam_to_laser_y, 2),
            "measured_at_z_mm": args.z,
            "measured_from_image": str(image_path),
        }, f, indent=2)
    print(f"[OK] Saved -> {OUT_PATH}")


if __name__ == "__main__":
    main()
