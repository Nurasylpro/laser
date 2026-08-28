"""
validate_click_points.py — click any point, see its real-world mm position
                           relative to the LASER BEAM point (not camera center)

Usage:
    python scripts/validate_click_points.py --camera 0 --z 50
    python scripts/validate_click_points.py --image data/raw/photo.jpg --z 50

How to validate accuracy:
    1. Home gantry  (position 0, 0)
    2. Place a ruler on the ground with 0mm at the laser beam point
    3. Run with --camera to capture fresh, or --image for existing photo
    4. Click known points (e.g. 50mm, 100mm marks on ruler)
    5. Compare what the script reports vs what the ruler shows
"""

import argparse
import sys
import math
import cv2

sys.path.insert(0, 'scripts')
from calibration_loader import load_all_calibrations, pixel_to_world_at_height
import path  as path_mod
import photo as photo_mod

# ── ARGS ───────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
src = parser.add_mutually_exclusive_group(required=True)
src.add_argument("--camera", metavar="INDEX", type=int, nargs="?", const=0,
                 help="Capture from USB camera (default index 0)")
src.add_argument("--image", metavar="PATH",
                 help="Use an existing image file")
parser.add_argument("--z", required=True, type=float,
                    help="Z height when photo is/was taken (mm)")
parser.add_argument("--gantry-x", type=float, default=0.0,
                    help="Gantry X when photo taken (default 0 = home)")
parser.add_argument("--gantry-y", type=float, default=0.0,
                    help="Gantry Y when photo taken (default 0 = home)")
args = parser.parse_args()

Z_MM     = args.z
GANTRY_X = args.gantry_x
GANTRY_Y = args.gantry_y

# CAM_TO_LASER comes from path.py — change it there, updates here automatically
CAM_TO_LASER_X = path_mod.CAM_TO_LASER_X
CAM_TO_LASER_Y = path_mod.CAM_TO_LASER_Y

# ── LOAD IMAGE ─────────────────────────────────────────────────
if args.camera is not None:
    print(f"[INFO] Capturing from camera {args.camera} …")
    IMAGE_PATH = photo_mod.capture_from_camera(args.camera, preview=False)
else:
    IMAGE_PATH = args.image

calibrations = load_all_calibrations('calibration2')
image        = cv2.imread(IMAGE_PATH)
if image is None:
    print(f"[ERROR] Cannot open image: {IMAGE_PATH}")
    sys.exit(1)

img_h, img_w = image.shape[:2]
cx_px = img_w / 2.0
cy_px = img_h / 2.0

# Frame center in raw world mm (homography output)
cx_raw, cy_raw = pixel_to_world_at_height((cx_px, cy_px), Z_MM, calibrations)

# mm-per-pixel scale at centre (for drawing laser marker on image)
# Horizontal pixel -> gantry Y mm/px; vertical pixel -> gantry X mm/px
# (matches path.py / weed_zap.py's Visualizer / test_laser_point.py)
p1    = pixel_to_world_at_height((cx_px + 1, cy_px), Z_MM, calibrations)
p2    = pixel_to_world_at_height((cx_px, cy_px + 1), Z_MM, calibrations)
mpp_h = p1[0] - cx_raw   # mm per horizontal pixel (gantry Y direction)
mpp_v = p2[1] - cy_raw   # mm per vertical pixel   (gantry X direction)

# Laser beam pixel position on the image
laser_px = int(cx_px - CAM_TO_LASER_Y / mpp_h)
laser_py = int(cy_px - CAM_TO_LASER_X / mpp_v)

print(f"[INFO] Image {img_w}x{img_h}  Z={Z_MM}mm  "
      f"CAM_TO_LASER=({CAM_TO_LASER_X}, {CAM_TO_LASER_Y})")
print(f"[INFO] Camera center pixel: ({int(cx_px)}, {int(cy_px)})")
print(f"[INFO] Laser beam pixel:    ({laser_px}, {laser_py})")
print()
print("Click any point on the image.")
print("Coordinates shown are relative to the LASER BEAM position.")
print("Press Q to quit.")
print()
print(f"  {'Pixel':^20}  {'From laser (mm)':^22}  {'Abs gantry target':^22}  Dist")
print(f"  {'-'*85}")

# ── DRAW PERMANENT MARKERS ─────────────────────────────────────
display = image.copy()

# camera center — green
cv2.drawMarker(display, (int(cx_px), int(cy_px)),
               (0, 200, 0), cv2.MARKER_CROSS, 20, 2)
cv2.putText(display, "camera", (int(cx_px) + 8, int(cy_px) - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 0), 1)

# laser beam point — blue/orange
cv2.drawMarker(display, (laser_px, laser_py),
               (255, 120, 0), cv2.MARKER_CROSS, 24, 2)
cv2.circle(display, (laser_px, laser_py), 16, (255, 120, 0), 2)
cv2.putText(display, "laser (0,0)", (laser_px + 10, laser_py - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 160, 0), 1)


# ── CLICK HANDLER ──────────────────────────────────────────────
def on_click(event, x, y, flags, param):
    if event != cv2.EVENT_LBUTTONDOWN:
        return

    # Raw world coords from homography
    wx, wy = pixel_to_world_at_height((x, y), Z_MM, calibrations)

    # X = forward/backward (away from home = +X)
    # Y = left/right       (left = +Y)
    # wx/wy are swapped relative to gantry axes — corrected here (same as path.py)
    # X sign confirmed empirically 2026-08-02 (see path.py's convert_weeds_to_mm)
    x_from_cam = (wy - cy_raw)
    y_from_cam = -(wx - cx_raw)

    # Offset from laser beam point
    x_from_laser = x_from_cam - CAM_TO_LASER_X
    y_from_laser = y_from_cam - CAM_TO_LASER_Y

    # Absolute gantry target (where gantry must go to put laser here)
    abs_x = GANTRY_X + x_from_cam - CAM_TO_LASER_X  # same as path.py
    abs_y = GANTRY_Y + y_from_cam - CAM_TO_LASER_Y

    dist = math.hypot(x_from_laser, y_from_laser)

    print(f"  ({x:4d}, {y:4d})        "
          f"({x_from_laser:+7.1f}, {y_from_laser:+7.1f})       "
          f"abs({abs_x:+7.1f}, {abs_y:+7.1f})    "
          f"{dist:6.1f} mm")

    # Draw dot and label on image
    cv2.circle(display, (x, y), 5, (0, 0, 255), -1)
    label = f"({x_from_laser:+.1f}, {y_from_laser:+.1f})mm"
    cv2.putText(display, label, (x + 8, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 80, 255), 1)

    # Line from laser to clicked point
    cv2.line(display, (laser_px, laser_py), (x, y),
             (100, 100, 255), 1, cv2.LINE_AA)


# ── WINDOW ─────────────────────────────────────────────────────
WIN = "Validate - distance from laser"
cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WIN, 1280, 720)
cv2.setMouseCallback(WIN, on_click)

while True:
    cv2.imshow(WIN, display)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
