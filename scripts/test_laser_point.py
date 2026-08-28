"""
test_laser_point.py — visually verify CAM_TO_LASER_X/Y offset on a real image

Run (capture fresh from USB camera — same as weed_zap.py):
    python scripts/test_laser_point.py --camera 0 --z 50

Run (use an already-saved image):
    python scripts/test_laser_point.py --image runs/captures/capture_xyz.jpg --z 50

Shows:
    GREEN  cross = camera center (pixel 640, 360)
    BLUE   cross = laser beam point calculated from CAM_TO_LASER values
    Labels show the pixel coordinates so you can verify they moved

Use +/- keys to nudge the offset live:
    X key + arrow left/right  → adjust CAM_TO_LASER_X
    Y key + arrow left/right  → adjust CAM_TO_LASER_Y
    Q                         → quit and print final values
"""

import argparse
import sys
from pathlib import Path

import cv2

SCRIPT_DIR   = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(SCRIPT_DIR))

from calibration_loader import load_all_calibrations, pixel_to_world_at_height
import path  as path_mod
import photo as photo_mod

CALIB_FOLDER = PROJECT_ROOT / "calibration2"
STEP         = 1.0    # mm per key press


def compute_laser_pixel(cx, cy, mppx, mppy, cam_x, cam_y):
    """Convert CAM_TO_LASER mm offset → pixel position on the image.

    mppx = wx change per horizontal pixel → gantry Y mm/px
    mppy = wy change per vertical   pixel → gantry X mm/px

    Therefore:
      cam_y (gantry Y, left/right)    drives horizontal pixel offset (lx)
      cam_x (gantry X, forward/back)  drives vertical   pixel offset (ly)
    """
    lx = int(cx - cam_y / mppx)   # Y offset (left/right)    → horizontal px
    ly = int(cy - cam_x / mppy)   # X offset (forward/back)  → vertical   px
    return lx, ly


def draw_frame(base_img, cx, cy, mppx, mppy, cam_x, cam_y):
    frame = base_img.copy()
    h, w  = frame.shape[:2]

    lx, ly = compute_laser_pixel(cx, cy, mppx, mppy, cam_x, cam_y)

    # ── camera center ──────────────────────────────────────────
    cv2.drawMarker(frame, (int(cx), int(cy)), (0, 200, 0),
                   cv2.MARKER_CROSS, 30, 2)
    cv2.putText(frame, f"camera center ({int(cx)}, {int(cy)})",
                (int(cx) + 10, int(cy) - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 230, 0), 1, cv2.LINE_AA)

    # ── laser beam point ───────────────────────────────────────
    in_frame = (0 <= lx < w) and (0 <= ly < h)
    color    = (255, 120, 0) if in_frame else (0, 0, 255)

    if in_frame:
        cv2.drawMarker(frame, (lx, ly), color, cv2.MARKER_CROSS, 30, 2)
        cv2.circle(frame, (lx, ly), 18, color, 2)
    else:
        # Clamp to frame edge and draw warning
        lx_c = max(5, min(w - 5, lx))
        ly_c = max(5, min(h - 5, ly))
        cv2.drawMarker(frame, (lx_c, ly_c), color, cv2.MARKER_CROSS, 30, 2)

    cv2.putText(frame, f"laser ({lx}, {ly})" + ("" if in_frame else "  OUT OF FRAME"),
                (max(0, lx + 10) if in_frame else 10, max(20, ly - 10) if in_frame else 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    # ── line connecting camera to laser ───────────────────────
    if in_frame:
        cv2.line(frame, (int(cx), int(cy)), (lx, ly), (180, 180, 180), 1,
                 cv2.LINE_AA)

    # ── info panel (top left) ─────────────────────────────────
    panel = [
        f"CAM_TO_LASER_X = {cam_x:+.2f} mm",
        f"CAM_TO_LASER_Y = {cam_y:+.2f} mm",
        f"mppx = {mppx:.4f} mm/px   mppy = {mppy:.4f} mm/px",
        f"laser pixel: ({lx}, {ly})",
        "",
        "Keys:  LEFT/RIGHT = adjust Y (left/right)   UP/DOWN = adjust X (forward/back)",
        "       SHIFT = fine step (0.1mm)   Q = quit",
    ]
    cv2.rectangle(frame, (0, 0), (520, 20 + 22 * len(panel)), (20, 20, 20), -1)
    for i, line in enumerate(panel):
        cv2.putText(frame, line, (8, 20 + 22 * i),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                    (220, 220, 220), 1, cv2.LINE_AA)

    return frame


def run(image_path: str, z_mm: float):
    img = cv2.imread(image_path)
    if img is None:
        print(f"[ERROR] Cannot open: {image_path}")
        sys.exit(1)

    h, w    = img.shape[:2]
    cx, cy  = w / 2.0, h / 2.0
    print(f"[INFO] Image: {w}×{h}  Z={z_mm}mm")

    calibrations = load_all_calibrations(str(CALIB_FOLDER))
    print(f"[INFO] Calibrations: {sorted(calibrations.keys())}")

    # mm-per-pixel at image centre
    # p1: step one pixel RIGHT  → wx changes → gantry Y mm/px  (mppx)
    # p2: step one pixel DOWN   → wy changes → gantry X mm/px  (mppy)
    p0   = pixel_to_world_at_height((cx,     cy),     z_mm, calibrations)
    p1   = pixel_to_world_at_height((cx + 1, cy),     z_mm, calibrations)
    p2   = pixel_to_world_at_height((cx,     cy + 1), z_mm, calibrations)
    mppx = p1[0] - p0[0]   # gantry Y mm per horizontal pixel
    mppy = p2[1] - p0[1]   # gantry X mm per vertical   pixel
    print(f"[INFO] mppx(Y/px)={mppx:.4f}  mppy(X/px)={mppy:.4f}")

    cam_x = path_mod.CAM_TO_LASER_X
    cam_y = path_mod.CAM_TO_LASER_Y
    print(f"[INFO] Starting CAM_TO_LASER  X={cam_x}  Y={cam_y}")

    WIN = "Laser point test — Q to quit"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, min(w, 1280), min(h, 720))

    step = STEP

    while True:
        frame = draw_frame(img, cx, cy, mppx, mppy, cam_x, cam_y)

        # proportional resize for display
        scale         = min(1280 / w, 720 / h)
        display_frame = cv2.resize(frame, (int(w * scale), int(h * scale)))
        cv2.imshow(WIN, display_frame)

        key = cv2.waitKey(50) & 0xFF

        if key == ord('q') or key == 27:
            break
        elif key == 81 or key == ord('a'):   # left arrow  → Y axis (left/right)
            cam_y -= step
        elif key == 83 or key == ord('d'):   # right arrow → Y axis (left/right)
            cam_y += step
        elif key == 82 or key == ord('w'):   # up arrow    → X axis (forward/back)
            cam_x -= step
        elif key == 84 or key == ord('s'):   # down arrow  → X axis (forward/back)
            cam_x += step
        elif key == ord('f'):                # fine / coarse toggle
            step = 0.1 if step == STEP else STEP
            print(f"  Step size: {step} mm")

    cv2.destroyAllWindows()

    print(f"\n{'='*45}")
    print(f"  Final values — copy these into path.py:")
    print(f"  CAM_TO_LASER_X = {cam_x:.2f}")
    print(f"  CAM_TO_LASER_Y = {cam_y:.2f}")
    print(f"{'='*45}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Visually verify and adjust CAM_TO_LASER offset on a real image."
    )

    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--camera", metavar="INDEX", type=int, nargs="?", const=0,
                     help="Capture from USB camera (same as weed_zap.py). "
                          "Default index 0.")
    src.add_argument("--image", metavar="PATH",
                     help="Use an already-saved image file.")

    parser.add_argument("--z", required=True, type=float,
                        help="Z height when the image was taken (mm)")
    args = parser.parse_args()

    if args.camera is not None:
        print(f"[INFO] Capturing from camera {args.camera} (same code as weed_zap.py) …")
        image_path = photo_mod.capture_from_camera(args.camera, preview=False)
    else:
        image_path = args.image

    run(image_path, args.z)
