"""
WeedZap calibration (from scratch, 2026-08-03).

Captures / loads a photo of an A4 sheet lying flat on the bed at a known
gantry Z height, lets you click its 4 corners, computes the pixel -> mm
homography, and saves it to ../calibration/z_<Z>.npz.

Run once for Z=0 and once for Z=90 (the two heights being recalibrated
right now):

    python calibrate.py --z 0
    python calibrate.py --z 90

Or, to calibrate from an already-saved photo instead of the live camera:

    python calibrate.py --z 0 --image path\to\photo.jpg
"""

import argparse
import platform
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# ========= EDIT THESE FOR YOUR A4 SHEET =========
A4_WIDTH_MM  = 297.0
A4_HEIGHT_MM = 210.0
# ==================================================

SCRIPT_DIR      = Path(__file__).resolve().parent
CALIBRATION_DIR = SCRIPT_DIR.parent / "calibration"

CAPTURE_WIDTH  = 1280
CAPTURE_HEIGHT = 720
CAMERA_WARMUP_FRAMES = 5

CAMERA_BACKEND = cv2.CAP_DSHOW if platform.system() == "Windows" else cv2.CAP_V4L2

# Click order — must match this exactly:
CORNER_LABELS = ["top-left", "top-right", "bottom-right", "bottom-left"]

clicked_points = []
img_show = None


def mouse_callback(event, x, y, flags, param):
    global clicked_points, img_show
    if event == cv2.EVENT_LBUTTONDOWN and len(clicked_points) < 4:
        clicked_points.append([x, y])
        cv2.circle(img_show, (x, y), 6, (0, 0, 255), -1)
        cv2.putText(
            img_show,
            f"{len(clicked_points)}:{CORNER_LABELS[len(clicked_points) - 1]}",
            (x + 10, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2,
        )
        cv2.imshow("Click A4 corners", img_show)


def capture_from_camera(camera_index: int) -> np.ndarray:
    print(f"[INFO] Opening camera index {camera_index} ...")
    cap = cv2.VideoCapture(camera_index, CAMERA_BACKEND)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {camera_index}.")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[INFO] Camera resolution: {actual_w}x{actual_h}")

    for _ in range(CAMERA_WARMUP_FRAMES):
        cap.read()

    ret, frame = cap.read()
    cap.release()

    if not ret or frame is None:
        print("[ERROR] Failed to capture frame from camera.")
        sys.exit(1)

    return frame


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--z", required=True, type=float,
                         help="Gantry Z height in mm this photo was taken at (e.g. 0 or 90)")
    parser.add_argument("--image", default=None,
                         help="Use an existing photo instead of the live camera")
    parser.add_argument("--camera", type=int, default=0,
                         help="Camera index (default 0)")
    args = parser.parse_args()

    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            raise FileNotFoundError(f"Cannot open image: {args.image}")
    else:
        img = capture_from_camera(args.camera)

    global img_show, clicked_points
    img_show = img.copy()
    clicked_points = []

    cv2.namedWindow("Click A4 corners", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Click A4 corners", mouse_callback)

    print(f"\nZ = {args.z} mm")
    print("Click the 4 A4 corners in this exact order:")
    for i, label in enumerate(CORNER_LABELS, 1):
        print(f"  {i}) {label}")
    print("Press 'r' to reset, 'q' to finish once 4 points are clicked.\n")

    while True:
        cv2.imshow("Click A4 corners", img_show)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('r'):
            clicked_points = []
            img_show = img.copy()
            print("Reset.")

        if key == ord('q'):
            if len(clicked_points) == 4:
                break
            print(f"Need exactly 4 points, have {len(clicked_points)}.")

    cv2.destroyAllWindows()

    img_pts = np.array(clicked_points, dtype=np.float32)

    world_pts = np.array([
        [0, 0],                          # top-left
        [A4_WIDTH_MM, 0],                # top-right
        [A4_WIDTH_MM, A4_HEIGHT_MM],     # bottom-right
        [0, A4_HEIGHT_MM],               # bottom-left
    ], dtype=np.float32)

    H = cv2.getPerspectiveTransform(img_pts, world_pts)

    print("\nHomography matrix (pixel -> mm):")
    print(H)

    CALIBRATION_DIR.mkdir(parents=True, exist_ok=True)
    out_path = CALIBRATION_DIR / f"z_{int(args.z)}.npz"

    np.savez(
        out_path,
        H=H,
        img_pts=img_pts,
        world_pts=world_pts,
        z_mm=args.z,
        a4_width_mm=A4_WIDTH_MM,
        a4_height_mm=A4_HEIGHT_MM,
    )

    print(f"\n[OK] Saved calibration -> {out_path}")


if __name__ == "__main__":
    main()
