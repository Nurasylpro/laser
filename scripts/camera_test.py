"""
camera_test.py — find your camera index and test live preview

Run:
    python scripts/camera_test.py

Shows a live window for each camera found (index 0, 1, 2 ...).
Press Q to skip to next camera, or just close the window.
"""

import os
import platform
import sys
import cv2

sys.path.insert(0, 'scripts')
from photo import CAPTURE_WIDTH, CAPTURE_HEIGHT, CAMERA_BACKEND

HAS_DISPLAY = platform.system() != "Linux" or bool(
    os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
)

def test_camera(index):
    print(f"  Trying camera {index} … ", end="", flush=True)
    cap = cv2.VideoCapture(index, CAMERA_BACKEND)
    if not cap.isOpened():
        print("not found.")
        cap.release()
        return False

    # Check default resolution first
    ret, frame = cap.read()
    if not ret or frame is None:
        print("opened but no frame.")
        cap.release()
        return False
    default_h, default_w = frame.shape[:2]

    # Now request the target resolution (must match calibration photos)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAPTURE_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
    ret, frame = cap.read()
    actual_h, actual_w = frame.shape[:2]

    match = "OK" if (actual_w == CAPTURE_WIDTH and actual_h == CAPTURE_HEIGHT) else "MISMATCH"
    print(f"found.")
    print(f"    Default resolution : {default_w}x{default_h}")
    print(f"    Requested          : {CAPTURE_WIDTH}x{CAPTURE_HEIGHT}")
    print(f"    Got                : {actual_w}x{actual_h}  [{match}]")
    if match == "MISMATCH":
        print(f"    !! Camera cannot do {CAPTURE_WIDTH}x{CAPTURE_HEIGHT}.")
        print(f"       Change CAPTURE_WIDTH/HEIGHT in photo.py to {actual_w}x{actual_h}")
        print(f"       and redo calibration photos at that resolution.")

    # Live preview at target resolution
    if HAS_DISPLAY:
        print(f"    → Preview at {actual_w}x{actual_h}. Press Q to go to next.")
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            cv2.imshow(f"Camera {index} - {actual_w}x{actual_h}", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()
    else:
        print(f"    → Headless: saving snapshot to camera_{index}_test.jpg")
        ret, frame = cap.read()
        if ret:
            cv2.imwrite(f"camera_{index}_test.jpg", frame)

    cap.release()
    return True


print(f"Scanning cameras 0-4  (target resolution: {CAPTURE_WIDTH}x{CAPTURE_HEIGHT}) …\n")
found = []
for i in range(5):
    if test_camera(i):
        found.append(i)

print()
if found:
    print(f"Cameras found at index(es): {found}")
    print(f"Use: python scripts/photo.py --camera {found[0]} --z 50")
else:
    print("No cameras found. Check USB connection.")
