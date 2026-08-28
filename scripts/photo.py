import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path
from ultralytics import YOLO
import cv2

# ── Paths ──────────────────────────────────────────────────────
SCRIPT_DIR   = Path(__file__).resolve().parent   # .../DAUREN/scripts
PROJECT_ROOT = SCRIPT_DIR.parent                 # .../DAUREN

# ── Settings ───────────────────────────────────────────────────
DEFAULT_MODEL    = PROJECT_ROOT / "model" / "last.pt"
OUTPUT_DIR       = PROJECT_ROOT / "runs" / "detections"
CAPTURE_DIR      = PROJECT_ROOT / "runs" / "captures"
KP_INDEX         = 0     # index of crown/stem keypoint in your model
KP_CONF_MIN      = 0.3   # minimum keypoint confidence to trust it
DEFAULT_CONF     = 0.30  # YOLO detection confidence threshold
DEDUP_DIST_PX    = 5     # crown points closer than this are the same weed
CAMERA_WARMUP_FRAMES = 5   # frames to discard so camera exposure stabilises

# Resolution must match the photos used for homography calibration.
# Run camera_test.py to find what your camera actually supports.
CAPTURE_WIDTH  = 1280
CAPTURE_HEIGHT = 720

import platform
if platform.system() == "Windows":
    CAMERA_BACKEND = cv2.CAP_DSHOW   # faster on Windows
else:
    CAMERA_BACKEND = cv2.CAP_V4L2    # native on Raspberry Pi / Linux

# True only when a display is available (False on headless Pi over SSH)
HAS_DISPLAY = platform.system() != "Linux" or bool(
    __import__("os").environ.get("DISPLAY") or
    __import__("os").environ.get("WAYLAND_DISPLAY")
)

def get_crown_pixel(box_xyxy, kp_xy, kp_conf):
    """
    Return best crown pixel (px, py) and how we got it.

    Tries keypoint first. Falls back to box center.
    """
    if kp_xy is not None and kp_conf is not None:
        kx = float(kp_xy[KP_INDEX][0])
        ky = float(kp_xy[KP_INDEX][1])
        kc = float(kp_conf[KP_INDEX])

        if kc >= KP_CONF_MIN and kx > 0 and ky > 0:
            return kx, ky, "keypoint"

    # fallback
    x1, y1, x2, y2 = box_xyxy
    cx = (float(x1) + float(x2)) / 2
    cy = (float(y1) + float(y2)) / 2
    return cx, cy, "bbox_center"

def deduplicate_weeds(weeds, dist_px=DEDUP_DIST_PX):
    """Remove duplicate detections whose crown pixels are within dist_px of each other.

    Sorts by confidence descending so the strongest detection is always kept.
    """
    weeds_sorted = sorted(weeds, key=lambda w: w["conf"], reverse=True)
    kept = []
    for w in weeds_sorted:
        too_close = any(
            ((w["crown_px"] - k["crown_px"]) ** 2 + (w["crown_py"] - k["crown_py"]) ** 2) ** 0.5 < dist_px
            for k in kept
        )
        if not too_close:
            kept.append(w)
    return kept


def capture_from_camera(camera_index: int = 0, preview: bool = False) -> str:
    """
    Open USB camera, warm it up, capture one frame, save and return the path.

    preview=True  — opens a window so you can see the frame before it is saved.
                    Press any key to capture, or wait 3 s for auto-capture.
    """
    print(f"[INFO] Opening camera index {camera_index} …")
    cap = cv2.VideoCapture(camera_index, CAMERA_BACKEND)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {camera_index}. "
              f"Try a different index with  --camera 1  etc.")
        sys.exit(1)

    # Force resolution to match calibration photos.
    # Change these to match whatever resolution your calibration was done at.
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAPTURE_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[INFO] Camera resolution: {actual_w}x{actual_h}")
    if actual_w != CAPTURE_WIDTH or actual_h != CAPTURE_HEIGHT:
        print(f"[WARN] Requested {CAPTURE_WIDTH}x{CAPTURE_HEIGHT} "
              f"but camera gave {actual_w}x{actual_h}. "
              f"Check CAPTURE_WIDTH/HEIGHT in photo.py.")

    # Warm up: read and discard frames so exposure/white-balance settles
    print(f"[INFO] Warming up camera ({CAMERA_WARMUP_FRAMES} frames) …")
    for _ in range(CAMERA_WARMUP_FRAMES):
        cap.read()

    if preview:
        if HAS_DISPLAY:
            print("[INFO] Preview window open — press any key to capture (auto in 3 s).")
            deadline = time.time() + 3.0
            while time.time() < deadline:
                ret, frame = cap.read()
                if not ret:
                    break
                cv2.imshow("Camera preview — press any key to capture", frame)
                if cv2.waitKey(30) & 0xFF != 255:
                    break
            cv2.destroyAllWindows()
        else:
            print("[INFO] --preview ignored: no display detected (headless mode).")

    # Capture final frame
    ret, frame = cap.read()
    cap.release()

    if not ret or frame is None:
        print("[ERROR] Failed to capture frame from camera.")
        sys.exit(1)

    # Save to captures/
    CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
    timestamp  = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_path  = CAPTURE_DIR / f"capture_{timestamp}.jpg"
    cv2.imwrite(str(save_path), frame)
    print(f"[INFO] Frame captured and saved → {save_path}")
    return str(save_path)


def run(image_path: str, z_mm: float, model_path: str, conf_thresh: float):

    # 1. Load image
    image = cv2.imread(image_path)
    if image is None:
        print(f"[ERROR] Cannot read image: {image_path}")
        sys.exit(1)

    img_h, img_w = image.shape[:2]
    print(f"[INFO] Image: {img_w}x{img_h}  Z={z_mm}mm")

    # 2. Run YOLO
    model   = YOLO(str(model_path))
    results = model(image_path, conf=conf_thresh, verbose=False)
    result  = results[0]

    boxes    = result.boxes
    kps_xy   = result.keypoints.xy   if result.keypoints is not None else None
    kps_conf = result.keypoints.conf if result.keypoints is not None else None

    if boxes is None or len(boxes) == 0:
        print("[INFO] No weeds detected.")
        return

    print(f"[INFO] Detected: {len(boxes)} weeds")

    # 3. Extract crown pixel for each weed
    weeds = []
    for i in range(len(boxes)):
        box_xyxy = boxes.xyxy[i].cpu().numpy()
        conf     = float(boxes.conf[i].cpu())
        kp_xy    = kps_xy[i].cpu().numpy()   if kps_xy   is not None else None
        kp_conf  = kps_conf[i].cpu().numpy() if kps_conf is not None else None

        crown_px, crown_py, source = get_crown_pixel(box_xyxy, kp_xy, kp_conf)

        weeds.append({
            "id":        i,
            "conf":      round(conf, 3),
            "crown_px":  round(crown_px, 1),
            "crown_py":  round(crown_py, 1),
            "source":    source,
            "box_xyxy":  [round(v, 1) for v in box_xyxy.tolist()],
        })

        print(f"  Weed {i}: pixel=({crown_px:.0f}, {crown_py:.0f})  "
              f"conf={conf:.2f}  src={source}")

    # 4. Deduplicate close detections
    before = len(weeds)
    weeds  = deduplicate_weeds(weeds)
    if len(weeds) < before:
        print(f"[INFO] Deduplicated {before} → {len(weeds)} weeds "
              f"(threshold={DEDUP_DIST_PX}px)")

    # 5. Save JSON
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    stem      = Path(image_path).stem
    json_path = OUTPUT_DIR / f"{stem}_z{int(z_mm)}.json"

    output = {
        "image":      image_path,
        "z_mm":       z_mm,
        "img_width":  img_w,
        "img_height": img_h,
        "weeds":      weeds,
    }

    with open(json_path, "w") as f:
        json.dump(output, f, indent=2)

    print(f"\n[INFO] Saved → {json_path}")
    return json_path

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    # Image source — one of --image or --camera
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--image",  metavar="PATH",
                     help="Path to a saved image file")
    src.add_argument("--camera", metavar="INDEX", type=int, nargs="?",
                     const=0,
                     help="Capture live from USB camera. "
                          "Optionally pass camera index (default 0).")

    parser.add_argument("--preview", action="store_true",
                        help="Show camera preview window before capturing "
                             "(only with --camera)")
    parser.add_argument("--z",    required=True, type=float,
                        help="Current gantry Z in mm")
    parser.add_argument("--model", default=str(DEFAULT_MODEL),
                        help="Path to YOLO weights")
    parser.add_argument("--conf",  default=DEFAULT_CONF, type=float,
                        help="YOLO confidence threshold")
    args = parser.parse_args()

    # Resolve image path
    if args.camera is not None:
        image_path = capture_from_camera(args.camera, preview=args.preview)
    else:
        image_path = args.image

    run(
        image_path  = image_path,
        z_mm        = args.z,
        model_path  = args.model,
        conf_thresh = args.conf,
    )