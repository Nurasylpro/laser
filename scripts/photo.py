"""
photo.py — capture a frame (or use a saved image), run YOLO weed detection,
extract each weed's crown pixel, save a detection JSON.
"""

import json
import platform
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
from ultralytics import YOLO

SCRIPT_DIR   = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent

DEFAULT_MODEL = PROJECT_ROOT / "model" / "best3.pt"
OUTPUT_DIR    = PROJECT_ROOT / "runs" / "detections"
CAPTURE_DIR   = PROJECT_ROOT / "runs" / "captures"

KP_INDEX      = 0     # index of crown/stem keypoint in the model
KP_CONF_MIN   = 0.3   # minimum keypoint confidence to trust it
DEFAULT_CONF  = 0.30
DEDUP_DIST_PX = 5      # crown points closer than this, within one photo, are the same weed

CAMERA_WARMUP_FRAMES = 5
CAPTURE_WIDTH  = 1280
CAPTURE_HEIGHT = 720

CAMERA_BACKEND = cv2.CAP_DSHOW if platform.system() == "Windows" else cv2.CAP_V4L2


def get_crown_pixel(box_xyxy, kp_xy, kp_conf):
    """Best crown pixel (px, py) — keypoint first, bbox-center fallback."""
    if kp_xy is not None and kp_conf is not None:
        kx = float(kp_xy[KP_INDEX][0])
        ky = float(kp_xy[KP_INDEX][1])
        kc = float(kp_conf[KP_INDEX])
        if kc >= KP_CONF_MIN and kx > 0 and ky > 0:
            return kx, ky, "keypoint"

    x1, y1, x2, y2 = box_xyxy
    cx = (float(x1) + float(x2)) / 2
    cy = (float(y1) + float(y2)) / 2
    return cx, cy, "bbox_center"


def deduplicate_weeds(weeds, dist_px=DEDUP_DIST_PX):
    """Remove duplicate detections within one photo whose crown pixels are close."""
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
    """Open USB camera, warm it up, capture one frame, save and return the path."""
    print(f"[INFO] Opening camera index {camera_index} ...")
    cap = cv2.VideoCapture(camera_index, CAMERA_BACKEND)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {camera_index}.")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAPTURE_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[INFO] Camera resolution: {actual_w}x{actual_h}")
    if actual_w != CAPTURE_WIDTH or actual_h != CAPTURE_HEIGHT:
        print(f"[WARN] Requested {CAPTURE_WIDTH}x{CAPTURE_HEIGHT} but got {actual_w}x{actual_h}.")

    for _ in range(CAMERA_WARMUP_FRAMES):
        cap.read()

    if preview:
        deadline = time.time() + 3.0
        while time.time() < deadline:
            ret, frame = cap.read()
            if not ret:
                break
            cv2.imshow("Camera preview - press any key to capture", frame)
            if cv2.waitKey(30) & 0xFF != 255:
                break
        cv2.destroyAllWindows()

    ret, frame = cap.read()
    cap.release()

    if not ret or frame is None:
        print("[ERROR] Failed to capture frame from camera.")
        sys.exit(1)

    CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    save_path = CAPTURE_DIR / f"capture_{timestamp}.jpg"
    cv2.imwrite(str(save_path), frame)
    print(f"[INFO] Frame captured -> {save_path}")
    return str(save_path)


def run(image_path: str, z_mm: float, model_path: str = str(DEFAULT_MODEL),
        conf_thresh: float = DEFAULT_CONF):
    image = cv2.imread(image_path)
    if image is None:
        print(f"[ERROR] Cannot read image: {image_path}")
        sys.exit(1)

    img_h, img_w = image.shape[:2]
    print(f"[INFO] Image: {img_w}x{img_h}  Z={z_mm}mm")

    model   = YOLO(str(model_path))
    results = model(image_path, conf=conf_thresh, verbose=False)
    result  = results[0]

    boxes    = result.boxes
    kps_xy   = result.keypoints.xy   if result.keypoints is not None else None
    kps_conf = result.keypoints.conf if result.keypoints is not None else None

    if boxes is None or len(boxes) == 0:
        print("[INFO] No weeds detected.")
        return None

    print(f"[INFO] Detected: {len(boxes)} weeds")

    weeds = []
    for i in range(len(boxes)):
        box_xyxy = boxes.xyxy[i].cpu().numpy()
        conf     = float(boxes.conf[i].cpu())
        kp_xy    = kps_xy[i].cpu().numpy()   if kps_xy   is not None else None
        kp_conf  = kps_conf[i].cpu().numpy() if kps_conf is not None else None

        crown_px, crown_py, source = get_crown_pixel(box_xyxy, kp_xy, kp_conf)

        weeds.append({
            "id":       i,
            "conf":     round(conf, 3),
            "crown_px": round(crown_px, 1),
            "crown_py": round(crown_py, 1),
            "source":   source,
            "box_xyxy": [round(v, 1) for v in box_xyxy.tolist()],
        })

        print(f"  Weed {i}: pixel=({crown_px:.0f}, {crown_py:.0f})  conf={conf:.2f}  src={source}")

    before = len(weeds)
    weeds  = deduplicate_weeds(weeds)
    if len(weeds) < before:
        print(f"[INFO] Deduplicated {before} -> {len(weeds)} weeds (threshold={DEDUP_DIST_PX}px)")

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

    print(f"[INFO] Saved -> {json_path}")
    return json_path
