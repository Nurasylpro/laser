"""
test_model.py — run one or more YOLO weight files against a folder of
already-captured photos and save annotated results, so you can compare
model versions (model/best.pt, best1.pt, best3.pt, last.pt, ...) without
touching the gantry.

Run one model against everything in runs/captures/:
    python scripts/test_model.py --model model/best3.pt

Run one model against a single photo:
    python scripts/test_model.py --model model/best3.pt --images runs/captures/capture_xyz.jpg

Compare ALL weight files in model/ against the same photos, one output
subfolder per model, so you can flip between them:
    python scripts/test_model.py --compare-all --images runs/captures/

Output goes to runs/model_test/<model_stem>/<image_stem>_annotated.jpg —
each detection is drawn with its box, crown point, and confidence, exactly
like save_review.py does during a real run, but standalone.
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
from ultralytics import YOLO

SCRIPT_DIR   = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(SCRIPT_DIR))

import photo as photo_mod   # reuse get_crown_pixel / deduplicate_weeds

DEFAULT_IMAGES_DIR = PROJECT_ROOT / "runs" / "captures"
OUT_DIR             = PROJECT_ROOT / "runs" / "model_test"
MODEL_DIR            = PROJECT_ROOT / "model"

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def collect_images(images_arg: str) -> list:
    p = Path(images_arg)
    if p.is_file():
        return [p]
    if p.is_dir():
        return sorted(f for f in p.iterdir() if f.suffix.lower() in IMAGE_EXTS)
    raise FileNotFoundError(f"No such file or directory: {images_arg}")


def annotate(image_path: Path, weeds: list, elapsed_s: float) -> "cv2.Mat":
    img = cv2.imread(str(image_path))
    h, w = img.shape[:2]

    for weed in weeds:
        x1, y1, x2, y2 = (int(v) for v in weed["box_xyxy"])
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

        px, py = int(weed["crown_px"]), int(weed["crown_py"])
        cv2.drawMarker(img, (px, py), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 14, 2)

        label = f"conf={weed['conf']:.2f}  {weed['source']}"
        cv2.putText(img, label, (x1, max(0, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 255), 2, cv2.LINE_AA)

    bar_h = 26
    cv2.rectangle(img, (0, 0), (w, bar_h), (30, 30, 30), -1)
    cv2.putText(img, f"{image_path.name}  |  {len(weeds)} weed(s)  |  {elapsed_s*1000:.0f}ms",
                (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 255, 200), 1, cv2.LINE_AA)
    return img


def run_one_model(model_path: Path, images: list, conf_thresh: float):
    print(f"\n{'='*60}")
    print(f"  Model: {model_path.name}")
    print(f"{'='*60}")

    model    = YOLO(str(model_path))
    out_dir  = OUT_DIR / model_path.stem
    out_dir.mkdir(parents=True, exist_ok=True)

    total_weeds = 0
    for image_path in images:
        t0 = time.time()
        results = model(str(image_path), conf=conf_thresh, verbose=False)
        elapsed = time.time() - t0
        result  = results[0]

        boxes    = result.boxes
        kps_xy   = result.keypoints.xy   if result.keypoints is not None else None
        kps_conf = result.keypoints.conf if result.keypoints is not None else None

        weeds = []
        if boxes is not None and len(boxes) > 0:
            for i in range(len(boxes)):
                box_xyxy = boxes.xyxy[i].cpu().numpy()
                conf     = float(boxes.conf[i].cpu())
                kp_xy    = kps_xy[i].cpu().numpy()   if kps_xy   is not None else None
                kp_conf  = kps_conf[i].cpu().numpy() if kps_conf is not None else None

                crown_px, crown_py, source = photo_mod.get_crown_pixel(box_xyxy, kp_xy, kp_conf)
                weeds.append({
                    "conf":     round(conf, 3),
                    "crown_px": round(crown_px, 1),
                    "crown_py": round(crown_py, 1),
                    "source":   source,
                    "box_xyxy": box_xyxy.tolist(),
                })
            weeds = photo_mod.deduplicate_weeds(weeds)

        total_weeds += len(weeds)
        print(f"  {image_path.name}: {len(weeds)} weed(s)  ({elapsed*1000:.0f}ms)")

        annotated = annotate(image_path, weeds, elapsed)
        out_path  = out_dir / f"{image_path.stem}_annotated.jpg"
        cv2.imwrite(str(out_path), annotated)

    print(f"\n  {model_path.name}: {total_weeds} weed(s) total across {len(images)} image(s)")
    print(f"  Saved -> {out_dir}")


def main():
    parser = argparse.ArgumentParser(
        description="Run YOLO model(s) against saved photos and save annotated results.")
    parser.add_argument("--model", default=str(photo_mod.DEFAULT_MODEL),
                        help=f"Path to one .pt weights file (default {photo_mod.DEFAULT_MODEL})")
    parser.add_argument("--compare-all", action="store_true",
                        help=f"Run every .pt file found in {MODEL_DIR} instead of just --model.")
    parser.add_argument("--images", default=str(DEFAULT_IMAGES_DIR),
                        help=f"A single image file or a directory of images "
                             f"(default {DEFAULT_IMAGES_DIR})")
    parser.add_argument("--conf", type=float, default=0.38,
                        help="YOLO confidence threshold (default 0.38)")
    args = parser.parse_args()

    images = collect_images(args.images)
    if not images:
        print(f"[ERROR] No images found in {args.images}")
        sys.exit(1)
    print(f"[INFO] {len(images)} image(s) to test.")

    if args.compare_all:
        model_paths = sorted(MODEL_DIR.glob("*.pt"))
        if not model_paths:
            print(f"[ERROR] No .pt files found in {MODEL_DIR}")
            sys.exit(1)
    else:
        model_paths = [Path(args.model)]

    for model_path in model_paths:
        run_one_model(model_path, images, args.conf)


if __name__ == "__main__":
    main()
