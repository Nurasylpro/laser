import argparse
import sys
from pathlib import Path

from ultralytics import YOLO

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR / "scripts"))
import photo as photo_mod   # reuses the project's own camera capture

parser = argparse.ArgumentParser()
src = parser.add_mutually_exclusive_group()
src.add_argument("--camera", type=int, nargs="?", const=0, metavar="INDEX",
                 help="Capture a live photo from this camera index (default 0) "
                      "instead of using a saved image.")
src.add_argument("--image", metavar="PATH",
                 help="Use an existing saved image instead of capturing.")
parser.add_argument("--preview", action="store_true",
                    help="Show a preview window before capturing (only with --camera).")
parser.add_argument("--model", default=r"C:\Users\nuamy\Desktop\dauren\model\best3.pt")
parser.add_argument("--conf", type=float, default=0.25)
args = parser.parse_args()

if args.image:
    source = args.image
else:
    # Default behavior: take a live photo (--camera with no value also works, index 0)
    camera_index = args.camera if args.camera is not None else 0
    source = photo_mod.capture_from_camera(camera_index, preview=args.preview)
    print(f"[INFO] Captured → {source}")

model = YOLO(args.model)

results = model.predict(
    source=source,
    conf=args.conf,
    save=True,
    project="runs/pose",
    name="predict_test",
    exist_ok=True,
)

for r in results:
    print(r.path, "→", len(r.boxes), "detections")