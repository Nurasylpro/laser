"""
export_model.py — convert YOLO .pt model to faster formats for Raspberry Pi

Run ONCE on your laptop (not on Pi — Pi is too slow for export):
    python scripts/export_model.py

Then copy the output file to the Pi and use it with --model flag:
    python scripts/weed_zap.py --port /dev/ttyACM0 --model model/best1.onnx

Formats and speed on Pi 4 (approximate inference time per frame):
    .pt    — PyTorch original    ~4–8 s   (too slow for real use)
    .onnx  — ONNX Runtime        ~0.8–2 s (good balance, easiest)
    ncnn   — ncnn (ARM native)   ~0.4–1 s (fastest, recommended for Pi)
"""

from pathlib import Path
import sys

SCRIPT_DIR   = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
MODEL_DIR    = PROJECT_ROOT / "model"
DEFAULT_PT   = MODEL_DIR / "best1.pt"

try:
    from ultralytics import YOLO
except ImportError:
    print("[ERROR] ultralytics not installed.  Run:  pip install ultralytics")
    sys.exit(1)


def export(model_path: str, fmt: str):
    print(f"\n[INFO] Loading {model_path} …")
    model = YOLO(model_path)

    print(f"[INFO] Exporting to {fmt.upper()} …")
    out = model.export(format=fmt, imgsz=640)
    print(f"[OK]  Saved → {out}")
    return out


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
        description="Export YOLO model for faster inference on Raspberry Pi."
    )
    parser.add_argument("--model", default=str(DEFAULT_PT),
                        help="Input .pt model file")
    parser.add_argument("--format", default="onnx",
                        choices=["onnx", "ncnn"],
                        help="Export format: onnx (easiest) or ncnn (fastest on Pi)")
    args = parser.parse_args()

    out_path = export(args.model, args.format)

    print(f"""
Done! Next steps:
  1. Copy '{out_path}' to your Raspberry Pi's model/ folder
  2. Install onnxruntime on Pi:
       pip install onnxruntime   (for .onnx)
  3. Run weed_zap.py with:
       python scripts/weed_zap.py --port /dev/ttyACM0 --model model/best1.onnx
""")
