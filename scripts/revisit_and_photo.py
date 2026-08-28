"""
revisit_and_photo.py — after weed_zap.py has toured + fired, drive the bare
camera (no tool attached — must already be returned; weed_zap.py does this
automatically unless run with --no-tool-put) back to every recorded weed
position and take a fresh photo with that weed centered in frame, so you
can see both the crown and the burn mark clearly for log_fire_error.py.

Why this is a separate step: the laser tool physically blocks the camera's
view, so there's no way to see the burn mark AT the moment of firing — you
have to come back afterward with the tool off.

Positioning: weed_zap.py's abs_x/abs_y is the LASER's target (see path.py),
which sits CAM_TO_LASER mm away from the camera. To get the CAMERA (and
therefore the weed) centered in frame here, this script moves to
abs + CAM_TO_LASER instead of abs itself.

Run (after a weed_zap.py run has produced runs/review/tour_weeds.json):
    python scripts/revisit_and_photo.py --port COM7
    python scripts/revisit_and_photo.py --port COM7 --home
"""

import argparse
import json
import sys
import time
from pathlib import Path

SCRIPT_DIR   = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(SCRIPT_DIR))

import photo as photo_mod
import path  as path_mod
from weed_zap import open_serial, send_and_wait, SCAN_Z_MM

TOUR_JSON  = PROJECT_ROOT / "runs" / "review" / "tour_weeds.json"
REVISIT_DIR = PROJECT_ROOT / "runs" / "review" / "revisit"


def run(port: str, baud: int, camera_index: int, do_home: bool):
    if not TOUR_JSON.exists():
        print(f"[ERROR] {TOUR_JSON} not found — run weed_zap.py first.")
        sys.exit(1)

    with open(TOUR_JSON) as f:
        weeds = json.load(f)["weeds"]

    if not weeds:
        print("[INFO] tour_weeds.json has no weeds recorded. Nothing to revisit.")
        return

    print(f"[INFO] {len(weeds)} weed(s) to revisit.")
    print("[WARNING] Make sure the laser tool is NOT attached (run 'tool put' "
          "first if unsure) — the camera needs a clear view.\n")

    ser = open_serial(port, baud)

    if do_home:
        print("[STEP 1] Homing all axes ...")
        send_and_wait(ser, "h", keyword="HOMING COMPLETE")
        print("[INFO] Homed.\n")

    REVISIT_DIR.mkdir(parents=True, exist_ok=True)

    for w in weeds:
        gid   = w.get("global_id", w.get("id"))
        abs_x = w["abs_x"]
        abs_y = w["abs_y"]

        # Undo the camera->laser offset so the CAMERA (not the laser) ends
        # up centered on the weed's real position.
        cam_x = abs_x + path_mod.CAM_TO_LASER_X
        cam_y = abs_y + path_mod.CAM_TO_LASER_Y

        print(f"[WEED {gid}] laser target was X={abs_x:+.1f} Y={abs_y:+.1f} -> "
              f"moving camera to X={cam_x:.1f} Y={cam_y:.1f} Z={SCAN_Z_MM:.1f}")

        cmd = f"m {cam_x:.2f} {cam_y:.2f} {SCAN_Z_MM:.2f}"
        send_and_wait(ser, cmd, keyword="Move complete")

        time.sleep(0.3)   # let the frame settle after the move
        image_path = photo_mod.capture_from_camera(camera_index, preview=False)

        out_path = REVISIT_DIR / f"weed{gid}.jpg"
        Path(image_path).replace(out_path)
        print(f"    -> {out_path}\n")

    ser.close()
    print(f"[INFO] Done. Revisit photos in {REVISIT_DIR}")
    print("Next: for each photo, run")
    print(f"    python scripts/log_fire_error.py --z {SCAN_Z_MM} --image {REVISIT_DIR}\\weed<N>.jpg")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Revisit every weed from the last weed_zap.py tour with the "
                    "bare camera and photograph it for beam-accuracy review.")
    parser.add_argument("--port", required=True, help="Arduino serial port, e.g. COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--camera", type=int, default=0, metavar="INDEX")
    parser.add_argument("--home", action="store_true",
                        help="Home all axes before starting (needed if this is a "
                             "fresh session since the weed_zap.py run).")
    args = parser.parse_args()

    run(args.port, args.baud, args.camera, args.home)
