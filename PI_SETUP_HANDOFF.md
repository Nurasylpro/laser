# WeedZap — handoff: bringing the `laser/` rebuild onto the Raspberry Pi

Paste this whole file into the Claude session running on the Raspberry Pi (the
one that's been working on the website / `pi/server.py`) as full context
before asking it to wire this in. It explains what was built, why, what's
still Windows-only, and what needs to happen on the Pi side.

## Why this exists

The old pipeline (under `~/Desktop/weedzap/` on the Pi, mirrored on the dev
machine as `dauren/`) had two problems:

1. The laser tool physically blocks the camera once mounted, but the old
   code attached it *before* scanning — camera and laser were coupled the
   whole time.
2. Weeds sometimes got zapped at the wrong spot. Root cause found: the
   row-scan trusted the *commanded* stop position instead of re-querying
   the gantry's real position, and separately, `get_gantry_position()`'s
   parser could never actually match the firmware's `pos`/`p` output format
   (a silent, permanent hang/mismatch bug).

Both were fixed by rebuilding the Python pipeline + firmware from scratch in
a new project folder, `laser/`, developed and bench-tested on a Windows PC
with the Arduino/gantry temporarily wired to that PC over USB (COM7) instead
of the Pi. **The physical camera, laser, and gantry are the same hardware**
— only the controlling computer differs. Everything here should carry over
to Pi control unchanged *except* the two Windows-vs-Pi-specific bits called
out below.

## What changed vs. the old `dauren/`/`weedzap/` pipeline

- **Tour-then-zap flow**: home → sweep both rows with NO tool attached
  (camera unobstructed) → record every weed's absolute (X,Y) → attach the
  laser tool once → visit every recorded weed and fire. Old flow attached
  the laser before scanning.
- **Position-query fix**: firmware's `printPositions()` was rewritten to
  print an unambiguous `X=<mm>mm Y=<mm>mm Z=<mm>mm` line (old format was
  `POS XL=<steps>(<mm>mm) XR=... Y=... Z=...`, which the Python parser could
  never match — every `p` query hung forever). Every tour stop now
  re-queries the real position fresh after each move instead of trusting
  the commanded coordinates.
- **Tool 1 (laser) gets 10mm more approach/retreat clearance** than tool 2
  (`TOOL1_X_OFFSET=110` vs the shared `TOOL_X_OFFSET=100`) — found 2026-08-04
  that the coarse move from the laser's rack toward row 1 was passing too
  close to the mechanical tool's rack.
- **Fire height raised**: `FIRE_Z_MM` is now 130 (was 90) — the scan height
  (`SCAN_Z_MM`, used for all photo/pixel-math) is unchanged at 0.
- **Laser now fires until you press Enter**, not a fixed timer — safer for
  bench testing, easy to change back to timed firing later if wanted.
- **CAM_TO_LASER_X/Y is measured, not eyeballed**: click the beam's actual
  landing point in a photo (`measure_laser_offset.py`) instead of nudging a
  guess with arrow keys. Saved to `calibration/cam_to_laser.json`, which
  `path.py` auto-loads.
- **New debug tooling** (all standalone, no gantry needed for most):
  `save_review.py` (annotates + saves every tour-stop photo, no blocking
  window), `log_fire_error.py` (click crown vs. burn mark, logs the real mm
  error, `--summary` reports whether error correlates with distance from
  frame center → lens distortion, or is a constant bias → offset
  re-measurement needed), `revisit_and_photo.py` (after a real run, drives
  the bare camera back to every recorded weed for a clean verification
  photo), `test_model.py` (runs YOLO weights against saved photos, no
  hardware, `--compare-all` tests every `.pt` in `model/` at once).

## File layout (`laser/`)

```
laser/
├── firmware/test_sketch/test_sketch.ino   ← flash via Arduino IDE, NOT PlatformIO
├── calibration/
│   ├── z_0.npz, z_90.npz                  ← already measured
│   └── cam_to_laser.json                  ← already measured
├── model/                                 ← best.pt, best1.pt, best3.pt, last.pt
│                                             (best3.pt is the current default)
├── scripts/
│   ├── calibration_loader.py    (piecewise-linear pixel->mm between z_0/z_90)
│   ├── photo.py                 (camera capture + YOLO crown detection)
│   ├── path.py                  (pixel->mm math, CAM_TO_LASER offset, path ordering)
│   ├── weed_zap.py              (main pipeline — tour then zap)
│   ├── calibrate.py             (re-run to add/redo a calibration height)
│   ├── measure_laser_offset.py  (re-run if the camera/laser mount changes)
│   ├── save_review.py           (used internally by weed_zap.py)
│   ├── log_fire_error.py        (offline accuracy diagnostics)
│   ├── revisit_and_photo.py     (post-run accuracy verification photos)
│   └── test_model.py            (offline model comparison, no hardware)
└── runs/                                  ← captures/detections/paths/review, gitignore-able
```

## Windows-vs-Pi differences to check/fix on the Pi side

1. **Serial port**: everything used `--port COM7` on Windows. On the Pi it
   will be something like `/dev/ttyACM0` — `weed_zap.py`/`revisit_and_photo.py`
   already list available ports if you omit `--port`.
2. **Camera backend**: `photo.py` already auto-selects `cv2.CAP_DSHOW`
   (Windows) vs `cv2.CAP_V4L2` (Linux/Pi) based on `platform.system()` — no
   code change needed, just confirm the Pi's camera index (`--camera N`,
   usually 0) and that `CAPTURE_WIDTH/HEIGHT` (1280x720) is actually what
   the Pi's camera driver delivers (`camera_test.py`-equivalent check if
   available, or just look at `[INFO] Camera resolution:` in the output).
3. **Model path**: `photo.py`'s `DEFAULT_MODEL` resolves relative to the
   script location (`model/best3.pt`), so it'll work as long as the whole
   `laser/` folder (including `model/`) is copied over intact — don't just
   copy `scripts/`.
4. **Arduino IDE vs PlatformIO**: `test_sketch.ino`'s header explicitly says
   Arduino IDE, not PlatformIO (unlike the old `dauren/firmware`'s
   PlatformIO setup, if that's still how the Pi flashes). Flashing can be
   done from whichever machine has the Arduino physically connected via USB
   at the time — doesn't have to be the Pi.
5. **Calibration validity**: `z_0.npz`/`z_90.npz`/`cam_to_laser.json` were
   measured with the camera/laser physically mounted as they are on the real
   rig — they should be valid regardless of which computer is running the
   script, PROVIDED the camera/lens/mount hasn't physically changed and the
   *same* USB camera (not a different one on the Pi) is being used. If the
   Pi uses a **different physical camera** than what was used for
   calibration on Windows, recalibrate from scratch on the Pi
   (`calibrate.py --z 0`, `--z 90`, then `measure_laser_offset.py`) — do
   not assume it carries over in that case.

## What's NOT done yet — needs the Pi-side Claude session

- **Integrating with the existing website/`server.py`**: the old FastAPI
  server (`pi/server.py`, WebSocket control, MJPEG stream, `pi/static/`
  web UI) drove the *old* `weed_zap.py`/`arduino.py` interface. It needs to
  be pointed at (or merged with) this new `laser/scripts/weed_zap.py`
  tour-then-zap flow instead — the serial command surface is compatible
  (`h`, `m x y z`, `tool take/put`, `laser <0-255>`, `p`) but the *pipeline
  orchestration* (tour phase, global weed list, revisit tooling) is new and
  the web UI has no concept of it yet.
- **Deciding where `laser/` lives on the Pi** relative to the existing
  `~/Desktop/weedzap/` tree — replace it, live alongside it, or merge.
- **Re-running `--dry-run` and a real bench test on the Pi** once wired up,
  before trusting it beyond what was already bench-tested on Windows.

## Immediate next step to ask the Pi's Claude to do

1. Copy this whole `laser/` folder onto the Pi (scp/rsync from the Windows
   machine, or git if this is tracked in a repo).
2. Confirm serial port and camera index for the Pi's actual hardware.
3. Run `python scripts/weed_zap.py --dry-run` there first to sanity check
   before touching real hardware from the Pi.
4. Then work out the `server.py`/web UI integration together.
