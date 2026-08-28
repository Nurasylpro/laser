"""
move.py — send weed path to Arduino over serial, move laser to each weed

Run:
    python scripts/move.py --path runs/paths/weed1_z-9_path.json --port COM3

Flags:
    --port      Serial port Arduino is on  (e.g. COM3  or  /dev/ttyUSB0)
    --baud      Baud rate (default 115200 — must match Arduino sketch)
    --dry-run   Print commands without sending — safe way to check coordinates first
    --home      Home all axes before starting (sends 'h' to Arduino)
"""

import argparse
import json
import sys
import time
from pathlib import Path

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[ERROR] pyserial not installed. Run:  pip install pyserial")
    sys.exit(1)


# ── Helpers ────────────────────────────────────────────────────

def list_ports():
    """Print all detected serial ports — helps find the right COM port."""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("  (no serial ports detected)")
    for p in ports:
        print(f"  {p.device:12s}  {p.description}")


def wait_for_response(ser, keyword: str) -> str:
    """
    Block until a line from Arduino contains `keyword`. No timeout —
    physical operations (homing, moves) can legitimately take longer than
    any fixed ceiling would allow. Ctrl+C if something is actually stuck.
    """
    while True:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if line:
            print(f"    Arduino: {line}")
        if keyword.lower() in line.lower():
            return line


def send_command(ser, cmd: str, wait_for: str, dry_run: bool = False) -> str:
    """Send a command string and wait for the expected response keyword."""
    print(f"  → {cmd}")
    if dry_run:
        return ""
    ser.write((cmd + "\n").encode("utf-8"))
    return wait_for_response(ser, wait_for)


def get_position(ser, dry_run=False) -> tuple[float, float, float]:
    """Query current gantry position via 'p' command. Returns (x, y, z) mm."""
    print("  → p  (querying current position)")
    if dry_run:
        return (0.0, 0.0, 0.0)
    ser.write(b"p\n")
    while True:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if "X=" in line and "Y=" in line and "Z=" in line:
            print(f"    Arduino: {line}")
            # parse  "X=12.50mm  Y=34.00mm  Z=50.00mm"
            try:
                parts = line.split()
                x = float(parts[0].replace("X=", "").replace("mm", ""))
                y = float(parts[1].replace("Y=", "").replace("mm", ""))
                z = float(parts[2].replace("Z=", "").replace("mm", ""))
                return x, y, z
            except (IndexError, ValueError):
                pass


# ── Main ───────────────────────────────────────────────────────

def run(path_json: str, port: str, baud: int,
        dry_run: bool, do_home: bool):

    # 1. Load path JSON
    with open(path_json) as f:
        data = json.load(f)

    z_mm   = data["z_mm"]
    weeds  = data["path"]           # already in visit order

    print(f"\n{'='*60}")
    print(f"  WeedZap — move.py")
    print(f"  Path file  : {path_json}")
    print(f"  Z height   : {z_mm} mm")
    print(f"  Weeds      : {len(weeds)}")
    print(f"  Port       : {port if not dry_run else '(dry-run)'}")
    print(f"{'='*60}\n")

    if not weeds:
        print("[INFO] No weeds in path. Nothing to do.")
        return

    # 2. Print visit table
    print(f"  {'#':>3}  {'abs_X':>8}  {'abs_Y':>8}  {'src'}")
    print(f"  {'-'*40}")
    for w in weeds:
        print(f"  {w['visit_order']:>3}  "
              f"{w['abs_x']:>+8.1f}  {w['abs_y']:>+8.1f}  "
              f"  {w.get('source','?')}")
    print()

    if dry_run:
        print("[DRY-RUN] Commands that would be sent:")
        if do_home:
            print("  → h  (home all axes)")
        for w in weeds:
            print(f"  → m {w['abs_x']:.2f} {w['abs_y']:.2f} {z_mm:.2f}")
        print("[DRY-RUN] Done — no serial port opened.")
        return

    # 3. Open serial port
    print(f"[INFO] Opening {port} at {baud} baud …")
    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open port: {e}")
        print("\nAvailable ports:")
        list_ports()
        sys.exit(1)

    # Give Arduino time to reset after serial connection
    print("[INFO] Waiting for Arduino to boot (2 s) …")
    time.sleep(2.0)
    ser.reset_input_buffer()

    # 4. Optional home
    if do_home:
        print("\n[STEP] Homing all axes …")
        send_command(ser, "h", wait_for="HOMING COMPLETE")
        print("[INFO] Homing complete.\n")

    # 5. Report current position
    print("[STEP] Current gantry position:")
    cx, cy, cz = get_position(ser)
    print(f"         X={cx:.1f}mm  Y={cy:.1f}mm  Z={cz:.1f}mm\n")

    # 6. Move to each weed in visit order
    print(f"[STEP] Executing path — {len(weeds)} weeds …\n")
    for w in weeds:
        num   = w["visit_order"]
        abs_x = w["abs_x"]
        abs_y = w["abs_y"]

        print(f"  Weed #{num}  →  X={abs_x:+.1f}mm  Y={abs_y:+.1f}mm  Z={z_mm:.1f}mm")
        cmd = f"m {abs_x:.2f} {abs_y:.2f} {z_mm:.2f}"
        send_command(ser, cmd, wait_for="Move complete")

        # ── laser-fire placeholder ──────────────────────────────
        # TODO: add GPIO / serial command to fire laser here
        # e.g.:  send_command(ser, "laser on", wait_for="ok")
        #        time.sleep(0.5)
        #        send_command(ser, "laser off", wait_for="ok")
        # ───────────────────────────────────────────────────────

        print(f"  ✓ Weed #{num} reached.\n")

    print("[INFO] All weeds visited. Path complete.")
    ser.close()


# ── Entry point ────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Send weed path to Arduino and move laser to each weed."
    )
    parser.add_argument("--path", required=True,
                        help="Path JSON from path.py  "
                             "e.g. runs/paths/weed1_z-9_path.json")
    parser.add_argument("--port", default=None,
                        help="Serial port  e.g. COM3  or  /dev/ttyUSB0. "
                             "Omit to list available ports.")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (default 115200)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print commands without sending — no serial needed")
    parser.add_argument("--home", action="store_true",
                        help="Home all axes before starting path")

    args = parser.parse_args()

    if args.port is None and not args.dry_run:
        print("Available serial ports:")
        list_ports()
        print("\nRe-run with  --port <port>  or use  --dry-run  to test without hardware.")
        sys.exit(0)

    run(
        path_json = args.path,
        port      = args.port or "DRYRUN",
        baud      = args.baud,
        dry_run   = args.dry_run,
        do_home   = args.home,
    )
