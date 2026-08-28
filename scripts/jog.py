"""
jog.py — interactive terminal for Arduino gantry testing

Run:
    python scripts/jog.py --port COM3

Commands (same as Arduino serial):
    h            — home all axes
    m X Y Z      — move to absolute position in mm  e.g.  m 100 0 0
    p            — print current position
    sw           — print limit switch states
    q / exit     — quit

Tips for testing precision:
    1. Home:          h
    2. Move to 100mm: m 100 0 0
    3. Measure with ruler on the machine
    4. Check position: p
    5. Move back home: m 0 0 0
    6. Repeat and compare
"""

import argparse
import sys
import threading
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[ERROR] pyserial not installed.  Run:  pip install pyserial")
    sys.exit(1)


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("  (no serial ports detected)")
    for p in ports:
        print(f"  {p.device:12s}  {p.description}")


def reader_thread(ser: serial.Serial, stop_event: threading.Event):
    """Background thread — continuously prints lines coming from Arduino."""
    while not stop_event.is_set():
        try:
            raw = ser.readline()
            if raw:
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    # Print on its own line, then re-show the prompt
                    print(f"\r  Arduino: {line}")
                    print("jog> ", end="", flush=True)
        except Exception:
            break


def run(port: str, baud: int):
    print(f"\nOpening {port} at {baud} baud …")
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"[ERROR] {e}")
        print("\nAvailable ports:")
        list_ports()
        sys.exit(1)

    print("Waiting for Arduino to boot (2 s) …\n")
    time.sleep(2.0)
    ser.reset_input_buffer()

    print("=" * 50)
    print("  WeedZap Jog Terminal")
    print("  Commands:  h | m X Y Z | p | sw | q")
    print("=" * 50)

    stop_event = threading.Event()
    t = threading.Thread(target=reader_thread, args=(ser, stop_event), daemon=True)
    t.start()

    try:
        while True:
            try:
                cmd = input("jog> ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not cmd:
                continue

            if cmd.lower() in ("q", "quit", "exit"):
                break

            # send to Arduino
            ser.write((cmd + "\n").encode("utf-8"))

    finally:
        stop_event.set()
        ser.close()
        print("\nPort closed. Bye.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Interactive gantry terminal for testing movement and homing."
    )
    parser.add_argument("--port", default=None,
                        help="Serial port  e.g. COM3  or  /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    if args.port is None:
        print("Available serial ports:")
        list_ports()
        print("\nRe-run with  --port <port>  e.g.  python scripts/jog.py --port COM3")
        sys.exit(0)

    run(args.port, args.baud)
