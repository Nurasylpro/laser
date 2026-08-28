"""
calibration_test.py — verify calibration accuracy at every known Z.

What it does
------------
1. Loads all calibrations from CALIB_FOLDER
2. For each calibration height Z:
     - prints mm/pixel scale at image centre
     - prints world-mm at the four image corners and centre
     - if poly_fit.npz is loaded, compares POLY vs PIECEWISE at the SAME Z
       (residuals should be ~0 mm — if not, the poly fit is bad)
3. Click a pixel in the image at the bottom to get its mm position
   under both poly and piecewise — useful for measuring real distances.

Run
---
    python scripts/calibration_test.py
    python scripts/calibration_test.py --image data/raw/test/some_photo.jpg --z 60
"""

import argparse
import sys
from pathlib import Path
import math
import numpy as np
import cv2

SCRIPT_DIR   = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(SCRIPT_DIR))

import calibration_loader as cl
from calibration_loader import (load_all_calibrations, get_homography,
                                 pixel_to_world, normalize_homography)

CALIB_FOLDER = PROJECT_ROOT / "calibration2"
IMG_W, IMG_H = 1280, 720


# ── helpers ────────────────────────────────────────────────────

def mm_per_px_at_center(H, cx, cy):
    p0 = pixel_to_world((cx,     cy), H)
    p1 = pixel_to_world((cx + 1, cy), H)
    p2 = pixel_to_world((cx,     cy + 1), H)
    return math.hypot(p1[0] - p0[0], p1[1] - p0[1]), \
           math.hypot(p2[0] - p0[0], p2[1] - p0[1])


def H_piecewise(z, calibrations):
    """Force piecewise linear, ignoring loaded polynomial."""
    return cl._get_homography_piecewise(z, calibrations)


def H_poly(z):
    """Force polynomial fit (raises if not loaded)."""
    return cl.H_from_poly(z)


# ── 1. Load + summary ──────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", help="Optional photo for click testing")
    parser.add_argument("--z", type=float, default=0.0,
                        help="Z height for click testing (default 0)")
    args = parser.parse_args()

    print("=" * 70)
    print(f"  CALIBRATION TEST  —  folder: {CALIB_FOLDER.name}")
    print("=" * 70)

    calibrations = load_all_calibrations(str(CALIB_FOLDER))
    has_poly     = cl._poly_coeffs is not None
    z_vals       = sorted(calibrations.keys())

    print(f"\nLoaded {len(z_vals)} calibrations: {z_vals}")
    print(f"Polynomial fit loaded: {has_poly}"
          f"{f' (degree {cl._poly_degree})' if has_poly else ''}")

    cx, cy = IMG_W / 2.0, IMG_H / 2.0

    # ── 2. Per-calibration scale + corner values ────────────────
    print("\n" + "=" * 70)
    print("  SECTION A — Raw measured calibration matrices")
    print("=" * 70)
    print(f"{'Z (mm)':>7} | {'mm/px X':>9} | {'mm/px Y':>9} | "
          f"{'centre (X_mm, Y_mm)':>22}")
    print("-" * 70)

    for z in z_vals:
        H_meas = calibrations[z]
        sx, sy = mm_per_px_at_center(H_meas, cx, cy)
        wx, wy = pixel_to_world((cx, cy), H_meas)
        print(f"{z:>7} | {sx:>9.4f} | {sy:>9.4f} | "
              f"({wx:>+8.2f}, {wy:>+8.2f})")

    # ── 3. Poly residuals at each known Z (THE KEY TEST) ────────
    if has_poly:
        print("\n" + "=" * 70)
        print("  SECTION B — Polynomial fit accuracy at calibration heights")
        print("            (residual = poly prediction − measured calibration)")
        print("            ✓ Good fit: residuals near 0 mm")
        print("            ✗ Bad fit:  residuals > 1 mm means polynomial is wrong")
        print("=" * 70)
        print(f"{'Z':>5} | {'centre err':>10} | {'corner TL':>10} | "
              f"{'corner TR':>10} | {'corner BR':>10} | {'corner BL':>10}")
        print("-" * 70)

        max_err = 0.0
        for z in z_vals:
            H_meas = calibrations[z]
            H_p    = H_poly(z)

            errs = []
            for px, py in [(cx, cy),
                           (0, 0), (IMG_W, 0),
                           (IMG_W, IMG_H), (0, IMG_H)]:
                w_m = pixel_to_world((px, py), H_meas)
                w_p = pixel_to_world((px, py), H_p)
                errs.append(math.hypot(w_p[0] - w_m[0], w_p[1] - w_m[1]))

            max_err = max(max_err, max(errs))
            print(f"{z:>5} | {errs[0]:>9.2f}m | {errs[1]:>9.2f}m | "
                  f"{errs[2]:>9.2f}m | {errs[3]:>9.2f}m | {errs[4]:>9.2f}m")

        print(f"\n  → Worst polynomial error at any calibration point: {max_err:.2f} mm")
        if max_err > 2.0:
            print("  ✗ POLYNOMIAL FIT IS POOR.  Recommended: delete poly_fit.npz")
            print("    or rebuild with: python scripts/calibration_bestfit.py --save --degree 3")
        else:
            print("  ✓ Polynomial fit is accurate at calibration heights.")

    # ── 4. Compare poly vs piecewise at midpoints ───────────────
    if has_poly and len(z_vals) >= 2:
        print("\n" + "=" * 70)
        print("  SECTION C — Poly vs Piecewise at midpoints between calibrations")
        print("            (large differences = poly disagrees with linear interp)")
        print("=" * 70)
        print(f"{'Z mid':>7} | {'poly centre':>22} | {'piece centre':>22} | "
              f"{'diff mm':>8}")
        print("-" * 70)

        for i in range(len(z_vals) - 1):
            z_mid = (z_vals[i] + z_vals[i + 1]) / 2.0
            H_p   = H_poly(z_mid)
            H_l   = H_piecewise(z_mid, calibrations)

            wp = pixel_to_world((cx, cy), H_p)
            wl = pixel_to_world((cx, cy), H_l)
            d  = math.hypot(wp[0] - wl[0], wp[1] - wl[1])
            print(f"{z_mid:>7.1f} | ({wp[0]:>+8.2f}, {wp[1]:>+8.2f})    | "
                  f"({wl[0]:>+8.2f}, {wl[1]:>+8.2f})    | {d:>8.2f}")

    # ── 5. Interactive click test on photo (optional) ───────────
    if args.image:
        print("\n" + "=" * 70)
        print(f"  SECTION D — Click test on {args.image}  (Z={args.z})")
        print("            Click any pixel.  Two mm values printed (poly + piecewise).")
        print("            ESC to exit.")
        print("=" * 70)
        click_test(args.image, args.z, calibrations, has_poly)

    print("\nDone.\n")


def click_test(img_path, z_mm, calibrations, has_poly):
    img = cv2.imread(img_path)
    if img is None:
        print(f"[ERROR] Cannot open {img_path}")
        return

    H_l = H_piecewise(z_mm, calibrations)
    H_p = H_poly(z_mm) if has_poly else None

    win = "Click test — ESC to exit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    def on_click(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        wl = pixel_to_world((x, y), H_l)
        if has_poly:
            wp = pixel_to_world((x, y), H_p)
            print(f"  pixel ({x:>4},{y:>4}) | "
                  f"PIECEWISE ({wl[0]:>+8.2f},{wl[1]:>+8.2f}) mm | "
                  f"POLY ({wp[0]:>+8.2f},{wp[1]:>+8.2f}) mm | "
                  f"diff {math.hypot(wp[0]-wl[0], wp[1]-wl[1]):.2f} mm")
        else:
            print(f"  pixel ({x:>4},{y:>4}) → ({wl[0]:>+8.2f},{wl[1]:>+8.2f}) mm")

    cv2.setMouseCallback(win, on_click)

    while True:
        cv2.imshow(win, img)
        if cv2.waitKey(20) & 0xFF == 27:
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
