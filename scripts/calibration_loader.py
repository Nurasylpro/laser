"""
calibration_loader.py — load laser/calibration/z_*.npz and evaluate a
pixel -> world-mm homography at any Z by interpolating/extrapolating
between the nearest measured heights (piecewise linear — with only two
calibrated heights, z_0 and z_90, this is a straight line between them).
"""

import os
import re
from pathlib import Path
import numpy as np

SCRIPT_DIR   = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
CALIB_FOLDER = str(PROJECT_ROOT / "calibration")


# ── Filename helpers ────────────────────────────────────────────

def filename_to_z_mm(filename: str) -> int:
    """Convert filename like z_0.npz / z_90.npz into integer Z in mm."""
    stem = os.path.splitext(filename)[0]
    match = re.fullmatch(r"z_(-?\d+)", stem)
    if not match:
        raise ValueError(f"Invalid calibration filename format: {filename}")
    return int(match.group(1))


def normalize_homography(H: np.ndarray) -> np.ndarray:
    """Normalize so H[2,2] == 1 before interpolation/comparison."""
    H = H.astype(np.float64)
    if H.shape != (3, 3):
        raise ValueError(f"Homography must be 3x3, got {H.shape}")
    if abs(H[2, 2]) < 1e-12:
        raise ValueError("Cannot normalize: H[2,2] is too close to zero.")
    return H / H[2, 2]


# ── Load calibrations ───────────────────────────────────────────

def load_all_calibrations(folder: str = CALIB_FOLDER) -> dict:
    """
    Load all z_*.npz calibration matrices from folder.

    Returns:
        { z_mm (int): H (3x3 np.ndarray), ... }
    """
    calibrations = {}

    if not os.path.exists(folder):
        raise FileNotFoundError(
            f"Calibration folder not found: {folder}\n"
            f"Run  python scripts/calibrate.py --z 0  and  --z 90  first."
        )

    for file in os.listdir(folder):
        if not file.endswith(".npz"):
            continue

        path = os.path.join(folder, file)
        try:
            z_mm = filename_to_z_mm(file)
            data = np.load(path)

            if "H" not in data:
                print(f"[WARNING] {file} has no 'H' key. Skipped.")
                continue

            H = data["H"]
            if H.shape != (3, 3):
                print(f"[WARNING] {file} has invalid H shape: {H.shape}. Skipped.")
                continue

            calibrations[z_mm] = normalize_homography(H)

        except Exception as e:
            print(f"[WARNING] Could not load {file}: {e}")

    if not calibrations:
        raise ValueError(
            f"No valid calibration files in {folder}. "
            f"Run  python scripts/calibrate.py --z 0  and  --z 90  first."
        )

    return dict(sorted(calibrations.items()))


def get_available_z(calibrations: dict) -> list:
    return sorted(calibrations.keys())


# ── Piecewise linear ─────────────────────────────────────────────

def interpolate_matrices(H1: np.ndarray, H2: np.ndarray, t: float) -> np.ndarray:
    """
    Linear interpolation/extrapolation between two homography matrices.
    t=0 -> H1,  t=1 -> H2,  t outside [0,1] -> extrapolation.
    """
    H = (1.0 - t) * H1 + t * H2
    return normalize_homography(H)


def get_homography(z_mm: float, calibrations: dict) -> np.ndarray:
    """Return a homography for any requested Z height (piecewise linear)."""
    available_z = get_available_z(calibrations)

    if len(available_z) < 2:
        raise ValueError("At least two calibration matrices are needed.")

    if z_mm in calibrations:
        return calibrations[z_mm]

    if z_mm < available_z[0]:
        z1, z2 = available_z[0], available_z[1]
        t = (z_mm - z1) / (z2 - z1)
        return interpolate_matrices(calibrations[z1], calibrations[z2], t)

    if z_mm > available_z[-1]:
        z1, z2 = available_z[-2], available_z[-1]
        t = (z_mm - z1) / (z2 - z1)
        return interpolate_matrices(calibrations[z1], calibrations[z2], t)

    for i in range(len(available_z) - 1):
        z1, z2 = available_z[i], available_z[i + 1]
        if z1 <= z_mm <= z2:
            t = (z_mm - z1) / (z2 - z1)
            return interpolate_matrices(calibrations[z1], calibrations[z2], t)

    raise ValueError(f"Could not compute homography for z={z_mm}")


# ── Coordinate conversion ───────────────────────────────────────

def pixel_to_world(pixel_xy, H):
    """Convert image pixel to world coordinates in mm using homography H."""
    px, py = pixel_xy
    p_img = np.array([px, py, 1.0], dtype=np.float64)
    p_world = H @ p_img

    if abs(p_world[2]) < 1e-12:
        raise ZeroDivisionError("Homogeneous coordinate too close to zero.")

    p_world /= p_world[2]
    return float(p_world[0]), float(p_world[1])


def pixel_to_world_at_height(pixel_xy, z_mm: float, calibrations: dict):
    """Convenience: pixel + Z height -> world mm (piecewise linear in Z)."""
    H = get_homography(z_mm, calibrations)
    return pixel_to_world(pixel_xy, H)


if __name__ == "__main__":
    calibrations = load_all_calibrations()

    print("Loaded Z values:", get_available_z(calibrations))

    for z in [0, 30, 45, 60, 90]:
        H = get_homography(z, calibrations)
        print(f"\nZ = {z} mm")
        print(H)
        x_mm, y_mm = pixel_to_world((640, 360), H)
        print(f"Pixel (640, 360) -> ({x_mm:.2f}, {y_mm:.2f}) mm")
