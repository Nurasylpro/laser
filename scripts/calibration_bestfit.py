"""
calibration_bestfit.py — visualise and build a polynomial best-fit
                         across all Z-height calibrations.

What it does:
    1. Loads all calibration matrices H(z0), H(z1), ..., H(zN)
    2. Fits a polynomial through ALL calibration points for each H element
    3. Visualises:
         - Each H element vs Z  (scatter = measured, curve = fit)
         - mm/pixel scale vs Z  (most intuitive view)
         - Piecewise linear vs polynomial at intermediate Z values
         - Residual errors per calibration
    4. Optionally saves the polynomial coefficients so calibration_loader
       can use them instead of piecewise interpolation.

Run:
    python scripts/calibration_bestfit.py
    python scripts/calibration_bestfit.py --degree 3   (try different degrees)
    python scripts/calibration_bestfit.py --save        (save fit coefficients)
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

SCRIPT_DIR   = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(SCRIPT_DIR))

from calibration_loader import load_all_calibrations, pixel_to_world_at_height

# ── Config ─────────────────────────────────────────────────────
CALIB_FOLDER   = PROJECT_ROOT / "calibration2"
OUTPUT_DIR     = PROJECT_ROOT / "runs" / "calibration_analysis"

# Test pixel: image center at 1280x720
TEST_PX = (640.0, 360.0)

# ── Parse args ─────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument("--degree", type=int, default=2,
                    help="Polynomial degree for fit (default 2 = quadratic)")
parser.add_argument("--save", action="store_true",
                    help="Save polynomial coefficients to calibration folder")
args = parser.parse_args()

DEGREE = args.degree

# ── Load calibrations ───────────────────────────────────────────
print(f"Loading calibrations from: {CALIB_FOLDER}")
calibrations = load_all_calibrations(str(CALIB_FOLDER))

z_vals = np.array(sorted(calibrations.keys()), dtype=float)
H_list = [calibrations[int(z)] for z in z_vals]

print(f"Loaded {len(z_vals)} calibrations at Z = {z_vals.tolist()} mm")
print(f"Fitting degree-{DEGREE} polynomial to each H element...\n")

# ── Fit polynomial to each H element ───────────────────────────
# H is 3x3 → 9 elements, each gets its own polynomial
coeffs = np.zeros((3, 3, DEGREE + 1))   # [row, col, coeff]

for i in range(3):
    for j in range(3):
        values = np.array([H[i, j] for H in H_list])
        coeffs[i, j] = np.polyfit(z_vals, values, DEGREE)


def H_from_poly(z: float) -> np.ndarray:
    """Evaluate the polynomial fit at height z, return normalised 3x3 H."""
    H = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            H[i, j] = np.polyval(coeffs[i, j], z)
    if abs(H[2, 2]) > 1e-12:
        H /= H[2, 2]
    return H


def px_to_mm_poly(px, z):
    H = H_from_poly(z)
    p = np.array([px[0], px[1], 1.0])
    w = H @ p
    w /= w[2]
    return float(w[0]), float(w[1])


# ── Piecewise linear (current method) ──────────────────────────
def H_piecewise(z: float) -> np.ndarray:
    """Current piecewise linear interpolation between neighbours."""
    from calibration_loader import get_homography
    return get_homography(z, calibrations)


def px_to_mm_piecewise(px, z):
    H = H_piecewise(z)
    p = np.array([px[0], px[1], 1.0])
    w = H @ p
    w /= w[2]
    return float(w[0]), float(w[1])


# ── Residuals ──────────────────────────────────────────────────
print("Residuals (polynomial fit vs measured calibration):")
print(f"  {'Z':>6}   {'err_x_mm':>10}   {'err_y_mm':>10}   {'dist_mm':>10}")
print(f"  {'-'*45}")

residuals_mm = []
for z, H_meas in zip(z_vals, H_list):
    # measured world coord of test pixel
    p  = np.array([TEST_PX[0], TEST_PX[1], 1.0])
    w_meas = H_meas @ p;  w_meas /= w_meas[2]
    # fitted world coord
    H_fit = H_from_poly(z)
    w_fit  = H_fit @ p;   w_fit  /= w_fit[2]

    ex = float(w_fit[0] - w_meas[0])
    ey = float(w_fit[1] - w_meas[1])
    d  = np.hypot(ex, ey)
    residuals_mm.append((ex, ey, d))
    print(f"  Z={int(z):>3}mm   {ex:>+10.3f}   {ey:>+10.3f}   {d:>10.3f} mm")

max_err = max(r[2] for r in residuals_mm)
mean_err = np.mean([r[2] for r in residuals_mm])
print(f"\n  Mean error: {mean_err:.3f} mm   Max error: {max_err:.3f} mm")

# ── Dense Z range for smooth curves ────────────────────────────
z_dense = np.linspace(z_vals[0], z_vals[-1], 300)

# mm/pixel scale at image centre — sample 1px right and 1px down
def mm_per_px_scale(z, method="poly"):
    if method == "poly":
        cx, cy = px_to_mm_poly(TEST_PX, z)
        rx, _  = px_to_mm_poly((TEST_PX[0] + 1, TEST_PX[1]), z)
        _, dy  = px_to_mm_poly((TEST_PX[0], TEST_PX[1] + 1), z)
    else:
        cx, cy = px_to_mm_piecewise(TEST_PX, z)
        rx, _  = px_to_mm_piecewise((TEST_PX[0] + 1, TEST_PX[1]), z)
        _, dy  = px_to_mm_piecewise((TEST_PX[0], TEST_PX[1] + 1), z)
    return abs(rx - cx), abs(dy - cy)   # mm/px in X and Y

# compute for all calibrations (measured)
scale_meas = [mm_per_px_scale(z, "poly") for z in z_vals]    # using each measured H directly
scale_poly  = [mm_per_px_scale(z, "poly") for z in z_dense]
scale_piece = [mm_per_px_scale(z, "piece") for z in z_dense]

# ── Compute measured scale from raw H matrices ──────────────────
scale_meas_raw = []
for z, H_m in zip(z_vals, H_list):
    p  = np.array([TEST_PX[0], TEST_PX[1], 1.0])
    pr = np.array([TEST_PX[0]+1, TEST_PX[1], 1.0])
    pd = np.array([TEST_PX[0], TEST_PX[1]+1, 1.0])
    wc = H_m @ p;  wc /= wc[2]
    wr = H_m @ pr; wr /= wr[2]
    wd = H_m @ pd; wd /= wd[2]
    sx = abs(wr[0] - wc[0])
    sy = abs(wd[1] - wc[1])
    scale_meas_raw.append((sx, sy))


# ═══════════════════════════════════════════════════════════════
# FIGURE 1 — H matrix elements vs Z
# ═══════════════════════════════════════════════════════════════
fig1, axes = plt.subplots(3, 3, figsize=(14, 10))
fig1.suptitle(f"Homography Matrix Elements vs Z Height\n"
              f"(degree-{DEGREE} polynomial fit through {len(z_vals)} calibrations)",
              fontsize=13, fontweight='bold')

element_labels = [
    ["H[0,0]  (x-scale)",  "H[0,1]  (x-shear)",  "H[0,2]  (x-translation)"],
    ["H[1,0]  (y-shear)",  "H[1,1]  (y-scale)",  "H[1,2]  (y-translation)"],
    ["H[2,0]  (perspective x)", "H[2,1]  (perspective y)", "H[2,2]  (norm=1)"],
]

for i in range(3):
    for j in range(3):
        ax = axes[i][j]
        meas_vals = [H[i, j] for H in H_list]
        fit_vals  = [np.polyval(coeffs[i, j], z) for z in z_dense]

        ax.scatter(z_vals, meas_vals, color='royalblue', s=50, zorder=5,
                   label='Measured' if (i == 0 and j == 0) else '')
        ax.plot(z_dense, fit_vals, color='tomato', lw=2,
                label=f'Poly deg {DEGREE}' if (i == 0 and j == 0) else '')

        ax.set_title(element_labels[i][j], fontsize=8)
        ax.set_xlabel("Z (mm)", fontsize=7)
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.3)

handles = [
    plt.Line2D([0],[0], marker='o', color='w', markerfacecolor='royalblue',
               markersize=8, label='Measured calibration'),
    plt.Line2D([0],[0], color='tomato', lw=2,
               label=f'Polynomial fit (degree {DEGREE})'),
]
fig1.legend(handles=handles, loc='upper right', fontsize=9)
plt.tight_layout()


# ═══════════════════════════════════════════════════════════════
# FIGURE 2 — mm/pixel scale vs Z
# ═══════════════════════════════════════════════════════════════
fig2, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
fig2.suptitle("mm/pixel Scale at Image Centre vs Z Height", fontsize=13, fontweight='bold')

# X scale
ax1.scatter(z_vals, [s[0] for s in scale_meas_raw],
            color='royalblue', s=60, zorder=5, label='Measured calibrations')
ax1.plot(z_dense, [s[0] for s in scale_poly],
         color='tomato', lw=2, label=f'Polynomial fit (deg {DEGREE})')
ax1.plot(z_dense, [s[0] for s in scale_piece],
         color='gray', lw=1.5, ls='--', label='Current piecewise linear')
ax1.set_xlabel("Z height (mm)"); ax1.set_ylabel("mm per pixel")
ax1.set_title("X direction scale"); ax1.legend(); ax1.grid(True, alpha=0.3)

# Y scale
ax2.scatter(z_vals, [s[1] for s in scale_meas_raw],
            color='royalblue', s=60, zorder=5, label='Measured calibrations')
ax2.plot(z_dense, [s[1] for s in scale_poly],
         color='tomato', lw=2, label=f'Polynomial fit (deg {DEGREE})')
ax2.plot(z_dense, [s[1] for s in scale_piece],
         color='gray', lw=1.5, ls='--', label='Current piecewise linear')
ax2.set_xlabel("Z height (mm)"); ax2.set_ylabel("mm per pixel")
ax2.set_title("Y direction scale"); ax2.legend(); ax2.grid(True, alpha=0.3)

plt.tight_layout()


# ═══════════════════════════════════════════════════════════════
# FIGURE 3 — Residuals per calibration
# ═══════════════════════════════════════════════════════════════
fig3, ax = plt.subplots(figsize=(10, 5))
fig3.suptitle(f"Polynomial Fit Residuals at Each Calibration Height\n"
              f"(how far the fit deviates from each measured calibration)",
              fontsize=12, fontweight='bold')

errs = [r[2] for r in residuals_mm]
colors = ['green' if e < 1.0 else 'orange' if e < 3.0 else 'red' for e in errs]

bars = ax.bar(z_vals, errs, color=colors, width=6, zorder=3)
ax.axhline(mean_err, color='navy', lw=1.5, ls='--', label=f'Mean error: {mean_err:.2f} mm')
ax.set_xlabel("Z height (mm)", fontsize=11)
ax.set_ylabel("Position error (mm)", fontsize=11)
ax.set_title(f"Max error: {max_err:.2f} mm  |  Mean error: {mean_err:.2f} mm", fontsize=10)
ax.set_xticks(z_vals)
ax.set_xticklabels([f"Z={int(z)}" for z in z_vals], rotation=30)
ax.legend(); ax.grid(True, alpha=0.3, axis='y')

# Colour legend
from matplotlib.patches import Patch
legend_elements = [Patch(facecolor='green',  label='< 1 mm  (excellent)'),
                   Patch(facecolor='orange', label='1–3 mm  (acceptable)'),
                   Patch(facecolor='red',    label='> 3 mm  (poor)')]
ax.legend(handles=legend_elements, loc='upper right')

for bar, err in zip(bars, errs):
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.03,
            f'{err:.2f}', ha='center', va='bottom', fontsize=8)

plt.tight_layout()


# ═══════════════════════════════════════════════════════════════
# FIGURE 4 — Piecewise vs Polynomial at intermediate Z values
# ═══════════════════════════════════════════════════════════════
# Test at midpoints between calibrations
z_mid = [(z_vals[i] + z_vals[i+1]) / 2 for i in range(len(z_vals)-1)]

diff_x, diff_y, diff_d = [], [], []
for z in z_mid:
    xp, yp = px_to_mm_poly(TEST_PX, z)
    xl, yl = px_to_mm_piecewise(TEST_PX, z)
    diff_x.append(xp - xl)
    diff_y.append(yp - yl)
    diff_d.append(np.hypot(xp - xl, yp - yl))

fig4, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
fig4.suptitle("Polynomial vs Piecewise Linear at Intermediate Z Values\n"
              "(difference in predicted mm position for image centre pixel)",
              fontsize=12, fontweight='bold')

ax1.bar(z_mid, diff_x, width=4, color='steelblue', label='X difference', alpha=0.8)
ax1.bar(z_mid, diff_y, width=4, color='tomato',    label='Y difference', alpha=0.8, bottom=diff_x)
ax1.axhline(0, color='black', lw=0.8)
ax1.set_xlabel("Z height (mm)"); ax1.set_ylabel("Difference (mm)")
ax1.set_title("X and Y difference (poly − piecewise)")
ax1.set_xticks(z_mid)
ax1.set_xticklabels([f"Z={z:.0f}" for z in z_mid], rotation=30)
ax1.legend(); ax1.grid(True, alpha=0.3, axis='y')

ax2.plot(z_mid, diff_d, 'o-', color='purple', lw=2, markersize=8)
ax2.fill_between(z_mid, diff_d, alpha=0.2, color='purple')
ax2.set_xlabel("Z height (mm)"); ax2.set_ylabel("Distance difference (mm)")
ax2.set_title("Total position difference (Euclidean)")
ax2.set_xticks(z_mid)
ax2.set_xticklabels([f"Z={z:.0f}" for z in z_mid], rotation=30)
ax2.grid(True, alpha=0.3)

plt.tight_layout()


# ═══════════════════════════════════════════════════════════════
# SAVE COEFFICIENTS (optional)
# ═══════════════════════════════════════════════════════════════
if args.save:
    out_path = CALIB_FOLDER / "poly_fit.npz"
    np.savez(str(out_path),
             coeffs=coeffs,
             degree=DEGREE,
             z_values=z_vals)
    print(f"\n[INFO] Polynomial coefficients saved → {out_path}")
    print("       To use in calibration_loader: call H_from_poly(z) instead of get_homography()")
else:
    print(f"\nRun with --save to save the polynomial coefficients.")
    print(f"Run with --degree 3 to try a cubic fit.")

# ── Show all figures ────────────────────────────────────────────
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
fig1.savefig(str(OUTPUT_DIR / "h_elements_vs_z.png"), dpi=120, bbox_inches='tight')
fig2.savefig(str(OUTPUT_DIR / "mm_per_px_vs_z.png"),  dpi=120, bbox_inches='tight')
fig3.savefig(str(OUTPUT_DIR / "residuals.png"),        dpi=120, bbox_inches='tight')
fig4.savefig(str(OUTPUT_DIR / "poly_vs_piecewise.png"),dpi=120, bbox_inches='tight')
print(f"\n[INFO] Plots saved to {OUTPUT_DIR}")

plt.show()
