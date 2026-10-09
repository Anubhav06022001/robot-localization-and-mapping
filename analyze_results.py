#!/usr/bin/env python3
import csv
import math
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

ROOT = Path.cwd()
OUT = ROOT / "results"
OUT.mkdir(exist_ok=True)

def read_csv(name):
    path = ROOT / name
    if not path.exists():
        raise FileNotFoundError(f"Missing {path}")
    with path.open(newline="") as f:
        return list(csv.DictReader(f))

traj = read_csv("trajectory_comparison.csv")
ekf = read_csv("ekf_covariance.csv")
scan = read_csv("scan_degeneracy.csv")
mapdata = read_csv("map_update.csv")

def arr(rows, key):
    return np.array([float(r[key]) for r in rows], dtype=float)

# ---------------- Trajectory ----------------
t = arr(traj, "timestamp")
gtx, gty = arr(traj, "gt_x"), arr(traj, "gt_y")
sx, sy = arr(traj, "slip_x"), arr(traj, "slip_y")
fx, fy = arr(traj, "fused_x"), arr(traj, "fused_y")

fig = plt.figure(figsize=(9, 6))
ax = fig.add_subplot(111)
ax.plot(gtx, gty, label="Ground truth")
ax.plot(sx, sy, label="Slipping odometry")
ax.plot(fx, fy, label="EKF fused")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_title("Ground Truth vs Slipping Odometry vs EKF Fused State")
ax.grid(True, alpha=0.3)
ax.legend()
fig.tight_layout()
fig.savefig(OUT / "01_trajectory_comparison.png", dpi=200)
plt.close(fig)

# Position error against ground truth
slip_err = np.sqrt((sx - gtx)**2 + (sy - gty)**2)
fused_err = np.sqrt((fx - gtx)**2 + (fy - gty)**2)

fig = plt.figure(figsize=(9, 5))
ax = fig.add_subplot(111)
ax.plot(t, slip_err, label="Slipping odometry error")
ax.plot(t, fused_err, label="EKF fused error")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Position error [m]")
ax.set_title("Position Error Relative to Ground Truth")
ax.grid(True, alpha=0.3)
ax.legend()
fig.tight_layout()
fig.savefig(OUT / "02_position_error.png", dpi=200)
plt.close(fig)

# ---------------- EKF covariance ----------------
te = arr(ekf, "time")
pxx, pyy, pyaw = arr(ekf, "Pxx"), arr(ekf, "Pyy"), arr(ekf, "Pyaw")

fig = plt.figure(figsize=(9, 5))
ax = fig.add_subplot(111)
ax.plot(te, pxx, label="Pxx")
ax.plot(te, pyy, label="Pyy")
ax.plot(te, pyaw, label="Pyaw")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Variance")
ax.set_title("EKF Pose Covariance")
ax.grid(True, alpha=0.3)
ax.legend()
fig.tight_layout()
fig.savefig(OUT / "03_ekf_covariance.png", dpi=200)
plt.close(fig)

# ---------------- Scan information / degeneracy ----------------
ts = arr(scan, "timestamp")
ixx, iyy, iyaw = arr(scan, "Ixx"), arr(scan, "Iyy"), arr(scan, "Iyaw")
cond = arr(scan, "condition_number")
deg = arr(scan, "degenerate")

fig = plt.figure(figsize=(9, 5))
ax = fig.add_subplot(111)
ax.plot(ts, ixx, label="Ixx")
ax.plot(ts, iyy, label="Iyy")
ax.plot(ts, iyaw, label="Iyaw")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Information")
ax.set_title("Scan-Matching Information Matrix Components")
ax.grid(True, alpha=0.3)
ax.legend()
fig.tight_layout()
fig.savefig(OUT / "04_information_matrix.png", dpi=200)
plt.close(fig)

fig = plt.figure(figsize=(9, 5))
ax = fig.add_subplot(111)
ax.semilogy(ts, np.maximum(cond, 1.0), label="Condition number")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Condition number")
ax.set_title("Scan Information Matrix Condition Number")
ax.grid(True, alpha=0.3)
ax.legend()
fig.tight_layout()
fig.savefig(OUT / "05_degeneracy_condition_number.png", dpi=200)
plt.close(fig)

# ---------------- Map updates ----------------
tm = arr(mapdata, "timestamp")
observed = arr(mapdata, "observed_cells")
map_cells = arr(mapdata, "map_cells")
removed = arr(mapdata, "removed_cells")

fig = plt.figure(figsize=(9, 5))
ax = fig.add_subplot(111)
ax.plot(tm, observed, label="Observed cells")
ax.plot(tm, map_cells, label="Map cells")
ax.plot(tm, removed, label="Removed stale cells")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Cell count")
ax.set_title("Lifelong Map Update / Stale-Cell Removal")
ax.grid(True, alpha=0.3)
ax.legend()
fig.tight_layout()
fig.savefig(OUT / "06_map_updates.png", dpi=200)
plt.close(fig)

# ---------------- Summary ----------------
def rmse(e):
    return float(np.sqrt(np.mean(e**2)))

warning_count = sum(1 for r in scan if r.get("warning", "").strip() == "LOCALIZATION_DEGENERACY_WARNING")
deg_fraction = float(np.mean(deg > 0))

summary = f"""
PACE ROBOTICS TAKE-HOME — EXPERIMENT SUMMARY

Trajectory samples: {len(traj)}
Trajectory time: {t[0]:.3f} s -> {t[-1]:.3f} s

Ground-truth displacement:
  start = ({gtx[0]:.3f}, {gty[0]:.3f})
  end   = ({gtx[-1]:.3f}, {gty[-1]:.3f})

Slipping odometry:
  start = ({sx[0]:.3f}, {sy[0]:.3f})
  end   = ({sx[-1]:.3f}, {sy[-1]:.3f})
  final position error = {slip_err[-1]:.4f} m
  RMSE = {rmse(slip_err):.4f} m

EKF fused:
  start = ({fx[0]:.3f}, {fy[0]:.3f})
  end   = ({fx[-1]:.3f}, {fy[-1]:.3f})
  final position error = {fused_err[-1]:.4f} m
  RMSE = {rmse(fused_err):.4f} m

EKF final covariance:
  Pxx  = {pxx[-1]:.6g}
  Pyy  = {pyy[-1]:.6g}
  Pyaw = {pyaw[-1]:.6g}

Scan degeneracy:
  samples = {len(scan)}
  degenerate samples = {int(np.sum(deg > 0))}
  degenerate fraction = {deg_fraction:.3f}
  warning rows = {warning_count}
  final Ixx = {ixx[-1]:.6g}
  final Iyy = {iyy[-1]:.6g}
  final Iyaw = {iyaw[-1]:.6g}
  maximum condition number = {np.max(cond):.6g}

Map updates:
  samples = {len(mapdata)}
  total removed-cell count = {int(np.sum(removed))}
  maximum removed in one update = {int(np.max(removed))}
"""

(OUT / "07_experiment_summary.txt").write_text(summary.strip() + "\n")

print(summary)
print(f"\nGenerated plots in: {OUT}")
for p in sorted(OUT.glob("*.png")):
    print(" ", p.name)
