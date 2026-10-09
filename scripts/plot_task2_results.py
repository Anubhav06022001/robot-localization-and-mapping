import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path.home() / "Documents" / "pace"
DATA = ROOT / "data"
OUT = ROOT / "results"


def read_csv(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise RuntimeError(f"No data rows found in {path}")
    return rows


# Trajectory comparison
rows = read_csv(DATA / "trajectory_comparison.csv")

gt_x = [float(r["gt_x"]) for r in rows]
gt_y = [float(r["gt_y"]) for r in rows]
slip_x = [float(r["slip_x"]) for r in rows]
slip_y = [float(r["slip_y"]) for r in rows]
fused_x = [float(r["fused_x"]) for r in rows]
fused_y = [float(r["fused_y"]) for r in rows]

x0, y0 = gt_x[0], gt_y[0]

fig, ax = plt.subplots(figsize=(9, 6))
ax.plot([x-x0 for x in gt_x], [y-y0 for y in gt_y],
        label="Ground truth", linewidth=2)
ax.plot([x-x0 for x in slip_x], [y-y0 for y in slip_y],
        label="Slipping odometry")
ax.plot([x-x0 for x in fused_x], [y-y0 for y in fused_y],
        label="EKF fused estimate")
ax.set_xlabel("X displacement from initial ground-truth position (m)")
ax.set_ylabel("Y displacement from initial ground-truth position (m)")
ax.set_title("Task 2: Trajectory Comparison")
ax.grid(True, alpha=0.3)
ax.legend()
fig.tight_layout()
fig.savefig(OUT / "01_trajectory_comparison.png", dpi=180)
plt.close(fig)


# EKF covariance diagonal
rows = read_csv(DATA / "ekf_covariance.csv")
tc = [float(r["time"]) for r in rows]

fig, ax = plt.subplots(figsize=(10, 6))
for key in ["Pxx", "Pyy", "Pyaw", "Pvv", "Pbb"]:
    values = [float(r[key]) for r in rows]
    ax.plot(tc, values, label=key, linewidth=1.2)

ax.set_yscale("log")
ax.set_xlabel("Simulation time (s)")
ax.set_ylabel("Logged covariance diagonal value (log scale)")
ax.set_title("Task 2: EKF Covariance Diagonal")
ax.grid(True, which="both", alpha=0.3)
ax.legend()
fig.tight_layout()
fig.savefig(OUT / "03_ekf_covariance.png", dpi=180)
plt.close(fig)

print("Plots saved successfully:")
print(OUT / "01_trajectory_comparison.png")
print(OUT / "03_ekf_covariance.png")
print(f"Trajectory samples: {len(gt_x)}")
print(f"Covariance samples: {len(tc)}")
print("Note: covariance CSV contains diagonal entries, not the full matrix.")