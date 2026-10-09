import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path.home() / "Documents" / "pace"
DATA = ROOT / "data"
OUT = ROOT / "results"


def load_csv(name):
    with open(DATA / name, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise RuntimeError(f"No data in {DATA / name}")
    return rows


# 1. Position error from the latest trajectory log
rows = load_csv("trajectory_comparison.csv")
t = [float(r["timestamp"]) for r in rows]

slip_error = [
    ((float(r["slip_x"]) - float(r["gt_x"]))**2 +
     (float(r["slip_y"]) - float(r["gt_y"]))**2)**0.5
    for r in rows
]
fused_error = [
    ((float(r["fused_x"]) - float(r["gt_x"]))**2 +
     (float(r["fused_y"]) - float(r["gt_y"]))**2)**0.5
    for r in rows
]

fig, ax = plt.subplots(figsize=(10, 5))
ax.plot(t, slip_error, label="Slipping odometry error")
ax.plot(t, fused_error, label="EKF fused error")
ax.set_xlabel("Simulation time (s)")
ax.set_ylabel("Position error (m)")
ax.set_title("Position Error Relative to Ground Truth")
ax.grid(True, alpha=0.3)
ax.legend()
fig.tight_layout()
fig.savefig(OUT / "02_position_error.png", dpi=180)
plt.close(fig)


# 2. Simplified LiDAR information terms
rows = load_csv("scan_degeneracy.csv")
td = [float(r["timestamp"]) for r in rows]

fig, ax = plt.subplots(figsize=(10, 5))
for key in ["Ixx", "Iyy", "Iyaw"]:
    ax.plot(td, [float(r[key]) for r in rows], label=key)
ax.set_xlabel("Simulation time (s)")
ax.set_ylabel("Information term (as computed by detector)")
ax.set_title("LiDAR Information Terms")
ax.grid(True, alpha=0.3)
ax.legend()
fig.tight_layout()
fig.savefig(OUT / "04_information_matrix.png", dpi=180)
plt.close(fig)


# 3. Degeneracy condition number
condition = [max(float(r["condition_number"]), 1e-12) for r in rows]
fig, ax = plt.subplots(figsize=(10, 5))
ax.plot(td, condition)
ax.set_yscale("log")
ax.set_xlabel("Simulation time (s)")
ax.set_ylabel("Condition number (log scale)")
ax.set_title("Simplified Degeneracy Indicator")
ax.grid(True, which="both", alpha=0.3)
fig.tight_layout()
fig.savefig(OUT / "05_degeneracy_condition_number.png", dpi=180)
plt.close(fig)


# 4. Map update counts
rows = load_csv("map_update.csv")
tm = [float(r["timestamp"]) for r in rows]

fig, ax = plt.subplots(figsize=(10, 5))
ax.plot(tm, [int(r["observed_cells"]) for r in rows],
        label="Observed cells")
ax.plot(tm, [int(r["map_cells"]) for r in rows],
        label="Map cells")
ax.plot(tm, [int(r["removed_cells"]) for r in rows],
        label="Removed cells")
ax.set_xlabel("Simulation time (s)")
ax.set_ylabel("Cell count per logged update")
ax.set_title("Local Map Update Log")
ax.grid(True, alpha=0.3)
ax.legend()
fig.tight_layout()
fig.savefig(OUT / "06_map_updates.png", dpi=180)
plt.close(fig)

warnings = sum(
    str(r.get("warning", "")).strip() == "LOCALIZATION_DEGENERACY_WARNING"
    for r in load_csv("scan_degeneracy.csv")
)
removed_total = sum(int(r["removed_cells"]) for r in load_csv("map_update.csv"))

print("Updated figures:")
for name in [
    "02_position_error.png",
    "04_information_matrix.png",
    "05_degeneracy_condition_number.png",
    "06_map_updates.png",
]:
    print(OUT / name)

print(f"Trajectory samples: {len(t)}")
print(f"Degeneracy samples: {len(td)}")
print(f"Degeneracy warning rows: {warnings}")
print(f"Map update samples: {len(tm)}")
print(f"Total logged cell removals: {removed_total}")
print("Note: the information terms are from the simplified detector, not full ICP.")
