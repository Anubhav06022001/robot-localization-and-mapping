#!/usr/bin/env python3

import argparse
import csv
import os
import numpy as np
import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration", type=float, default=60.0)

    args = parser.parse_args()

    # ========================================================
    # Read trajectory CSV without pandas
    # ========================================================

    timestamps = []
    gt_x = []
    gt_y = []

    with open(args.input, "r") as f:

        reader = csv.DictReader(f)

        for row in reader:

            try:
                if (
                    row.get("timestamp") is None
                    or row.get("gt_x") is None
                    or row.get("gt_y") is None
                    or row["timestamp"] == ""
                    or row["gt_x"] == ""
                    or row["gt_y"] == ""
                ):
                    continue

                timestamps.append(float(row["timestamp"]))
                gt_x.append(float(row["gt_x"]))
                gt_y.append(float(row["gt_y"]))

            except (ValueError, KeyError, TypeError):
                continue

    if len(timestamps) == 0:
        raise RuntimeError(
            "No valid trajectory data found in " + args.input
        )

    t = np.array(timestamps)
    x = np.array(gt_x)
    y = np.array(gt_y)

    # Normalize time
    t = t - t[0]

    # ========================================================
    # Create figure
    # ========================================================

    fig, ax = plt.subplots(figsize=(12, 6))

    ax.set_xlim(-10, 10)
    ax.set_ylim(-3, 3)

    ax.set_xlabel("Corridor X (m)")
    ax.set_ylabel("Corridor Y (m)")

    ax.set_title(
        "Pace Robotics - Dynamic Corridor Demonstration"
    )

    ax.set_aspect("equal")

    # ========================================================
    # Corridor walls
    # ========================================================

    ax.plot(
        [-10, 10],
        [2, 2],
        linewidth=3
    )

    ax.plot(
        [-10, 10],
        [-2, -2],
        linewidth=3
    )

    # ========================================================
    # Moving wall
    # ========================================================

    wall_x = 0.0

    wall_line, = ax.plot(
        [wall_x, wall_x],
        [-0.5, 0.5],
        linewidth=6
    )

    # ========================================================
    # Robot
    # ========================================================

    robot, = ax.plot(
        [x[0]],
        [y[0]],
        marker="o",
        markersize=12
    )

    # ========================================================
    # Robot trajectory
    # ========================================================

    trajectory, = ax.plot(
        [x[0]],
        [y[0]],
        linewidth=2
    )

    # ========================================================
    # Text
    # ========================================================

    time_text = ax.text(
        0.02,
        0.95,
        "",
        transform=ax.transAxes,
        fontsize=12
    )

    wall_text = ax.text(
        0.02,
        0.90,
        "",
        transform=ax.transAxes,
        fontsize=12
    )

    # ========================================================
    # FFmpeg writer
    # ========================================================

    writer = FFMpegWriter(
        fps=10,
        metadata={
            "title": "Pace Robotics Headless Demo"
        },
        bitrate=3000
    )

    # ========================================================
    # Generate frames
    # ========================================================

    total_frames = int(args.duration * 10)

    frame_indices = np.linspace(
        0,
        len(t) - 1,
        total_frames
    ).astype(int)

    with writer.saving(
        fig,
        args.output,
        dpi=120
    ):

        for idx in frame_indices:

            current_t = t[idx]

            # ------------------------------------------------
            # Robot
            # ------------------------------------------------

            robot.set_data(
                [x[idx]],
                [y[idx]]
            )

            # ------------------------------------------------
            # Trajectory
            # ------------------------------------------------

            trajectory.set_data(
                x[:idx + 1],
                y[:idx + 1]
            )

            # ------------------------------------------------
            # Moving wall
            #
            # 0-15 s   -> center
            # 15-30 s  -> +1.5 m
            # 30-45 s  -> -1.5 m
            # 45-60 s  -> +1.5 m
            # ------------------------------------------------

            shift_number = int(current_t // 15)

            if shift_number == 0:
                wall_y = 0.0

            elif shift_number == 1:
                wall_y = 1.5

            elif shift_number == 2:
                wall_y = -1.5

            else:
                wall_y = 1.5

            wall_line.set_data(
                [wall_x, wall_x],
                [wall_y - 0.5, wall_y + 0.5]
            )

            # ------------------------------------------------
            # Text
            # ------------------------------------------------

            time_text.set_text(
                f"Simulation time: {current_t:.1f} s"
            )

            wall_text.set_text(
                f"Moving wall position: Y = {wall_y:+.1f} m"
            )

            writer.grab_frame()

    plt.close(fig)

    print()
    print("==========================================")
    print(" VIDEO GENERATED SUCCESSFULLY")
    print("==========================================")
    print()
    print(args.output)


if __name__ == "__main__":
    main()