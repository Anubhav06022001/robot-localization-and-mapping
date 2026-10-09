#!/bin/bash

set -e

# ============================================================
# Pace Robotics - HEADLESS GAZEBO VIDEO RECORDER
# ============================================================

PACE="$HOME/Documents/pace"

VIDEO="$PACE/pace_gazebo_demo_$(date +%Y%m%d_%H%M%S).mp4"

DISPLAY_NUM=":99"
WIDTH=1920
HEIGHT=1080
DURATION=60

echo "=========================================="
echo " Pace Robotics - Gazebo Headless Recorder"
echo "=========================================="
echo
echo "Video:"
echo "$VIDEO"
echo

# ============================================================
# Cleanup
# ============================================================

cleanup() {

    echo
    echo "Stopping processes..."

    kill ${CMD_PID:-0} 2>/dev/null || true
    kill ${LOGGER_PID:-0} 2>/dev/null || true
    kill ${SCAN_PID:-0} 2>/dev/null || true
    kill ${EKF_PID:-0} 2>/dev/null || true
    kill ${NOISE_PID:-0} 2>/dev/null || true
    kill ${GZCLIENT_PID:-0} 2>/dev/null || true
    kill ${GAZEBO_PID:-0} 2>/dev/null || true
    kill ${XVFB_PID:-0} 2>/dev/null || true

    sleep 2

    echo
    echo "=========================================="
    echo " RECORDING COMPLETE"
    echo "=========================================="

    if [ -f "$VIDEO" ]; then
        ls -lh "$VIDEO"
        echo
        echo "Saved:"
        echo "$VIDEO"
    else
        echo
        echo "ERROR: Video was not created."
    fi
}

trap cleanup EXIT

# ============================================================
# ROS
# ============================================================

echo "[1/9] Loading ROS..."

source /opt/ros/humble/setup.bash
source "$PACE/install/setup.bash"

export GAZEBO_MODEL_PATH="$PACE/install/task1_simulation/share/task1_simulation/models"
export GAZEBO_PLUGIN_PATH="$PACE/install/moving_wall_plugin/lib"

# ============================================================
# Start virtual display
# ============================================================

echo "[2/9] Starting virtual display..."

Xvfb $DISPLAY_NUM \
    -screen 0 ${WIDTH}x${HEIGHT}x24 \
    -ac \
    +extension GLX \
    +render \
    -noreset \
    > /tmp/pace_xvfb.log 2>&1 &

XVFB_PID=$!

export DISPLAY=$DISPLAY_NUM

sleep 3

echo "Virtual display: $DISPLAY"
echo "Resolution: ${WIDTH}x${HEIGHT}"

# ============================================================
# Start Gazebo server
# ============================================================

echo "[3/9] Starting Gazebo server..."

gzserver --verbose \
"$PACE/src/task1_simulation/worlds/corridor.world" \
-s libgazebo_ros_init.so \
-s libgazebo_ros_factory.so \
> /tmp/pace_gazebo.log 2>&1 &

GAZEBO_PID=$!

sleep 7

# ============================================================
# Start Gazebo GUI inside virtual display
# ============================================================

echo "[4/9] Starting Gazebo GUI inside virtual display..."

gzclient \
> /tmp/pace_gzclient.log 2>&1 &

GZCLIENT_PID=$!

sleep 10

# ============================================================
# Spawn robot
# ============================================================

echo "[5/9] Spawning TurtleBot..."

ros2 run gazebo_ros spawn_entity.py \
    -entity burger3d \
    -file "$PACE/install/task1_simulation/share/task1_simulation/models/turtlebot3_burger_3d_lidar/model.sdf" \
    -x -8.0 \
    -y 0.0 \
    -z 0.05 \
    > /tmp/pace_spawn.log 2>&1 || true

sleep 5

# ============================================================
# Sensor degradation
# ============================================================

echo "Starting sensor degradation..."

ros2 run noise_injection noise_node \
    > /tmp/pace_noise.log 2>&1 &

NOISE_PID=$!

sleep 2

# ============================================================
# EKF
# ============================================================

echo "Starting EKF..."

ros2 run ekf_localization ekf_node \
    > /tmp/pace_ekf.log 2>&1 &

EKF_PID=$!

sleep 2

# ============================================================
# LiDAR degeneracy
# ============================================================

echo "Starting LiDAR degeneracy analysis..."

ros2 run scan_degeneracy scan_info_node \
    > /tmp/pace_scan.log 2>&1 &

SCAN_PID=$!

sleep 2

# ============================================================
# Trajectory logger
# ============================================================

echo "Starting trajectory logger..."

ros2 run trajectory_logger logger_node \
    > /tmp/pace_logger.log 2>&1 &

LOGGER_PID=$!

sleep 3

# ============================================================
# Robot motion
# ============================================================

echo "[6/9] Starting robot motion..."

ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.20}, angular: {z: 0.0}}" \
    > /tmp/pace_cmdvel.log 2>&1 &

CMD_PID=$!

sleep 3

# ============================================================
# Record actual Gazebo GUI
# ============================================================

echo
echo "=========================================="
echo " GAZEBO RECORDING STARTED"
echo "=========================================="
echo
echo "Virtual display : $DISPLAY"
echo "Resolution      : ${WIDTH}x${HEIGHT}"
echo "Duration        : ${DURATION} seconds"
echo
echo "Recording ACTUAL Gazebo GUI."
echo

ffmpeg \
    -y \
    -f x11grab \
    -video_size ${WIDTH}x${HEIGHT} \
    -framerate 30 \
    -draw_mouse 0 \
    -i ${DISPLAY}.0 \
    -c:v libx264 \
    -preset veryfast \
    -pix_fmt yuv420p \
    -movflags +faststart \
    -t ${DURATION} \
    "$VIDEO"

echo
echo "[9/9] Gazebo recording finished."