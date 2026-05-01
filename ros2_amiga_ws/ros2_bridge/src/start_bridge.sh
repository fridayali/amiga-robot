#!/bin/bash
# ==========================================
# Farm-ng Amiga ROS Bridge Launcher
# ==========================================

source /opt/ros/humble/setup.bash
source /ros2_amiga_ws/install/setup.bash

# Paket dizinleri
PACKAGE_DIR="/ros2_amiga_ws/src/ros2_bridge"
SRC="$PACKAGE_DIR/src"
CONFIG="$PACKAGE_DIR/config"

# farm-ng — pip ile kurulu, direkt python3
VENV="python3"

# LOG dizini
LOG_DIR="$PACKAGE_DIR/logs"
mkdir -p "$LOG_DIR"

echo "Starting Farm-ng Amiga ROS bridge services..."
echo "Logs are saved under: $LOG_DIR"

$VENV $SRC/ros2_to_twist.py --service-config $CONFIG/ros2_to_twist.json > "$LOG_DIR/control_ros2.log" 2>&1 &
PID_CONTROL=$!

$VENV $SRC/gps_filter.py --config $CONFIG/gps_filter.json > "$LOG_DIR/gps_filter.log" 2>&1 &
PID_GPS=$!

$VENV $SRC/odometry.py --service-config $CONFIG/odometry.json > "$LOG_DIR/odometry.log" 2>&1 &
PID_ODO=$!

$VENV $SRC/imu_to_ros.py --service-config $CONFIG/imu_to_ros.json > "$LOG_DIR/imu_to_ros.log" 2>&1 &
PID_IMU=$!

$VENV $SRC/cam_to_ros.py --service-config $CONFIG/cam_to_ros.json > "$LOG_DIR/camera.log" 2>&1 &
PID_CAMERA=$!

echo "All services started."
echo "Control PID:          $PID_CONTROL"
echo "GPS Filter PID:       $PID_GPS"
echo "Odometry PID:         $PID_ODO"
echo "IMU PID:              $PID_IMU"
echo "Camera PID:           $PID_CAMERA"
echo ""
echo "Press [CTRL+C] to stop all."

trap "echo 'Stopping all services...'; \
kill $PID_CONTROL $PID_GPS $PID_ODO $PID_IMU $PID_CAMERA 2>/dev/null; exit 0" SIGINT

wait
