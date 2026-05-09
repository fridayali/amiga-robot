#!/bin/bash
source /opt/ros/humble/setup.bash
source /ros2_amiga_ws/install/setup.bash

PACKAGE_DIR="/ros2_amiga_ws/src/ros2_bridge"
SRC="$PACKAGE_DIR/src"
CONFIG="$PACKAGE_DIR/config"
LOG_DIR="$PACKAGE_DIR/logs"
mkdir -p "$LOG_DIR"

echo "Starting Farm-ng Amiga ROS bridge services..."

python3 $SRC/ros2_to_twist.py --service-config $CONFIG/ros2_to_twist.json > "$LOG_DIR/control_ros2.log" 2>&1 &
PID_CONTROL=$!

python3 $SRC/gps_filter.py --config $CONFIG/gps_filter.json > "$LOG_DIR/gps_filter.log" 2>&1 &
PID_GPS=$!

python3 $SRC/odometry.py --service-config $CONFIG/odometry.json > "$LOG_DIR/odometry.log" 2>&1 &
PID_ODO=$!

python3 $SRC/imu_to_ros.py --service-config $CONFIG/imu_to_ros.json > "$LOG_DIR/imu_to_ros.log" 2>&1 &
PID_IMU=$!

python3 $SRC/cam_to_ros.py --service-config $CONFIG/cam_to_ros.json > "$LOG_DIR/camera.log" 2>&1 &
PID_CAMERA=$!

python3 $SRC/motor_battery.py --service-config $CONFIG/motor_battery.json > "$LOG_DIR/motor_battery.log" 2>&1 &
PID_MOTOR=$!

ros2 run ros2_bridge websocket_bridge > "$LOG_DIR/websocket_bridge.log" 2>&1 &
PID_WS=$!

ros2 run task_manager task_manager_node \
  --ros-args -p "track_follower_config:=$CONFIG/track_follower.json" > "$LOG_DIR/task_manager.log" 2>&1 &
PID_TM=$!

echo "Control PID:         $PID_CONTROL"
echo "GPS PID:             $PID_GPS"
echo "Odometry PID:        $PID_ODO"
echo "IMU PID:             $PID_IMU"
echo "Camera PID:          $PID_CAMERA"
echo "Motor/Battery PID:   $PID_MOTOR"
echo "WebSocket Bridge PID:$PID_WS"
echo "Task Manager PID:    $PID_TM"
echo "Press [CTRL+C] to stop."

trap "kill $PID_CONTROL $PID_GPS $PID_ODO $PID_IMU $PID_CAMERA $PID_MOTOR $PID_WS $PID_TM 2>/dev/null; exit 0" SIGINT
wait
