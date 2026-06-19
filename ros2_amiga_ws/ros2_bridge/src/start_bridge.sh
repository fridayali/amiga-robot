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

sleep 3

echo "Starting robot description (TF)..."
ros2 launch amiga_description description.launch.py > "$LOG_DIR/description.log" 2>&1 &
PID_DESC=$!
sleep 2

echo "Starting EKF localization (navsat_ekf)..."
ros2 launch amiga_navsat_ekf navsat_ekf.launch.py > "$LOG_DIR/navsat_ekf.log" 2>&1 &
PID_EKF=$!
sleep 3

echo "Starting LiDAR + laser filter..."
ros2 launch amiga_lidar lidar.launch.py > "$LOG_DIR/lidar.log" 2>&1 &
PID_LIDAR=$!
sleep 3

echo "Starting Nav2 navigation stack..."
ros2 launch amiga_navigation navigation.launch.py > "$LOG_DIR/navigation.log" 2>&1 &
PID_NAV=$!
sleep 2

ros2 run ros2_bridge websocket_bridge > "$LOG_DIR/websocket_bridge.log" 2>&1 &
PID_WS=$!

echo "Starting rosbridge_server (web-teleop için)..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > "$LOG_DIR/rosbridge.log" 2>&1 &
PID_ROSBRIDGE=$!

ros2 run task_manager task_manager_node \
  --ros-args -p "track_follower_config:=$CONFIG/track_follower.json" > "$LOG_DIR/task_manager.log" 2>&1 &
PID_TM=$!

PYTHONPATH=/mnt/managed_home/farm-ng-user-ertugrulkalkan/farm-ng-amiga/py:$PYTHONPATH \
python3 $SRC/tool_control.py $CONFIG/tool_control.json > "$LOG_DIR/tool_control.log" 2>&1 &
PID_TOOL=$!

echo "Control PID:         $PID_CONTROL"
echo "GPS PID:             $PID_GPS"
echo "Odometry PID:        $PID_ODO"
echo "IMU PID:             $PID_IMU"
echo "Camera PID:          $PID_CAMERA"
echo "Motor/Battery PID:   $PID_MOTOR"
echo "Description PID:    $PID_DESC"
echo "EKF PID:             $PID_EKF"
echo "LiDAR PID:           $PID_LIDAR"
echo "Nav2 PID:            $PID_NAV"
echo "WebSocket Bridge PID:$PID_WS"
echo "Rosbridge PID:       $PID_ROSBRIDGE"
echo "Task Manager PID:    $PID_TM"
echo "Tool Control PID:    $PID_TOOL"
echo "Press [CTRL+C] to stop."

trap "kill $PID_CONTROL $PID_GPS $PID_ODO $PID_IMU $PID_CAMERA $PID_MOTOR $PID_DESC $PID_EKF $PID_LIDAR $PID_NAV $PID_WS $PID_ROSBRIDGE $PID_TM $PID_TOOL 2>/dev/null; exit 0" SIGINT
wait
