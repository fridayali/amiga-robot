#!/bin/bash
set -e

# ROS2 ortamı
source /opt/ros/${ROS_DISTRO}/setup.bash

# Workspace install varsa source et
if [ -f "${WORKSPACE_ROOT}/install/setup.bash" ]; then
    source ${WORKSPACE_ROOT}/install/setup.bash
fi

# /dev/ttyUSB0 izni (RPLidar)
if [ -e /dev/ttyUSB0 ]; then
    chmod 666 /dev/ttyUSB0
    echo "[entrypoint] /dev/ttyUSB0 hazır"
fi

echo "-------------------------------------------"
echo " ROS2 ${ROS_DISTRO} — Amiga Robot"
echo " Domain ID : ${ROS_DOMAIN_ID:-0}"
echo " Workspace : ${WORKSPACE_ROOT}"
echo "-------------------------------------------"

exec "$@"
