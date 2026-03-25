#!/bin/bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR" || exit

if [ ! -d "install" ]; then
    echo "❌ 未找到 install 目录，请先执行：colcon build --symlink-install"
    exit 1
fi

source /opt/ros/humble/setup.bash
source install/setup.bash

case "$1" in
    dognav) ros2 launch waypoint_nav waypoint_nav.launch.py ;;
    gpsloc) ros2 launch gps_imu_fusion gps_imu_fusion.launch.py ;;
    *) echo "用法：$0 [dognav|gpsloc]"
       echo "  dognav - 导航节点"
       echo "  gpsloc - GPS定位融合节点"
       exit 1 ;;
esac
