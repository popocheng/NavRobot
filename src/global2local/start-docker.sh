#!/bin/bash


domain=$2
IP=$(nslookup "$domain" | awk '/^Address: / {print $2}')
echo "IP address of $domain is: $IP"

export ROS_DISCOVERY_SERVER="$IP:11811"
export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
source ~/.bashrc

source /global2local_ws/install/setup.bash
ros2 launch global2local global2local.launch.py drone_id:=$1
