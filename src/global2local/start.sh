#!/bin/bash

source ../../install/setup.bash
if [ $1 ]; then
    ros2 launch global2local global2local.launch.py drone_id:=$1;
else
    ros2 launch global2local global2local.launch.py ;
fi
