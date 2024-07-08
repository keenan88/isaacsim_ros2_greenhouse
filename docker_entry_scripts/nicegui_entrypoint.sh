#!/bin/bash
set -e


source /opt/ros/humble/setup.bash

colcon build --packages-select gui

source install/setup.bash

ros2 launch gui main_launch.py

#exec "$@"
exec bash
