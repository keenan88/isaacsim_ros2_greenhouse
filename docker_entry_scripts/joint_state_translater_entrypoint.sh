#!/bin/bash

source /opt/ros/humble/setup.bash
# gnome-terminal

cd /home/humble_ws/
colcon build --packages-select moveit_rolling_humble_interface
source install/setup.bash
ros2 run moveit_rolling_humble_interface joint_state_translater



bash