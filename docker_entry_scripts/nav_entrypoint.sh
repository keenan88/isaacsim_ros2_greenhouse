#!/bin/bash

source /opt/ros/humble/setup.bash
colcon build --packages-select antworker_navigation && source install/setup.bash
ros2 launch antworker_navigation antworker_nav.launch.py

bash