#!/bin/bash

source /opt/ros/humble/setup.bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'

bash