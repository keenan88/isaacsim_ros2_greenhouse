# isaacsim_ros2_greenhouse

# Purpose of this repo

To develop, test, & demonstrate behaviour of Ecoation's ANT robots in a containerized docker environment.

# Validated Environment

Operating System: Ubuntu 22.04 
Graphics Card: NVIDIA GeForce RTX 3050
NVIDIA Driver version: 535.171.04
Docker 26.1.1

# Environment setup

1. Follow instructions to prepare to run IsaacSim headless in a docker container: https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html.
  a. Note 1: Just follow the steps installing Nvidia drivers, Docker, and the Nvidia container toolkit. The image we use comes from 
  b. Note 2: The tutorial reccomends using Nvidia drivers version 525.x.x, in the linked tutorial and here: https://docs.omniverse.nvidia.com/platform/latest/common/technical-requirements.html. I have successfully used 535.x.x drivers.
3. Run docker-compose.yaml to start containers:
  a. IsaacSim (headless), with ROS2 humble.
  b. ROS2 Humble-equipped container.
