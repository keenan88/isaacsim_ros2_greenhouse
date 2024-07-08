# Nvidia IsaacSim Startup Instructions

## Tested System Specifications
 - Ubuntu 22.04
 - Docker version 27.0.3, build 7d4bcd8
 - No locally installed version of ROS2 (locally installed may have conflicting RMW versions with containerized ROS2 applications)
 - NVIDIA GeForce RTX 3050
 - NVIDIA Driver Version: 535.183.01
 - CUDA Version: 12.2

## Nvidia Setup

1. Follow the instructions in [Nvidia IsaacSim Installation Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html) to gain access to Nvidia's IsaacSim container.
2. Download the Omniverse Launcher from [Nvidia's website](https://www.nvidia.com/en-us/omniverse/download/). To run the .AppImage file, ensure you install `libfuse2`, **not** `fuse`.
3. Within the Omniverse Launcher, install the streaming client from the "exchange" window.

## Worker Navigation App Setup

4. Open a terminal and enter: ```xhost +local: docker```
5. Navigate to the root of this repository and run: ```docker-compose build```
6. Once the build is complete, start the containers with: ```docker-compose up```
7. It may take approximately 2 minutes for IsaacSim to fully start up the first time. An RVIZ window should appear with a large map: 

![image](https://github.com/keenan88/isaacsim_ros2_greenhouse/assets/45887966/eb9a79eb-21ce-4491-9871-2ece68a995e6)

9. Run the following command on your local machine to find the IP address of the humble-isaac-sim container: ```docker network inspect isaacsim_ros2_greenhouse_default```
10. Launch the streaming client from the Omniverse Launcher. Enter the IP address of the IsaacSim container when prompted. This should open a view of IsaacSim.
11. Press `Ctrl + O` in IsaacSim to open the simulation file: ```/isaac-sim/humble_ws/src/antworker_isaacsim_world/antworker_isaacsim_world/antworker_greenhouse_v3.usd```

![image](https://github.com/keenan88/isaacsim_ros2_greenhouse/assets/45887966/961b571e-49ee-48c2-a36c-e8cf8841fcc1)

12. Press the play button in IsaacSim to start the simulation. The terminal should stop displaying messages about transforming from the map frame.

![image](https://github.com/keenan88/isaacsim_ros2_greenhouse/assets/45887966/15aad294-e4fe-433e-8257-d021e90f2acc)

13. Open a web browser and navigate to: ```localhost:8080```. You should see a simple control window.

![image](https://github.com/keenan88/isaacsim_ros2_greenhouse/assets/45887966/495d6371-09d2-435f-ac23-3bd1aec78539)

# Worker Navigation Interaction

13. Use the control box in the web interface as a joystick to drive the robot forward/backward. Observe the robot's movement in the control panel, RVIZ, and IsaacSim. The robot's pathing and frames should be visible in RVIZ.

![image](https://github.com/keenan88/isaacsim_ros2_greenhouse/assets/45887966/d14c3969-5e61-4052-8168-bfbf34015aa8)


14. Enter a numeric value into the text box and click the navigation button to command the robot to navigate to that displacement.
15. The robot's position and navigation status will be displayed in the control window to confirm movement.


