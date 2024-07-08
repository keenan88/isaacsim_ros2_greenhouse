# Nvidia IsaacSim Startup Instructions

## Nvidia Setup

1. Follow the instructions in [Nvidia IsaacSim Installation Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html) to gain access to the IsaacSim container.
2. Download the Omniverse Launcher from [Nvidia's website](https://www.nvidia.com/en-us/omniverse/download/). Ensure you install `libfuse2`, **not** `fuse`.
3. Within the Omniverse Launcher, install the streaming client from the "exchange" window.

## Source Code Setup

4. Open a terminal and enter: xhost +local: docker
5. Navigate to the root of the repository containing your source code and run: docker-compose build
6. Once the build is complete, start the containers with: docker-compose up
7. It may take approximately 2 minutes for IsaacSim to fully start up. At the end, you may see terminal messages regarding transforms from map not existing yet.
8. Run the following command on your local machine to find the IP address of the `humble-isaac-sim` container: docker network inspect isaacsim_ros2_greenhouse_default.
9. Launch the streaming client from the Omniverse Launcher. Enter the IP address of the IsaacSim container when prompted. This should open a view of IsaacSim.
10. Press `Ctrl + O` to open the simulation file: /isaac-sim/humble_ws/src/antworker_isaacsim_world/antworker_isaacsim_world/antworker_greenhouse_v3.usd
11. Press the play button in IsaacSim to start the simulation. The terminal should stop displaying messages about transforming from the `map` frame.
12. Open a web browser and navigate to:
 ```
 localhost:8080
 ```
 You should see a simple control window.

12. Use the control box in the web interface as a joystick to drive the robot forward/backward. You should observe the robot's movement in the control panel, RVIZ, and IsaacSim.

13. Enter a numeric value into the text box and click the navigation button to command the robot to navigate to that displacement.

14. The robot's position and navigation status will be displayed in the control window to confirm movement.


