# isaacsim_ros2_greenhouse

# Purpose of this repo

To develop, test, & demonstrate behaviour of Ecoation's ANT robots in a IsaacSim + ROS2 Humble Dockerized environment.   


# Steps to install necessary software
1. Verify your machine fulfills the [IsaacSim system requirements] (https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html)
   a. I have used 535.x.x instead of the reccomended 525.x.x drivers to run IsaacSim.
2. Download [Nvidia Omniverse](https://www.nvidia.com/en-us/omniverse/download/) and use it to install the Omniverse Streaming Client.
3. Follow [these steps](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html) to install: 
   a. Nvidia drivers
   b. Docker
   c. Nvidia container toolkit
   d. IsaacSim Docker container image

# Steps to launch simulation
1. Go to the root of the repo and enter <docker compose up>
   a. If this command works properly, you should be able to see:
      i. A huge dump of text in the terminal from IsaacSim starting up. There should be no red text about ROS2 bridge failing to start.
      ii. 2 containers named isaac-sim and humble-desktop if you run <docker ps -a>
2. Open a new terminal and run ifconfig to get your wifi card's inet IP address.
3. Launch Omniverse Streaming Client from Omniverse and enter the IP address.
4. If the connection is successful, you should see IsaacSim load up to a blank world.
5. In IsaacSim, go to File -> Open, and navigate to */isaac-sim/humble_ws/src/antworker_description/usd* and open *greenhouse_worker_v1.usd*.
6. If successful, you should see a world with a pipe rail, two rows of plants, and the ANT Worker robot.

# Steps to drive the robot
1. Open a new terminal and enter <docker exec -it humble-desktop bash> to enter the humble development container.
   a. You could alternatively use the IsaacSim container.
2. Run <source /opt/ros/humble/setup.bash>.
3. Run <ros2 run teleop_twist_keyboard teleop_twist_keyboard>.
4. Hit the play button in IsaacSim. 
5. As you enter Twist commands in the teleop terminal, you should see the robot drive front/back in IsaacSim. It may move slowly if the sim has a low framerate.
6. You can verify the velocity commands are getting to the wheels by viewing velocityCommand textbox in IsaacSim -> Action_Graph_01 -> Articulation Controller
