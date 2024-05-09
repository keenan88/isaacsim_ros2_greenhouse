# !/bin/bash

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/catkin_ws/devel/setup.bash"

roscore &

# wait for roscore to start
until rostopic list
do
  sleep 1
done

roslaunch iris_gazebo create_world.launch

roslaunch iris_gazebo iris_world.launch gui:=false &

# # wait for gazbeo to launch map generation plugin
until rosservice list | grep /gazebo_2dmap_plugin/generate_map
do
  sleep 1
done

sleep 1
rosservice call /gazebo_2dmap_plugin/generate_map

# wait for map to be created
until rostopic list | grep /map2d
do
  sleep 1
done

# save map and expose it to host
roscd iris_gazebo
cd maps
rosrun map_server map_saver -f greenhouse map:=/map2d
