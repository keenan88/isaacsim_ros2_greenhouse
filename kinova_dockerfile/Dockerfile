# Fine to use humble image since it is the only base image
FROM moveit/moveit2:iron-release 

RUN apt update
RUN apt install vim -y

# Packages for Kinova & Moveit, documentation: https://github.com/Kinovarobotics/ros2_kortex
#RUN apt install ros-humble-kortex-bringup -y
#RUN apt install ros-humble-kinova-gen3-6dof-robotiq-2f-85-moveit-config -y
#RUN apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp -y
#ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt install gnome-terminal dbus-x11 -y
#RUN apt install ros-humble-moveit-setup-asssistant -y

RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc