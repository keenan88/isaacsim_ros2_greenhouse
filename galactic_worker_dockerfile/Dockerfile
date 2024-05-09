# Fine to use galactic image since it is the only base image
FROM osrf/ros:galactic-desktop

RUN apt update
RUN apt install vim -y

RUN apt install ros-galactic-joint-state-publisher-gui -y
RUN apt install ros-galactic-navigation2 -y
RUN apt install ros-galactic-nav2-bringup -y
RUN apt install ros-galactic-teleop-twist-joy -y
# Installing dbus-x11 allows gnome-terminal to run as new shell in docker container, I dont know why
# Inspiration for dbus-x11 taken from: https://github.com/ecoation-labs/ant-drone-ros/blob/a7c76d847c44bf1a9e6afba329de81660ced5a3d/docker_dual_system/local/dockerfiles/commander.Dockerfile#L24
RUN apt install gnome-terminal dbus-x11 -y

RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc

