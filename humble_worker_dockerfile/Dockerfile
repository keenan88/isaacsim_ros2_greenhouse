# Fine to use humble image since it is the only base image
FROM osrf/ros:humble-desktop

RUN apt update
RUN apt install vim -y

RUN apt install ros-humble-joint-state-publisher-gui -y
RUN apt install ros-humble-navigation2 -y
RUN apt install ros-humble-nav2-bringup -y
RUN apt install ros-humble-teleop-twist-joy -y

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

