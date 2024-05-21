FROM osrf/ros:humble-desktop

RUN apt-get update
RUN apt-get install -y ros-humble-ros-gz

WORKDIR /humble_ws

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc