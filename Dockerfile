FROM ros:noetic-ros-base

RUN apt-get update -y

ENV DZIK_WS=/dzik_ws

WORKDIR $DZIK_WS

COPY src src
RUN apt-get install -y python3-catkin-tools python3-osrf-pycommon
RUN rosdep install --from-paths src --ignore-src -yr
RUN rm -r src
RUN apt-get install -y ros-noetic-gazebo-ros-control
