FROM ros:noetic-ros-base

RUN apt-get update -y

ENV DZIK_WS=/dzik_ws

WORKDIR $DZIK_WS

COPY src src
RUN rosdep install --from-paths src --ignore-src -yr
RUN rm -r src
RUN apt-get install -y ros-noetic-xacro ros-noetic-gazebo-ros-control
