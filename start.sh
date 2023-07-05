#!/usr/bin/env bash

export ROS_IP=192.168.1.11

source /opt/ros/noetic/setup.bash
source /home/rav/ws_dzik/devel/setup.bash

roslaunch argo_mini raspberry_benchmark.launch mapping:=true amcl:=false navigation:=false
