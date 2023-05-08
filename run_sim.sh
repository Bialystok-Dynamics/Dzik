#!/usr/bin/bash

catkin build
source devel/setup.bash
roslaunch argo_mini startup.launch rviz:=false
