#!/usr/bin/bash

catkin_make
source devel/setup.bash
roslaunch argo_mini startup.launch rviz:=false
