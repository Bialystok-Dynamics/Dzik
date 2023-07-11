#!/usr/bin/env bash
source /home/rafpi/ws_dzik/devel/setup.bash

#export ROS_IP=192.168.0.100
#export ROS_MASTER_URI=http://192.168.0.102:11311

export ROS_IP=192.168.1.18
export ROS_MASTER_URI=http://192.168.1.11:11311

exec "$@"
