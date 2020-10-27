#!/bin/sh
cd
export ROS_MASTER_URI=http://172.20.10.8:11311 
export ROS_IP=172.20.10.7
# run rqt with a specific perspective
rqt --perspective-file ~/catkin_ws/src/ema_fes_cycling/perspective/EMA_Trike_remote.perspective
