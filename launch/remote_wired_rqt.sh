#!/bin/sh
cd
export ROS_MASTER_URI=http://10.42.0.84:11311 
export ROS_IP=10.42.0.90
# run rqt with a specific perspective
rqt --perspective-file ~/catkin_ws/src/ema_fes_cycling/perspective/EMA_Trike_remote.perspective
