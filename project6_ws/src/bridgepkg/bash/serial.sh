#!/bin/bash


source /opt/ros/noetic/setup.bash 
sleep 2 
source /home/robocon/RC/project6_ws/devel/setup.bash
sleep 2

rosrun slampkg serial_node
