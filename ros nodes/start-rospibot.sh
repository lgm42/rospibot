#!/bin/bash

source /home/ubuntu/catkin_ws/devel/setup.bash
   
#demarrage des service ros désirés
roslaunch rosbridge_server rosbridge_websocket.launch &
rosrun motors motors.py &
rosrun ina219 ina219Reader.py 
