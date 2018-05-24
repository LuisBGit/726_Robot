#!/bin/bash

catkin_make
source devel/setup.bash
roslaunch src/robot_driver/launch/PartATask1.launch
