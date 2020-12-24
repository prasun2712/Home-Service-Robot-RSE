#!/bin/bash

cd $(pwd)/../..; catkin_make

xterm  -e  "source devel/setup.bash; roslaunch home_service_robot world.launch" &
sleep 5
xterm -e "source devel/setup.bash; roslaunch my_robot merge_scan.launch" &
sleep 3
xterm -e "source devel/setup.bash; roslaunch home_service_robot gmapping.launch" &
sleep 3
xterm -e "source devel/setup.bash; roslaunch my_robot teleop.launch" &
sleep 3
xterm -e "source devel/setup.bash; roslaunch home_service_robot mapping_rviz.launch"
