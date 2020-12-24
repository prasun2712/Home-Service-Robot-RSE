#!/bin/bash

cd $(pwd)/../..; catkin_make

xterm  -e  "source devel/setup.bash; roslaunch home_service_robot world.launch" &
sleep 5
xterm -e "source devel/setup.bash; roslaunch my_robot merge_scan.launch" &
sleep 3
xterm -e "source devel/setup.bash; roslaunch home_service_robot map_server.launch" &
sleep 3
xterm -e "source devel/setup.bash; roslaunch home_service_robot amcl.launch" &
sleep 3
xterm -e "source devel/setup.bash; roslaunch my_robot move_base.launch" &
sleep 3
xterm -e "source devel/setup.bash; roslaunch home_service_robot navigation_rviz.launch" &
sleep 3
xterm -e "source devel/setup.bash; roslaunch pick_objects pick_objects.launch"
