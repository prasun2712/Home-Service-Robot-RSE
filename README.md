# Home-Service-Robot-RSE
In this project we will use custom built bot as home service robot by using mapping, localisation and navigation packages of ROS.

## Task
* Student submitted all required files:
    * ROS Packages
    * Shell scripts
* Student's simulation world and robot could properly load in Gazebo.
* The student should write a test_slam.sh script file and launch it to manually test SLAM.
* Student created a functional map of the environment which would be used for localization and navigation tasks.
* The student's robot could navigate in the environment after a 2D Nav Goal command is issued. The student created a test_navigation.sh script file to launch it for manual navigation test.
* The student created a pick_objects.sh file that will send multiple goals for the robot to reach. The robot travels to the desired pickup zone, displays a message that it reached its destination, waits 5 seconds, travels to the desired drop off zone, and displays a message that it reached the drop off zone.
* The student should write a add_marker.sh file that will publish a marker to rviz. The marker should initially be published at the pickup zone. After 5 seconds it should be hidden. Then after another 5 seconds it should appear at the drop off zone.
* The student should write a home_service.sh file that will run all the nodes in this project. The student's home service robot should be simulated as follow:
    * Initially show the marker at the pickup zone.
    * Hide the marker once your robot reach the pickup zone.
    * Wait 5 seconds to simulate a pickup.
    * Show the marker at the drop off zone once your robot reaches it.
* The student should include a brief write-up explaining the packages used for this project, covering localization, mapping and navigation.

## Folder Structure
* **Home-Service-Robot-RSE**
    * **catkin_ws**
        * **src**
            * **add_markers** - Package to model virtual objects. It has two nodes:
                * One publishes marker at pick-up zone, hides it after 5 seconds and then publishes it at drop-off zone again after 5 seconds.
                * Another subscribes to AMCL pose of the robot, initially show the marker at the pickup zone. Hide the marker once the robot reach the pickup zone, waits for 5 seconds and then shows the marker at the drop-off zone once the robot reaches it.
            * **home_service_robot** - Package to containing map file, world file, launch and config files home service robot.
            * **ira_laser_tools** - Submodule, package to merge two laser scans (My robot has two 2-D lidar).
            * **my_robot** - Package containing my custom built robot.
            * **pick_objects** - Package that uses move_base action client to give multiple goals to robot so that it moves to pick-up location, waits for 5 sec and goes to drop-off location after that. It also displays desired messages.
            * **scripts** - All the required script files :
                * **add_markers.sh** - It publishes marker initially at pick-up location, after 5 seconds it hides and after another 5 seconds it appears at drop-off location.
                * **home_service.sh** - It opens gazebo with robot and rviz. Initially virtual object is shown at pick-up location, object disappears as the robot reaches the pickup location. Robot waits for 5 seconds to simulate a pickup and moves to drop-off location. Object appears again as robot reaches drop-off location.
                * **pick_objects.sh** - It opens gazebo with robot and rviz. It sends multiple goals to robot to move it first to the pick-up location and then move it to drop-off location after waiting at pick-up location for 5 seconds.
                * **test_navigation.sh** - It opens gazebo with robot and rviz. You and give goals manually to robot using rviz.
                * **test_slam.sh** - It opens gazebo with robot and rviz. You can tele-operate the robot and test the SLAM.
            * **slam_gmapping** - Submodule, gmapping package to build map of the world.
            * **teleop_twist_keyboard** - Tele-operation package.
    * **images** - Assignment images.

## Images for assignment
|World View   | RVIZ View |
| ----------- |  :---------           |
| ![](https://github.com/prasun2712/Home-Service-Robot-RSE/blob/main/images/home_service_robot_world.png) | ![](https://github.com/prasun2712/Home-Service-Robot-RSE/blob/main/images/pick_location.png) |

## Prerequisite
* Basic knowledge of ROS.
* ROS Kinetic installed.
* xterm, ros-kinetic-navigation, ros-kinetic-map-server, ros-kinetic-move-base, ros-kinetic-amcl, ros-kinetic-dwa-local-planner

## Build and Run
### Build
```
cd ~/
git clone https://github.com/prasun2712/Home-Service-Robot-RSE.git
cd ~/Home-Service-Robot-RSE/
git submodule init
git submodule update
cd ~/Home-Service-Robot-RSE/catkin_ws
catkin_make
```
### Run
#### SLAM test.
```
cd ~/Home-Service-Robot-RSE/catkin_ws/src/scripts
./test_slam.sh
```
Select the xterm in which teleop node is running and move the robot to perform slam.
#### Navigation test
```
cd ~/Home-Service-Robot-RSE/catkin_ws/src/scripts
./test_navigation.sh
```
From rviz give **2D Nav Goal** using the button from the **Tool** panel.
#### Sending multiple goals.
```
cd ~/Home-Service-Robot-RSE/catkin_ws/src/scripts
./pick_objects.sh
```
This will send multiple goals to robot. You can check robot moving to pick location, wait for 5 seconds before going to drop location.
#### Publishing virtual object in rviz.
```
cd ~/Home-Service-Robot-RSE/catkin_ws/src/scripts
./add_markers.sh
```
In rviz you can see object appearing at pick-up location for 5 seconds, disappear after that and again appear after 5 seconds at drop-off location.
#### Home service robot demo.
```
cd ~/Home-Service-Robot-RSE/catkin_ws/src/scripts
./home_service.sh
```
In rviz you can see object apperaing at pick-up location. Object disappears as robot reaches near pick-up location. Robot waits at pick-up location for 5 seconds to simulate pick-up. After that robot moves to drop-off location and object reappears at drop-off location as soon as robot reaches there.