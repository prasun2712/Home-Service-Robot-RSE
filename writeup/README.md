# Write-up

### Robot
* This is a custom built skid-steer type of robot.
* It has two 2D LIDARS.
* It has one realsense RGBD camera.

### Packages used in the project.
* [ira_laser_tools](http://wiki.ros.org/ira_laser_tools) - This package is used to merge two laser scan data into a single scan. This single scan was used for mapping, localisation and navigation. Node name is ```laserscan_multi_merger```.
* [gmapping](http://wiki.ros.org/gmapping) - This package is used to generate map od the environment. It is a ROS wrapper for OpenSlam's gmapping. It uses laser data and pose data of the robot and generates 2-D occupancy grid map of the environment.
* [map_server](http://wiki.ros.org/map_server) - It provides the ```map_saver``` command-line utility, which allows dynamically generated maps to be saved to file, used this to save map generated using gmapping. It also provides the ```map_server``` ROS Node, which offers map data as a ROS service which is used while localization and navigation.
* [amcl](http://wiki.ros.org/amcl) - This package is used for global localization of the robot against a given map. It uses particle filter to track the pose of a robot against a known map. There are different configurable parameters for overall filter, laser model and odometry model which can be used to tune it.
* [move_base]() - This package is used for navigation. It runs an action server which receives a goal and moves the mobile base to the goal in the map using three major components:
    * global planner to generate the global plan from start position on map to goal position. It uses global costmap to generate path. The global planner used for the project is **navfn**. 
    * local planner to publish the velocity to mobile base, it uses local costmap to avoid obstacles and select the best path based on metric that incorporates characteristics such as: proximity to obstacles, proximity to the goal, proximity to the global path, and speed. Discard illegal trajectories (those that collide with obstacles). The local planner used for the project was **dwa planner**.
    * recovery behavior is performed when the robot is stuck. Robot does in-place rotation and tries to clear the space around few times. In case the recovery behavior fails, move_base notifies the used that it has aborted.