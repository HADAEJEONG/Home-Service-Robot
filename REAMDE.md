HOME SERVICE ROBOT SIMULATION

1) Overview
This package loads the custom robot created via urdf into the world. Then, it loads the map created in advance through SLAM, moves to the specified coordinates, and returns to the origin. Here, the marker at the goal point disappears when it arrives at the goal point, and when it arrives at the origin point, a marker is created at the origin point.

2) Installation
cd catkin_workspace/src
git clone https://github.com/ethz-asl/ros_best_practices.git
cd ../
rosdep install --from-paths . --ignore-src
catkin_make

3) Package description
- add_markers: It serves to display and remove markers in rviz. This package subscribes amcl_pose and the size, shape and coordinates of markers can be changed in this package (add_markers.cpp)
- my_robot:This package contains setting files related to robot and world. Robot settings can be changed in my_robot.gazebo and my_robot.xacro within the urdf file. World settings can be changed in myworld.world in the worlds file.
- pick_objects: This is a package that makes the robot move to the designated goal point and then return to the origin. The goal point, dwell time, etc. can be changed in 'pick_objects.cpp'.
- slam_gmapping: A package that stores algorithms for generating maps. SALM is done through gmapping.
- turtlbot_navigation: The move_base algorithm for navigation and parameter values for costmap settings are stored. Navigation performance can be adjusted by changing parameter values.
- turtlebot_gazebo: This package contains the launch files needed to run the program.
- scripts: This file contains .sh files for program execution. (slam, navigation, home_service)

4) Run
cd catkin_workspace/src/scripts
./home_service.sh
