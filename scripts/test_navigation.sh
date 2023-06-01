#!/bin/sh

xterm  -e  " roslaunch my_robot world.launch " &
sleep 5

xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find my_robot)/maps/map.yaml" &
sleep 5

xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5

