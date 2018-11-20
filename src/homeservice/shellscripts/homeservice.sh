#!/bin/sh

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/homeservice/worlds/blue_u.world" &
sleep 5
xterm -e "export TURTLEBOT_3D_SENSOR=kinect; roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/homeservice/worlds/blue_u_map.yaml" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10
xterm -e "rosrun location_monitor location_monitor" &
sleep 5
xterm -e "rosrun add_marker add_marker" &
sleep 5
xterm -e "rosrun pick_objects pick_objects"
