# Mobile Robotics Final Project

Colin Vincent
12/13/2017

## On turtlebot

roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_navigation gmapping_demo.launch

## On workstation

roslaunch turtlebot_teleop keyboard_teleop.launch
roslaunch apriltags_ros april_tags_
python ~/catkin_ws/src/robotics_final.py

Added to .bashrc
* echo export ROS_HOSTNAME=10.110.58.81 >> ~/.bashrc
* export ROS_MASTER_URI=http://rocksteady.neu.edu:11311 >> ~/.bashrc
* export ROS_IP=10.110.58.81 >> ~/.bashrc
* cd ~/catkin_ws
* source devel/setup.bash

## Mapping Demo

roslaunch turtlebot_rviz_launchers view_robot.launch
roslaunch turtlebot_rviz_launchers view_navigation.launch

## Notes
* /camera/rbg/image_rect_color
* changed image to raw in launch file
* --rviz using tag_detections_image