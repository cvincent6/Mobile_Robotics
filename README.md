# Mobile Robotics Final Project

Colin Vincent
12/13/2017

## On turtlebot

1. roslaunch turtlebot_bringup minimal.launch
2. roslaunch turtlebot_navigation gmapping_demo.launch

## On workstation

3. roslaunch apriltags_ros april_tags_detector.launch
4. python ~/catkin_ws/src/robotics_final.py
5. roslaunch turtlebot_rviz_launchers view_navigation.launch

## Notes

* /camera/rbg/image_rect_color changed to raw in launch file
* use rviz camera tag_detections_image to see tags
* roslaunch turtlebot_teleop keyboard_teleop.launch
* roslaunch turtlebot_rviz_launchers view_robot.launch

Added to .bashrc
* export ROS_MASTER_URI=http://splinter.neu.edu:11311 >> ~/.bashrc
* export ROS_IP=10.110.58.81 >> ~/.bashrc
* source devel/setup.bash
