#!/bin/bash
# https://github.com/richardw05/mybot_ws

sudo killall gzserver
sudo killall gzclient
sudo killall rviz
sudo killall robot_state_publisher
sudo killall roscore
sudo killall rosmaster

roslaunch robik_gazebo robik_home.launch &
sleep 8

# NOTE: due to bug gazebo_ros_api_plugin.cpp, launch files had to be split
roslaunch robik_gazebo robik_spawn.launch
roslaunch robik_gazebo robik_robot.launch
