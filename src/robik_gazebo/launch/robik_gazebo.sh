#!/bin/bash
roslaunch robik_gazebo robik_home.launch &
sleep 8

# NOTE: due to bug gazebo_ros_api_plugin.cpp, launch files had to be split
roslaunch robik_gazebo robik_spawn.launch
