#!/bin/bash

rosrun xacro xacro `rospack find robik_description`/urdf/robik.urdf.xacro -o ./robik.urdf

echo 'Must be installed: sudo apt-get install liburdfdom-tools'
check_urdf ./robik.urdf
urdf_to_graphiz ./robik.urdf
rm ./robik.gv
evince ./robik.pdf
