#!/bin/bash

echo 'Generating RDF'
rosrun xacro xacro `rospack find robik_description`/urdf/robik.urdf.xacro -o ./robik.urdf

echo 'Note: Must be installed: sudo apt-get install liburdfdom-tools'

echo 'Checking gazebo SDF'
gz sdf -k ./robik.urdf

echo 'Checking URDF'
check_urdf ./robik.urdf



#urdf_to_graphiz ./robik.urdf
#rm ./robik.gv
#evince ./robik.pdf
