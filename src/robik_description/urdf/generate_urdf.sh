#!/bin/bash

rosrun xacro xacro `rospack find robik_description`/urdf/robik.urdf.xacro -o ./robik.urdf
