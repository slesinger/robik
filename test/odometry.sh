/bin/bash

rostopic pub -1 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
sleep 3
rostopic pub -1 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
