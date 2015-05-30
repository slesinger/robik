#!/usr/bin/env python

#rosbag record -O obyvak_sul_loop.bag

import roslib; roslib.load_manifest('robik')
import rospy
import time

from geometry_msgs.msg import Twist


def go(x, z, cnt):

    twist = Twist()
    twist.linear.x = x
    twist.angular.z = z

    for i in range(cnt):
	p.publish(twist)
	rospy.sleep(0.1)


#--------------
x_speed = 0.3
x_sec = 30

z_speed = 1.0 # *100ms
z_sec = 8 # *100ms
#-------------

rospy.loginfo("Starting Obyvak loop")

rospy.init_node('loop')
p = rospy.Publisher('cmd_vel', Twist, queue_size=500)
rospy.sleep(0.2)


go(x_speed, 0.0, x_sec)
go(0.0, z_speed, z_sec)

go(x_speed, 0.0, x_sec)
go(0.0, z_speed, z_sec)

go(x_speed, 0.0, x_sec)
go(0.0, z_speed, z_sec)

go(x_speed, 0.0, x_sec)
go(0.0, z_speed, z_sec)

#Stop
go(0.0, 0.0, z_sec)

rospy.loginfo("Loop finished")


