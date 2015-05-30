#! /usr/bin/env python

import roslib; roslib.load_manifest('robik')
from time import sleep
import rospy
import robik
import robik.msg
import trajectory_msgs.msg
from urdf_parser_py import urdf


arm_pub = rospy.Publisher('/robik_arm_controller_joint_states', trajectory_msgs.msg.JointTrajectory, queue_size=50)
gen_pub = rospy.Publisher('/robik_generic_control', robik.msg.GenericControl, queue_size=50)

current_joint = "n/a"
current_position = 0

def power_arm(value):
    gen_msg = robik.msg.GenericControl()
    gen_msg.header.stamp = rospy.Time.now()
    gen_msg.gen_operation = 5
    gen_msg.gen_param1 = value
    gen_pub.publish(gen_msg)

def set_arm(joint_name, position):
    durS = 0.5 #second
    arm_msg = trajectory_msgs.msg.JointTrajectory()
    arm_msg.header.stamp = rospy.Time.now()
    arm_msg.joint_names = [joint_name]
    arm_msg.points = [
                            trajectory_msgs.msg.JointTrajectoryPoint([position], [], [], [50], rospy.Duration.from_sec(durS)),
                        ]
    arm_pub.publish(arm_msg)
    arm_pub.publish(arm_msg)
    sleep(durS +0.5)


def frange(x, y, jump):
    while x < y:
        yield x
        x += jump


def test_joint(joint_idx):
    global current_joint
    global current_position

    current_joint = robot.joints[joint_idx].name
    for i in frange(robot.joints[joint_idx].limit.lower, robot.joints[joint_idx].limit.upper, 0.15):
        current_position = i
        set_arm(robot.joints[joint_idx].name, current_position)
        rospy.sleep(1.0)

    set_arm(robot.joints[joint_idx].name, robot.joints[1].limit.upper)


def callback(msg):

    print "{0}|{1}|{2}|{3}|{4}|{5}|{6}".format(current_joint,current_position*100, msg.arm_yaw, msg.arm_shoulder, msg.arm_elbow, msg.arm_roll, msg.arm_clamp)

if __name__ == '__main__':

    robot = urdf.Robot.from_xml_file('robot.urdf')

    rospy.init_node('talker', anonymous=True)
    rospy.Subscriber("/robik_status", robik.msg.GenericStatus, callback)

    power_arm(1)
    power_arm(1)

    print "name|position*100|yaw|shoulder|elbow|roll|clamp"

    #raise shoulder upwards to perform most movement safe
    set_arm("shoulder_joint", 1.57)
    set_arm("elbow_joint", 0)

    #cycle yaw
    test_joint(1)

    #reset yaw
    set_arm("yaw_joint", -1.57)

    #cycle elbow
    test_joint(3)

    #reset elbow
    set_arm("elbow_joint", 0.0)

    #cycle roll
    test_joint(4)

    #reset roll
    set_arm("roll_joint", 0.0)

    #cycle clamp
    test_joint(5)

    #reset clamp
    set_arm("clamp_joint", 0.1)

    #set elbow straight
    set_arm("elbow_joint", 0.0)

    #cycle shoulder
    set_arm("yaw_joint", 0.0)
    test_joint(2)

    #park the arm
    set_arm("shoulder_joint", 1.57)
    set_arm("yaw_joint", -1.57)
    set_arm("elbow_joint", -1.6)

    power_arm(0)
    power_arm(0)
