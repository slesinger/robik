#!/usr/bin/env python

import roslib; roslib.load_manifest('robik')
from time import sleep
import rospy
import math
import robik
import robik.msg
from robik.msg import ArmControl
from geometry_msgs.msg import Twist
from robik.msg import GenericStatus

class Robik(object):

	MOTOR_MAX_TICKS = 20 #number of ticks generated during maximum speed
	range_timer100 = 11

	odomTicksLeft = 0
	odomTicksRight = 0
	motor_left_dir = 1
	motor_right_dir = 1

	servoSenseMedian_yaw = 400
	servoSenseMedian_shoulder = 250
	servoSenseMedian_elbow = 450
	servoSenseMedian_roll = 310
	servoSenseMedian_clamp = 130

	pub_status = rospy.Publisher('robik_status', GenericStatus, queue_size=50)

	def __init__(self, name):
		rospy.Subscriber("robik_arm_control", ArmControl, self.armMessageListener)
		rospy.Subscriber("robik_velocity_control", Twist, self.velocityMessageListener) #tady nedesi typ zpravy

		#TODO set inital dummy arm position
		Robik.armPreInit(self)

		rospy.loginfo('Started, entering main loop')


		r = rospy.Rate(10) # 10hz
		loop_count = 0;
		while not rospy.is_shutdown():
			Robik.loop(self)

			loop_count += 1
			if (loop_count >= 10):
				Robik.loop1000(self)
				loop_count = 0
			

			r.sleep()

	
#---------------------- LOOP() ---------------------
	def loop(self):

		#send status message

		status_msg = robik.msg.GenericStatus()
		status_msg.header.stamp = rospy.Time.now()

	# const std::basic_string<char> log_message = "XXX";
	# _log_message_type log_message;
	# status_msg.log_message = log_message;
	# status_msg.log_count = 0;
	
		#bumper
		status_msg.bumper_front = False

		#odom
		status_msg.odom_ticksLeft = self.motor_left_dir * self.odomTicksLeft
		status_msg.odom_ticksRight = self.motor_right_dir * self.odomTicksRight
		self.odomTicksLeft = 0
		self.odomTicksRight = 0
		status_msg.odom_millisSinceLastUpdate = 100

		#sonar
		status_msg.ultrasound_Back = 0.6
		
		#arm
		status_msg.arm_yaw = self.servoSenseMedian_yaw
		status_msg.arm_shoulder = self.servoSenseMedian_shoulder
		status_msg.arm_elbow = self.servoSenseMedian_elbow
		status_msg.arm_roll = self.servoSenseMedian_roll
		status_msg.arm_clamp = self.servoSenseMedian_clamp

		status_msg.arm_fore_clamp = False
		status_msg.arm_back_clamp = False

		#motion detector
		status_msg.motion_detector = True


		#parking photo sensor
		status_msg.parkSens_inner = 400
		status_msg.parkSens_outer = 400

		self.pub_status.publish(status_msg)


	def loop1000(self):
		#loops once a second
		second_cnt = 0

#------------------------ Listeners -------------------

	def armMessageListener(self, msg):

		self.armSetJointState(msg.arm_clamp, msg.arm_roll, msg.arm_elbow, msg.arm_shoulder, msg.arm_yaw, msg.time_to_complete)



	def velocityMessageListener(self, msg):

		#left
		if (msg.motor_left < 0):
			self.motor_left_dir = -1
		
		if (msg.motor_left > 0):
			self.motor_left_dir = 1
		
		self.odomTicksLeft = self.map_int(abs(msg.motor_left), 0, 1000, 0, self.MOTOR_MAX_TICKS)

		#right
		if (msg.motor_right < 0):
			self.motor_right_dir = -1
		
		if (msg.motor_right > 0):
			self.motor_right_dir = 1
		
		self.odomTicksRight = self.map_int(abs(msg.motor_right), 0, 1000, 0, self.MOTOR_MAX_TICKS)


#------------------------ Utility -------------------

	def armPreInit(self):
		
		Robik.armSetJointState(self, 1092, 1440, 2064, 1045, 1908, 500)



	def armSetJointState(self, clamp, roll, elbow, shoulder,yaw, time_to_complete):

		#TODO will need to interpolate during time

		#rospy.logerr("Y %d S %d E %d R %d C %d", yaw, shoulder, elbow, roll, clamp)

		#Hodnoty PWM
		ARM_MIN_CLAMP = 1200
		ARM_MAX_CLAMP = 650
		ARM_MIN_ROLL = 2250
		ARM_MAX_ROLL = 730
		ARM_MIN_ELBOW = 2250
		ARM_MAX_ELBOW = 500
		ARM_MIN_SHOULDER = 950
		ARM_MAX_SHOULDER = 1830
		ARM_MIN_YAW = 2130
		ARM_MAX_YAW = 600

		#Hodnoty Resistoru (OHM)
		ARM_RES_MIN_CLAMP = 223
		ARM_RES_MAX_CLAMP = 122
		ARM_RES_MIN_ROLL = 378
		ARM_RES_MAX_ROLL = 118
		ARM_RES_MIN_ELBOW = 545
		ARM_RES_MAX_ELBOW = 44
		ARM_RES_MIN_SHOULDER = 170
		ARM_RES_MAX_SHOULDER = 423
		ARM_RES_MIN_YAW = 424
		ARM_RES_MAX_YAW = 111

		ARM_DEG_MIN_CLAMP = 0
		ARM_DEG_MAX_CLAMP = 40
		ARM_DEG_MIN_ROLL = -90
		ARM_DEG_MAX_ROLL = 90
		ARM_DEG_MIN_ELBOW = -110
		ARM_DEG_MAX_ELBOW = 40
		ARM_DEG_MIN_SHOULDER = 0
		ARM_DEG_MAX_SHOULDER = 120
		ARM_DEG_MIN_YAW = -107
		ARM_DEG_MAX_YAW = 63

		#convert PWM to DEG
		yaw_deg      = self.map(yaw, ARM_MIN_YAW, ARM_MAX_YAW, ARM_DEG_MIN_YAW, ARM_DEG_MAX_YAW)
		shoulder_deg = self.map(shoulder, ARM_MIN_SHOULDER, ARM_MAX_SHOULDER, ARM_DEG_MIN_SHOULDER, ARM_DEG_MAX_SHOULDER)
		elbow_deg    = self.map(elbow, ARM_MIN_ELBOW, ARM_MAX_ELBOW, ARM_DEG_MIN_ELBOW, ARM_DEG_MAX_ELBOW)
		roll_deg     = self.map(roll, ARM_MIN_ROLL, ARM_MAX_ROLL, ARM_DEG_MIN_ROLL, ARM_DEG_MAX_ROLL)
		clamp_deg    = self.map(clamp, ARM_MIN_CLAMP, ARM_MAX_CLAMP, ARM_DEG_MIN_CLAMP, ARM_DEG_MAX_CLAMP)

		#convert DEG to RES
		if (yaw != 0):
			self.servoSenseMedian_yaw      = self.map(yaw_deg     , ARM_DEG_MIN_YAW     , ARM_DEG_MAX_YAW     , ARM_RES_MIN_YAW, ARM_RES_MAX_YAW)
		if (shoulder != 0):
			self.servoSenseMedian_shoulder = self.map(shoulder_deg, ARM_DEG_MIN_SHOULDER, ARM_DEG_MAX_SHOULDER, ARM_RES_MIN_SHOULDER, ARM_RES_MAX_SHOULDER)
		if (elbow != 0):
			self.servoSenseMedian_elbow    = self.map(elbow_deg   , ARM_DEG_MIN_ELBOW   , ARM_DEG_MAX_ELBOW   , ARM_RES_MIN_ELBOW, ARM_RES_MAX_ELBOW)
		if (roll != 0):
			self.servoSenseMedian_roll     = self.map(roll_deg    , ARM_DEG_MIN_ROLL    , ARM_DEG_MAX_ROLL    , ARM_RES_MIN_ROLL, ARM_RES_MAX_ROLL)
		if (clamp != 0):
			self.servoSenseMedian_clamp    = self.map(clamp_deg   , ARM_DEG_MIN_CLAMP   , ARM_DEG_MAX_CLAMP   , ARM_RES_MIN_CLAMP, ARM_RES_MAX_CLAMP)
		# rospy.loginfo("PWM %f > DEG %f > RES %f", yaw, yaw_deg, self.servoSenseMedian_yaw)
		#rospy.logerr("y %d s %d e %d r %d c %d", self.servoSenseMedian_yaw, self.servoSenseMedian_shoulder, self.servoSenseMedian_elbow, self.servoSenseMedian_roll, self.servoSenseMedian_clamp)


	def map(self, value, fromLow, fromHigh, toLow, toHigh):

		if (fromHigh == fromLow):
			return fromLow
			
		retur = float(value-fromLow) * ( float(toHigh-toLow) / float(fromHigh-fromLow) ) + toLow
		return retur

	def map_int(self, a_value, a_min, a_max, b_min, b_max):

		if (a_max == a_min):
			return b_min

		a = (a_value / (a_max - a_min))
		b = a * (b_max - b_min)
		return math.floor(b + b_min)






if __name__ == '__main__':

	rospy.init_node('rosserial_node')
	Robik(rospy.get_name())
