#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('sound_play')
import roslib; roslib.load_manifest('robik')
import rospy
import actionlib
import robik
import robik.msg
import time
from geometry_msgs.msg import Twist
from robik.msg import *
from robik.srv import *
from sound_play.libsoundplay import SoundClient

OPERATION_SET_DPIN = 4
PIN_CHARGER_BUTTON = 24
HIGH = 1
LOW = 0


class MoveAction(object):
  # create messages that are used to publish feedback/result
  _feedback = robik.msg.moveFeedback()
  _result   = robik.msg.moveResult()


  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, robik.msg.moveAction, execute_cb=self.execute_cb)
    self._as.start()


  #zastav
  def actionCommand1(self, goal):
    start_time = rospy.Time.now()
    rospy.loginfo('%s: I''m going to execute move action %i' % (self._action_name, goal.command))

    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0
    pub_cmd_vel.publish(msg)

    self._feedback.status = 1000
    self._feedback.msElapsed = rospy.Time.now() - start_time
    self._as.publish_feedback(self._feedback)
    return True

  #zaparkuj se
  def actionCommand2(self, goal):
    start_time = rospy.Time.now()
    rospy.loginfo('%s: I''m going to execute move action %i' % (self._action_name, goal.command))

    #Parking phase 1 - navigate close to homebase
    #to be done
    self._feedback.status = 1000
    self._feedback.msElapsed = rospy.Time.now() - start_time
    self._as.publish_feedback(self._feedback)

    #Parking phase 2 - send request for phase 2 to Driver
    rospy.wait_for_service('setParkingPhase')
    try:
      setParkingPhase_service = rospy.ServiceProxy('setParkingPhase', setParkingPhase)  #service name, message
      resp1 = setParkingPhase_service(2)  #phase 2
      self._feedback.status = 2001
      self._feedback.msElapsed = rospy.Time.now() - start_time
      self._as.publish_feedback(self._feedback)
      return True
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
      return False

  #nabij se
  def actionCommand10(self, goal):
    start_time = rospy.Time.now()
    rospy.loginfo('%s: I''m going to execute move action %i' % (self._action_name, goal.command))

    msg = GenericControl()
    msg.gen_operation = OPERATION_SET_DPIN
    msg.gen_param1 = PIN_CHARGER_BUTTON

    #long press to set charging mode
    msg.gen_param2 = HIGH
    pub_generic_control.publish(msg)
    time.sleep(3)
    msg.gen_param2 = LOW
    pub_generic_control.publish(msg)
    time.sleep(2)
    #short press to start charging
    msg.gen_param2 = HIGH
    pub_generic_control.publish(msg)
    time.sleep(0.2)
    msg.gen_param2 = LOW
    pub_generic_control.publish(msg)

    self._feedback.status = 1000
    self._feedback.msElapsed = rospy.Time.now() - start_time
    self._as.publish_feedback(self._feedback)
    return True

  def pohnise(self, vel, twist, ms):
    Hz = 1.0
    msg = Twist()
    msg.linear.x = vel
    msg.angular.z = twist
#    for i in range(int(ms/(1000.0/Hz))):
    pub_cmd_vel.publish(msg)
    pub_cmd_vel.publish(msg)
    pub_cmd_vel.publish(msg)
    pub_cmd_vel.publish(msg)
    rospy.sleep(ms/1000)
#      rospy.sleep(1/Hz) #0.1 == 100ms == 10Hz

  def pohnirukou(self, yaw, shoulder, elbow, roll, clamp, ms):
    Hz = 10.0
    msg = ArmControl()
    msg.arm_yaw = yaw
    msg.arm_shoulder = shoulder
    msg.arm_elbow = elbow
    msg.arm_roll = roll
    msg.arm_clamp = clamp
    msg.time_to_complete = int(ms)
    pub_arm_control.publish(msg)
    rospy.sleep(0.1)
    pub_arm_control.publish(msg)
    rospy.sleep((ms/1000.0) + 0.1)
#    for i in range(int(ms/(1000.0/Hz))):
#      rospy.loginfo('%i', int(ms - i * 1000.0 / Hz))
#      msg.time_to_complete = int(ms - i * 1000 / Hz)
#      pub_arm_control.publish(msg)
#      rospy.sleep(1.0/Hz) #0.1 == 100ms == 10Hz

  #narozeniny pojidani dortu
  def say(self, text):
    print("I say: " + text)
    soundhandle.say(text.decode('utf8').encode('iso-8859-2'), "voice_czech_ph")

  def actionCommand13(self, goal):
    start_time = rospy.Time.now()
    rospy.loginfo('%s: I''m going to execute move action %i' % (self._action_name, goal.command))

    self.say("bezva, jedu pro dort")
    rospy.sleep(2.0)

    self.pohnirukou(1700, 1280, 1450, 2250, 1000, 900) #zadej o lzicku
    self.say("prosím půjč mi lžičku. nakrmím tě")
    rospy.sleep(5.0)
    self.pohnirukou(0, 0, 0, 0, 1200, 300) #zcvakni prsty
    self.say("díky")
    rospy.sleep(2.0)

    hlasky = ["tu máš. vem si dortik", "za maminku i za tatinka", "doufám že ti chutná můj dortíček", "pak to budeš muset jít vyběhat", "a naposledy ham"]
    for i in range (0, 5):
        self.pohnirukou(1000 + 50 * i, 1300, 1600, 1200, 1200, 900) #priprav
        rospy.sleep(0.5)
        self.say(hlasky[i])
        self.pohnirukou(0, 0, 1870, 0, 0, 300) #naber dolu
        rospy.sleep(0.2)
        self.pohnirukou(0, 0, 0, 2250, 0, 300) #naber otoc
        self.pohnirukou(1700, 1280, 1450, 2250, 1200, 900) #podej
        rospy.sleep(5.0)

    self.say("tak to by zatím stačilo")
    self.pohnirukou(0, 0, 0, 0, 700, 300) #otevri prsty
    rospy.sleep(3.0)
    self.pohnirukou(1908, 1380, 2250, 1740, 1090, 900) #zaparkuj ruku
    rospy.sleep(1.0)
    self.pohnirukou(1908, 1080, 2250, 1740, 1090, 900) #zaparkuj ruku dolu

    return True

  def execute_cb(self, goal):
    # helper variables
    success = True

    # action init
    self._feedback.status = 0

    # start executing the action
    if goal.command:
        success = getattr(self, 'actionCommand' + `goal.command`)(goal)

    if success:
      self._result.msElapsed = 100000
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)


if __name__ == '__main__':
  rospy.init_node('robik_action_move')
  MoveAction(rospy.get_name())
  pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=0)
  pub_arm_control = rospy.Publisher('/robik_arm_control', ArmControl, queue_size=0)
  pub_generic_control = rospy.Publisher('/robik_generic_control', GenericControl, queue_size=0)

  global soundhandle
  soundhandle = SoundClient()

  rospy.spin()
