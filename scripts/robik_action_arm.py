#! /usr/bin/env python

import roslib; roslib.load_manifest('robik')
from time import sleep
import rospy
import actionlib
import robik
import robik.msg
import trajectory_msgs.msg

class ArmAction(object):
  # create messages that are used to publish feedback/result
  _feedback = robik.msg.armFeedback()
  _result   = robik.msg.armResult()

  effort = 50.0


  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, robik.msg.armAction, execute_cb=self.execute_cb)
    self._as.start()


  #zaparkuj
  def actionCommand1(self,goal):
      rospy.loginfo('%s: I''m going to execute arm action %i during %i ms, %i seq0' % (self._action_name, goal.command, goal.opTimeMs, self._feedback.status))

      armTwistPublisher = rospy.Publisher('/robik_arm_controller_joint_states', trajectory_msgs.msg.JointTrajectory, queue_size=50)

      #zvedni pazi
      durS = goal.opTimeMs * 0.5 / 1000
      armTwistMsg = trajectory_msgs.msg.JointTrajectory()
      armTwistMsg.header.stamp = rospy.Time.now()
      armTwistMsg.joint_names = ["shoulder_joint"]
      armTwistMsg.points = [
                                trajectory_msgs.msg.JointTrajectoryPoint([0.872664626], [], [], [ArmAction.effort], rospy.Duration(durS))
                             ]
      armTwistPublisher.publish(armTwistMsg)
      sleep(durS)

      #zaparkuj
      durS = goal.opTimeMs * 0.5 / 1000
      armTwistMsg = trajectory_msgs.msg.JointTrajectory()
      armTwistMsg.joint_names = ["yaw_joint", "shoulder_joint", "elbow_joint", "roll_joint", "clamp_joint"]
      armTwistMsg.points = [
                                trajectory_msgs.msg.JointTrajectoryPoint([-1.4311699870], [], [], [ArmAction.effort], rospy.Duration(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([ 0.7853981634], [], [], [ArmAction.effort], rospy.Duration(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([-1.9198621770], [], [], [ArmAction.effort], rospy.Duration(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([ 0.4886921906], [], [], [ArmAction.effort], rospy.Duration(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([-0.1396263402], [], [], [ArmAction.effort], rospy.Duration(durS))
                             ]
      armTwistPublisher.publish(armTwistMsg)
      sleep(durS)

      # publish the feedback
      self._feedback.status = 0
      self._as.publish_feedback(self._feedback)

      return True

  #priprav na chnapnuti
  def actionCommand2(self,goal):
      rospy.loginfo('%s: I''m going to execute arm action %i during %i ms, %i seq0' % (self._action_name, goal.command, goal.opTimeMs, self._feedback.status))

      armTwistPublisher = rospy.Publisher('/robik_arm_controller_joint_states', trajectory_msgs.msg.JointTrajectory, queue_size=20)

      #zvedni pazi
      durS = goal.opTimeMs * 0.3 / 1000
      armTwistMsg = trajectory_msgs.msg.JointTrajectory()
      armTwistMsg.joint_names = ["shoulder_joint"]
      armTwistMsg.points = [
                                trajectory_msgs.msg.JointTrajectoryPoint([0.872664626], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS))
                             ]
      armTwistPublisher.publish(armTwistMsg)
      sleep(durS)

      #otoc se
      durS = goal.opTimeMs * 0.3 / 1000
      armTwistMsg = trajectory_msgs.msg.JointTrajectory()
      armTwistMsg.joint_names = ["yaw_joint", "clamp_joint"]
      armTwistMsg.points = [
                                trajectory_msgs.msg.JointTrajectoryPoint([0.01], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([ 0.6981317008], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS))
                             ]
      armTwistPublisher.publish(armTwistMsg)
      sleep(durS)

      #dolu
      durS = goal.opTimeMs * 0.4 / 1000
      armTwistMsg = trajectory_msgs.msg.JointTrajectory()
      armTwistMsg.joint_names = ["shoulder_joint", "elbow_joint", "roll_joint", "clamp_joint"]
      armTwistMsg.points = [
                                trajectory_msgs.msg.JointTrajectoryPoint([ 0.4], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([-1.7], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([ 0.01], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([ 0.6981317008], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS))
                             ]
      armTwistPublisher.publish(armTwistMsg)
      sleep(durS)

      # publish the feedback
      self._feedback.status = 0
      self._as.publish_feedback(self._feedback)

      return True

  #chnapni
  def actionCommand5(self,goal):
      rospy.loginfo('%s: I''m going to execute arm action %i during %i ms, %i seq0' % (self._action_name, goal.command, goal.opTimeMs, self._feedback.status))

      armTwistPublisher = rospy.Publisher('/robik_arm_controller_joint_states', trajectory_msgs.msg.JointTrajectory, queue_size=50)

      #dolu
      durS = goal.opTimeMs * 0.3 / 1000
      armTwistMsg = trajectory_msgs.msg.JointTrajectory()
      armTwistMsg.joint_names = ["shoulder_joint"]
      armTwistMsg.points = [
                                trajectory_msgs.msg.JointTrajectoryPoint([0.3], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                             ]
      armTwistPublisher.publish(armTwistMsg)
      sleep(durS)

      #stiskni prsty
      durS = goal.opTimeMs * 0.3 / 1000
      armTwistMsg = trajectory_msgs.msg.JointTrajectory()
      armTwistMsg.joint_names = ["clamp_joint"]
      armTwistMsg.points = [
                                trajectory_msgs.msg.JointTrajectoryPoint([ 0.01], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                             ]
      armTwistPublisher.publish(armTwistMsg)
      sleep(durS)

      #nahoru
      durS = goal.opTimeMs * 0.4 / 1000
      armTwistMsg = trajectory_msgs.msg.JointTrajectory()
      armTwistMsg.joint_names = ["shoulder_joint"]
      armTwistMsg.points = [
                                trajectory_msgs.msg.JointTrajectoryPoint([0.4], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS))
                             ]
      armTwistPublisher.publish(armTwistMsg)
      sleep(durS)

      # publish the feedback
      self._feedback.status = 0
      self._as.publish_feedback(self._feedback)

      return True

  #ukaz
  def actionCommand6(self,goal):
      rospy.loginfo('%s: I''m going to execute arm action %i during %i ms, %i seq0' % (self._action_name, goal.command, goal.opTimeMs, self._feedback.status))

      armTwistPublisher = rospy.Publisher('/robik_arm_controller_joint_states', trajectory_msgs.msg.JointTrajectory, queue_size=50)

      #pazi nahoru
      durS = goal.opTimeMs * 0.3 / 1000
      armTwistMsg = trajectory_msgs.msg.JointTrajectory()
      armTwistMsg.joint_names = ["shoulder_joint"]
      armTwistMsg.points = [
                                trajectory_msgs.msg.JointTrajectoryPoint([1.047197551], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                             ]
      armTwistPublisher.publish(armTwistMsg)
      sleep(durS)

      #ukaz
      durS = goal.opTimeMs * 0.7 / 1000
      armTwistMsg = trajectory_msgs.msg.JointTrajectory()
      armTwistMsg.joint_names = ["yaw_joint", "elbow_joint", "roll_joint", "clamp_joint"]
      armTwistMsg.points = [
                                trajectory_msgs.msg.JointTrajectoryPoint([ 0.00000], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([ 0.11], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([ 0.00], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([ 0.10], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS))
                             ]
      armTwistPublisher.publish(armTwistMsg)
      sleep(durS)

      # publish the feedback
      self._feedback.status = 0
      self._as.publish_feedback(self._feedback)

      return True

  #zamavej
  def actionCommand7(self,goal):
      rospy.loginfo('%s: I''m going to execute arm action %i during %i ms, %i seq0' % (self._action_name, goal.command, goal.opTimeMs, self._feedback.status))

      armTwistPublisher = rospy.Publisher('/robik_arm_controller_joint_states', trajectory_msgs.msg.JointTrajectory, queue_size=50)

      #mavni doleva
      durS = goal.opTimeMs * 0.5 / 1000
      armTwistMsg = trajectory_msgs.msg.JointTrajectory()
      armTwistMsg.joint_names = ["yaw_joint", "shoulder_joint", "elbow_joint", "roll_joint", "clamp_joint"]
      armTwistMsg.points = [
                                trajectory_msgs.msg.JointTrajectoryPoint([-1.570796327], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([ 1.919862177], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([ 0.3490658504], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([ 0.0], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([ 0.10], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS))
                             ]
      armTwistPublisher.publish(armTwistMsg)
      sleep(durS)

      #mavni doprava
      durS = goal.opTimeMs * 0.5 / 1000
      armTwistMsg = trajectory_msgs.msg.JointTrajectory()
      armTwistMsg.joint_names = ["yaw_joint", "shoulder_joint", "elbow_joint", "roll_joint", "clamp_joint"]
      armTwistMsg.points = [
                                trajectory_msgs.msg.JointTrajectoryPoint([-1.570796327], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([ 1.221730476], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([-0.3490658504], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([ 0.0], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([ 0.10], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS))
                             ]
      armTwistPublisher.publish(armTwistMsg)
      sleep(durS)
      # publish the feedback
      self._feedback.status = 0
      self._as.publish_feedback(self._feedback)

      return True

  #test case 1
  def actionCommand8(self,goal):
      rospy.loginfo('%s: I''m going to execute arm action %i during %i ms, %i seq0' % (self._action_name, goal.command, goal.opTimeMs, self._feedback.status))

      armTwistPublisher = rospy.Publisher('/robik_arm_controller_joint_states', trajectory_msgs.msg.JointTrajectory, queue_size=50)

      durS = goal.opTimeMs * 1 / 1000
      armTwistMsg = trajectory_msgs.msg.JointTrajectory()
      armTwistMsg.joint_names = ["yaw_joint", "shoulder_joint", "elbow_joint", "roll_joint", "clamp_joint"]
      armTwistMsg.points = [
                                trajectory_msgs.msg.JointTrajectoryPoint([ -1.570796327], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([  1.570796327], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([ -1.570796327], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([  0.000000000], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([  0.000000000], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS))
                             ]
      armTwistPublisher.publish(armTwistMsg)
      sleep(durS)
      # publish the feedback
      self._feedback.status = 0
      self._as.publish_feedback(self._feedback)

      return True

  #test case 2
  def actionCommand9(self,goal):
      rospy.loginfo('%s: I''m going to execute arm action %i during %i ms, %i seq0' % (self._action_name, goal.command, goal.opTimeMs, self._feedback.status))

      armTwistPublisher = rospy.Publisher('/robik_arm_controller_joint_states', trajectory_msgs.msg.JointTrajectory, queue_size=50)

      durS = goal.opTimeMs * 1 / 1000
      armTwistMsg = trajectory_msgs.msg.JointTrajectory()
      armTwistMsg.joint_names = ["yaw_joint", "shoulder_joint", "elbow_joint", "roll_joint", "clamp_joint"]
      armTwistMsg.points = [
                                trajectory_msgs.msg.JointTrajectoryPoint([  0.000000000], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([  0.000000000], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([  0.000000000], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([ -1.570796327], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS)),
                                trajectory_msgs.msg.JointTrajectoryPoint([  0.698131701], [], [], [ArmAction.effort], rospy.Duration.from_sec(durS))
                             ]
      armTwistPublisher.publish(armTwistMsg)
      sleep(durS)
      # publish the feedback
      self._feedback.status = 0
      self._as.publish_feedback(self._feedback)

      return True


  #1-zaparkuj; 2-prepare; 3-left; 4-right; 5-grasp; 6-ukaz; 7-dejpac; 8-load    
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
  rospy.init_node('action_arm')
  ArmAction(rospy.get_name())
  rospy.spin()

