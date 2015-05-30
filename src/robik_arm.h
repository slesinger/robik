/*
 * robik_arm.h
 *
 *  Created on: Oct 28, 2014
 *      Author: honza
 */

#ifndef ROBIK_ARM_H_
#define ROBIK_ARM_H_

#include "robik/ArmControl.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include "robik/GenericStatus.h"

extern ros::Publisher pub_arm_control;

extern sensor_msgs::JointState arm_state;


void init_joint_state_message(sensor_msgs::JointState *arm_state);
void map_joint_state_message(const robik::GenericStatus *msg, sensor_msgs::JointState *arm_state);
void armCallback(const trajectory_msgs::JointTrajectory& msg);
float JointTrajectory_getPosition(const trajectory_msgs::JointTrajectory& msg, const char * joint_name);
int JointTrajectory_getTimeFromStart(const trajectory_msgs::JointTrajectory& msg);
void checkClampAndStop();

struct estimated_pos_t {
  ros::Time orig_time;
  long time_to_complete;
  int orig_pos;
  int target_pos;
  bool corrected;
};


#endif /* ROBIK_ARM_H_ */
