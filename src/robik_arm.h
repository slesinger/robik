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


void arm_init(); //constructor
void init_joint_state_message(sensor_msgs::JointState *msg);

//from arduino
void arm_set_joint_state(const robik::GenericStatus *msg);  //set received joint state from arduino sensors to arm_state
void arm_get_joint_state(sensor_msgs::JointState *dest); //read arm_state

//to arduino
robik::ArmControl* arm_get_arm_control_command();  //get message to be sent to arduino

float JointTrajectory_getPosition(const trajectory_msgs::JointTrajectory& msg, const char * joint_name);
int JointTrajectory_getTimeFromStart(const trajectory_msgs::JointTrajectory& msg);
void checkClampAndStop();

#endif /* ROBIK_ARM_H_ */
