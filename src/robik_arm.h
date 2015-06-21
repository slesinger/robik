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

//from arduino
void arm_set_joint_state(const robik::GenericStatus *msg);  //set received joint state from arduino sensors to arm_state
const sensor_msgs::JointState* arm_get_joint_state(); //read arm_state

//to arduino
void arm_controller_set_trajectory(const trajectory_msgs::JointTrajectory& msg);  //receive trajectory from web or moveit
const robik::ArmControl* arm_get_arm_control_command();  //get message to be sent to arduino

float JointTrajectory_getPosition(const trajectory_msgs::JointTrajectory& msg, const char * joint_name);
int JointTrajectory_getTimeFromStart(const trajectory_msgs::JointTrajectory& msg);
void checkClampAndStop();

#endif /* ROBIK_ARM_H_ */
