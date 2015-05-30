/*
 * robik_move.h
 *
 *  Created on: Oct 28, 2014
 *      Author: honza
 */

#ifndef ROBIK_MOVE_H_
#define ROBIK_MOVE_H_

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"


extern ros::Publisher pub_velocity_control;

extern double odom_x;
extern double odom_y;
extern double odom_theta;

void processOdomTicks(double *vx, double *vy, double *vth,
    int odomTicksLeft, int odomTicksRight,
    unsigned int odom_millisSinceLastUpdate);

void cmdvelCallback(const geometry_msgs::Twist& msg);
void velocity_control(float vel, float th);

#endif /* ROBIK_MOVE_H_ */
