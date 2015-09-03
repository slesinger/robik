/*
 * robik_util.h
 *
 *  Created on: Oct 28, 2014
 *      Author: honza
 * This header file contains local defines
 */

#ifndef ROBIK_UTIL_H_
#define ROBIK_UTIL_H_

#include "ros/ros.h"

#define _USE_MATH_DEFINES

#define RAD_TO_DEG 180 / M_PI
#define DEG_TO_RAD M_PI / 180
#define FLOAT_NAN 999


double map_unchecked(double value, double fromLow, double fromHigh, double toLow, double toHigh);
double map_check_inf(double value, double fromLow, double fromHigh, double toLow, double toHigh);
double map(double value, double fromLow, double fromHigh, double toLow, double toHigh);


#endif /* ROBIK_UTIL_H_ */
