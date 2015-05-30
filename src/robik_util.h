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


float map_unchecked(float value, float fromLow, float fromHigh, float toLow, float toHigh);
float map_check_inf(float value, float fromLow, float fromHigh, float toLow, float toHigh);
float map(float value, float fromLow, float fromHigh, float toLow, float toHigh);


#endif /* ROBIK_UTIL_H_ */
