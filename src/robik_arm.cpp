#include "ros/ros.h"
#include "robik_arm.h"
#include "robik_api.h"
#include "robik_driver.h"
#include "robik_util.h"
#include "robik/GenericStatus.h"

#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>

using namespace cv;

sensor_msgs::JointState arm_state;
sensor_msgs::JointState arm_origin;  //start position of arm control
sensor_msgs::JointState arm_target;  //desired position of arm control

robik::ArmControl arm_control_msg;   //message to be send to arduino, reusing memory

void init_joint_state_message(sensor_msgs::JointState *msg);
void init_arm_control_message();
void arm_kalman_init();


KalmanFilter KF(5, 5, 5);  //dynam, measure, control


void arm_init() {
	init_joint_state_message(&arm_state);
	init_joint_state_message(&arm_origin);
	init_joint_state_message(&arm_target);
	init_arm_control_message();

	arm_kalman_init();
}

/**
* init message that is published from driver, contains arm joint state in radians
*/
void init_joint_state_message(sensor_msgs::JointState *msg) {
	msg->header.frame_id = "base_link";
	msg->name.resize(5);
	msg->position.resize(5);
	msg->name[0] = "yaw_joint";
	msg->name[1] = "shoulder_joint";
	msg->name[2] = "elbow_joint";
	msg->name[3] = "roll_joint";
	msg->name[4] = "clamp_joint";
}

void init_arm_control_message() {
	arm_control_msg.header.frame_id = "base_link";
	arm_control_msg.header.stamp = ros::Time::now();
	arm_control_msg.arm_yaw = 0;
	arm_control_msg.arm_shoulder = 0;
	arm_control_msg.arm_elbow = 0;
	arm_control_msg.arm_roll = 0;
	arm_control_msg.arm_clamp = 0;
}

void arm_kalman_init() {
    // intialization of KF...
    setIdentity(KF.transitionMatrix);  //model prechodu stavu (pada dolu)
    setIdentity(KF.controlMatrix);

    KF.statePost = (Mat_<float>(5,1) << ARM_RES_INIT_YAW, ARM_RES_INIT_SHOULDER, ARM_RES_INIT_ELBOW, ARM_RES_INIT_ROLL, ARM_RES_INIT_CLAMP);
    setIdentity(KF.measurementMatrix);

    setIdentity(KF.processNoiseCov, Scalar::all(1));
    setIdentity(KF.measurementNoiseCov, Scalar::all(9));  //cim mensi, tim presnejsi mereni
    setIdentity(KF.errorCovPost, Scalar::all(.1));

}


/**
* map message from Arduino that is then published by driver, contains arm joint state in radians
*/
void arm_set_joint_state(const robik::GenericStatus *msg) {

	Mat prediction = KF.predict((Mat_<float>(5,1) << 0, 0, 0, 0, 0));  //controlMatrix  //TODO put true values here
	Mat estimated = KF.correct((Mat_<float>(5,1) << msg->arm_yaw, msg->arm_shoulder, msg->arm_elbow, msg->arm_roll, msg->arm_clamp));  //measurementMatrix

	arm_state.header.stamp = ros::Time::now();
	arm_state.position[0] = map_check_inf(estimated.at<float>(0), ARM_RES_MIN_YAW, ARM_RES_MAX_YAW, ARM_DEG_MIN_YAW, ARM_DEG_MAX_YAW) * DEG_TO_RAD;
	arm_state.position[1] = map_check_inf(prediction.at<float>(1), ARM_RES_MIN_SHOULDER, ARM_RES_MAX_SHOULDER, ARM_DEG_MIN_SHOULDER, ARM_DEG_MAX_SHOULDER) * DEG_TO_RAD;
	arm_state.position[2] = map_check_inf(prediction.at<float>(2), ARM_RES_MIN_ELBOW, ARM_RES_MAX_ELBOW, ARM_DEG_MIN_ELBOW, ARM_DEG_MAX_ELBOW) * DEG_TO_RAD;
	arm_state.position[3] = map_check_inf(prediction.at<float>(3), ARM_RES_MIN_ROLL, ARM_RES_MAX_ROLL, ARM_DEG_MIN_ROLL, ARM_DEG_MAX_ROLL) * DEG_TO_RAD;
	arm_state.position[4] = map_check_inf(prediction.at<float>(4), ARM_RES_MIN_CLAMP, ARM_RES_MAX_CLAMP, ARM_DEG_MIN_CLAMP, ARM_DEG_MAX_CLAMP) * DEG_TO_RAD;
}

void set_joint_state_message(sensor_msgs::JointState *msg) {
	msg->header.frame_id = "base_link";
	msg->name.resize(5);
	msg->position.resize(5);
	msg->name[0] = "yaw_joint";
	msg->name[1] = "shoulder_joint";
	msg->name[2] = "elbow_joint";
	msg->name[3] = "roll_joint";
	msg->name[4] = "clamp_joint";
}

/**
* Publish arm commands to arduino. Info received from (web) control widget (/robik_arm_controller_joint_states topic)
* Recevies info in radians, converts to servo physical PWM signal values
*/
void arm_set_controller_state(const trajectory_msgs::JointTrajectory& msg) { //TODO if there will be more points planned in advance execute them one by one

  float val;

  val = JointTrajectory_getPosition(msg, "clamp_joint") * RAD_TO_DEG;
  if (fabs(val) < FLOAT_NAN) {
    arm_target.position[4] = map_unchecked(val, ARM_DEG_MIN_CLAMP, ARM_DEG_MAX_CLAMP, ARM_MIN_CLAMP, ARM_MAX_CLAMP);
  }
  val = JointTrajectory_getPosition(msg, "roll_joint") * RAD_TO_DEG;
  if (fabs(val) < FLOAT_NAN) {
    arm_target.position[3] = map_unchecked(val, ARM_DEG_MIN_ROLL, ARM_DEG_MAX_ROLL, ARM_MIN_ROLL, ARM_MAX_ROLL);
  }
  val = JointTrajectory_getPosition(msg, "elbow_joint") * RAD_TO_DEG;
  if (fabs(val) < FLOAT_NAN) {
    arm_target.position[2] = map_unchecked(val, ARM_DEG_MIN_ELBOW, ARM_DEG_MAX_ELBOW, ARM_MIN_ELBOW, ARM_MAX_ELBOW);
  }
  val = JointTrajectory_getPosition(msg, "shoulder_joint") * RAD_TO_DEG;
  if (fabs(val) < FLOAT_NAN) {
    arm_target.position[1] = map_unchecked(val, ARM_DEG_MIN_SHOULDER, ARM_DEG_MAX_SHOULDER, ARM_MIN_SHOULDER, ARM_MAX_SHOULDER);
  }
  val = JointTrajectory_getPosition(msg, "yaw_joint") * RAD_TO_DEG;
  if (fabs(val) < FLOAT_NAN) {
    arm_target.position[0] = map_unchecked(val, ARM_DEG_MIN_YAW, ARM_DEG_MAX_YAW, ARM_MIN_YAW, ARM_MAX_YAW);
  }
  arm_target.header.stamp = ros::Time::now() + msg.points[0].time_from_start; //JointTrajectory_getTimeFromStart(msg);//TODO zmatek prepromyslet //when should arm reach this point, defined as duraion from start

  arm_origin = arm_get_joint_state(); //populate from real position read from sensors            zmatek protoze vraci pointer
}

const robik::ArmControl* arm_get_arm_controll_command(){

	arm_control_msg.header.stamp = ros::Time::now();  //command valid at current time
	//TODO tady vypocitat aktualni controll prikaz, takze interpolace
	return &arm_control_msg;
}

void arm_set_arm_controll_command(){
//TODO ale nevim co
}

//provide data read from arm sensors
const sensor_msgs::JointState* arm_get_joint_state() {
	//TODO read from sensors current state at time of query
	return &arm_state;
}

float JointTrajectory_getPosition(const trajectory_msgs::JointTrajectory& msg, const char * joint_name) {

  for (int i = 0; i < msg.points.size(); i++) {
    const std::basic_string<char> strtmp = msg.joint_names[i];

    if (strtmp.compare(joint_name) == 0) {
      float val = msg.points[i].positions[0];
      if (val > 1000) val = 0;
      return val;
    }
  }
  return FLOAT_NAN + 1;
}

potrebuji?
//Returns time in milliseconds
int JointTrajectory_getTimeFromStart(const trajectory_msgs::JointTrajectory& msg) {
  if (msg.points.size() > 0) {
    ros::Duration dur = msg.points[0].time_from_start;
    uint32_t nsecs = (uint32_t)(dur.toNSec() / 1000000ll);
    if (nsecs < 50) //if time is too aggressive
      nsecs = 50;
    return nsecs;
  }
  return 1000;
}
potrebuji?
ros::Duration JointTrajectory_getDurationFromStart(const trajectory_msgs::JointTrajectory& msg) {
  if (msg.points.size() > 0) {
    ros::Duration dur = msg.points[0].time_from_start;
    uint32_t nsecs = (uint32_t)(dur.toNSec() / 1000000ll);
    if (nsecs < 50) //if time is too aggressive
      dur = ros::Duration(0, 50);
    return dur;
  }
  return ros::Duration(1, 0);
}
potrebuji?
double ros_time_diff(ros::Time before, ros::Time after) {
  int sec = after.sec - before.sec;
  int nsec = after.nsec - before.nsec;

  if (nsec < 0) {
    nsec = 1000 + nsec;
  }

  return (double) sec + (double) nsec / 1000000000;
}

//-1: t2<t1
// 0: t2==t1
// 1: t2>t1
int ros_time_cmp(ros::Time t1, ros::Time t2) {

  if (t1.sec == t2.sec && t1.nsec == t2.nsec)
    return 0;

  if (ros_time_diff(t1, t2) > 0)
    return 1;
  else
    return -1;
}

void checkClampAndStop() {

estimated_pos_t estimated_clamp_pos;
estimated_clamp_pos.orig_time = ros::Time::now() - ros::Duration(1.6);
estimated_clamp_pos.time_to_complete = 2;
estimated_clamp_pos.orig_pos = 100;
estimated_clamp_pos.target_pos = 300;
//////
  ros::Time now = ros::Time::now();

  // if (estimated_clamp_pos.orig_time < now && now < (estimated_clamp_pos.orig_time + estimated_clamp_pos.time_to_complete)) { //clamp is being moved
    double seconds_since_start = ros_time_diff(estimated_clamp_pos.orig_time, now);
    double progress = seconds_since_start / estimated_clamp_pos.time_to_complete;
    int estimatedClampPos = estimated_clamp_pos.orig_pos + ((estimated_clamp_pos.target_pos - estimated_clamp_pos.orig_pos) * progress);

    //ROS_INFO("seconds_since_start: %d", ros_time_cmp(now, now));


    // ultrasoundBack = progress;
    // estimated_clamp_pos.corrected = true;
    // armSetJointState(estimatedClampPos, 0, 0, 0, 0, 80);
  // }
}

