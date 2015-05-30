#include "ros/ros.h"
#include "robik_arm.h"
#include "robik_api.h"
#include "robik_driver.h"
#include "robik_util.h"
#include "robik/GenericStatus.h"

ros::Publisher pub_arm_control;

sensor_msgs::JointState arm_state;

/**
* init message that is published from driver, contains arm joint state in radians
*/
void init_joint_state_message(sensor_msgs::JointState *arm_state) {
	arm_state->header.frame_id = "base_link";
	arm_state->name.resize(5);
	arm_state->position.resize(5);
	arm_state->name[0] = "yaw_joint";
	arm_state->name[1] = "shoulder_joint";
	arm_state->name[2] = "elbow_joint";
	arm_state->name[3] = "roll_joint";
	arm_state->name[4] = "clamp_joint";
}

/**
* map message from Arduino that is then published by driver, contains arm joint state in radians
*/
void map_joint_state_message(const robik::GenericStatus *msg, sensor_msgs::JointState *arm_state) {
	arm_state->header.stamp = ros::Time::now();
	arm_state->position[0] = map_check_inf(msg->arm_yaw, ARM_RES_MIN_YAW, ARM_RES_MAX_YAW, ARM_DEG_MIN_YAW, ARM_DEG_MAX_YAW) * DEG_TO_RAD;
	arm_state->position[1] = map_check_inf(msg->arm_shoulder, ARM_RES_MIN_SHOULDER, ARM_RES_MAX_SHOULDER, ARM_DEG_MIN_SHOULDER, ARM_DEG_MAX_SHOULDER) * DEG_TO_RAD;
	arm_state->position[2] = map_check_inf(msg->arm_elbow, ARM_RES_MIN_ELBOW, ARM_RES_MAX_ELBOW, ARM_DEG_MIN_ELBOW, ARM_DEG_MAX_ELBOW) * DEG_TO_RAD;
	arm_state->position[3] = map_check_inf(msg->arm_roll, ARM_RES_MIN_ROLL, ARM_RES_MAX_ROLL, ARM_DEG_MIN_ROLL, ARM_DEG_MAX_ROLL) * DEG_TO_RAD;
	arm_state->position[4] = map_check_inf(msg->arm_clamp, ARM_RES_MIN_CLAMP, ARM_RES_MAX_CLAMP, ARM_DEG_MIN_CLAMP, ARM_DEG_MAX_CLAMP) * DEG_TO_RAD;
}

/**
* Publish arm commands to arduino. Info received from (web) control widget (/robik_arm_controller_joint_states topic)
* Recevies info in radians, converts to servo physical PWM signal values
*/
void armCallback(const trajectory_msgs::JointTrajectory& msg) {

  robik::ArmControl arm_msg;
  arm_msg.header.stamp = ros::Time::now();
  float val = JointTrajectory_getPosition(msg, "clamp_joint") * RAD_TO_DEG;
  if (fabs(val) < FLOAT_NAN) {
    arm_msg.arm_clamp = map_unchecked(val, ARM_DEG_MIN_CLAMP, ARM_DEG_MAX_CLAMP, ARM_MIN_CLAMP, ARM_MAX_CLAMP);
  }
  val = JointTrajectory_getPosition(msg, "roll_joint") * RAD_TO_DEG;
  if (fabs(val) < FLOAT_NAN) {
    arm_msg.arm_roll = map_unchecked(val, ARM_DEG_MIN_ROLL, ARM_DEG_MAX_ROLL, ARM_MIN_ROLL, ARM_MAX_ROLL);
  }
  val = JointTrajectory_getPosition(msg, "elbow_joint") * RAD_TO_DEG;
  if (fabs(val) < FLOAT_NAN) {
    arm_msg.arm_elbow = map_unchecked(val, ARM_DEG_MIN_ELBOW, ARM_DEG_MAX_ELBOW, ARM_MIN_ELBOW, ARM_MAX_ELBOW);
  }
  val = JointTrajectory_getPosition(msg, "shoulder_joint") * RAD_TO_DEG;
  if (fabs(val) < FLOAT_NAN) {
    arm_msg.arm_shoulder = map_unchecked(val, ARM_DEG_MIN_SHOULDER, ARM_DEG_MAX_SHOULDER, ARM_MIN_SHOULDER, ARM_MAX_SHOULDER);
  }
  val = JointTrajectory_getPosition(msg, "yaw_joint") * RAD_TO_DEG;
  if (fabs(val) < FLOAT_NAN) {
    arm_msg.arm_yaw = map_unchecked(val, ARM_DEG_MIN_YAW, ARM_DEG_MAX_YAW, ARM_MIN_YAW, ARM_MAX_YAW);
  }
  arm_msg.time_to_complete = JointTrajectory_getTimeFromStart(msg);
  pub_arm_control.publish(arm_msg);

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
  ros::Time now = ros::Time::now();  //!!bylo nh.now()

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
