#include "ros/ros.h"

//Messages
#include "geometry_msgs/Twist.h"
#include "wiimote/State.h"
#include "sensor_msgs/Joy.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "control_msgs/JointTrajectoryControllerState.h"

//towards other nodes
ros::Publisher pub_cmdvel;
ros::Publisher pub_armcmd;

geometry_msgs::Twist cmdvel_msg;
trajectory_msgs::JointTrajectory armcmd_msg;
float armstate[5];  //including gripper


#define MOVE_LINEAR_SPEED 0.2
#define MOVE_ANGULAR_SPEED 0.5
#define MOVE_LINEAR_MULTIPLIER 2
#define ARM_STEP 0.2

////////////////// Callback for subscribers //////////////////

bool arm_enabled() {

  for (int i = 0; i < 4; i++) {
    if (abs(armstate[i]) > 0.00000000001) return true;
  }

  //enable arm power
  armcmd_msg.points[0].positions[3] = 0.2;
  armcmd_msg.points[0].time_from_start = ros::Duration(5.0);
  pub_armcmd.publish(armcmd_msg);
  return false;
}

//http://docs.ros.org/jade/api/wiimote/html/msg/State.html
void wiimoteStateCallback(const wiimote::State& msg) {
  
  //CMD VEL
  bool cmdvel = false;
  int fwd = 1;

  cmdvel_msg.linear.x = 0;
  cmdvel_msg.angular.z = 0;

  if (msg.buttons[msg.MSG_BTN_LEFT]) {  //UP
    cmdvel_msg.linear.x = MOVE_LINEAR_SPEED;
    cmdvel_msg.angular.z = 0;
    cmdvel = true;
  }
  if (msg.buttons[msg.MSG_BTN_RIGHT]) {  //DOWN
    cmdvel_msg.linear.x = -MOVE_LINEAR_SPEED;
    cmdvel_msg.angular.z = 0;
    cmdvel = true;
    fwd = -1;
  }
  if (msg.buttons[msg.MSG_BTN_UP]) {  //LEFT
    cmdvel_msg.angular.z = fwd * MOVE_ANGULAR_SPEED;
    if (!cmdvel) cmdvel_msg.linear.x = 0;
    cmdvel = true;
  }
  if (msg.buttons[msg.MSG_BTN_DOWN]) {  //RIGHT
    cmdvel_msg.angular.z = -1 * fwd * MOVE_ANGULAR_SPEED;
    if (!cmdvel) cmdvel_msg.linear.x = 0;
    cmdvel = true;
  }

  if (cmdvel && msg.buttons[msg.MSG_BTN_MINUS]) { //B run faster
    cmdvel_msg.linear.x *= MOVE_LINEAR_MULTIPLIER;
    cmdvel_msg.angular.z *= MOVE_LINEAR_MULTIPLIER;
  }

  if (cmdvel) {
    pub_cmdvel.publish(cmdvel_msg);
  }

  //ARM CMD
  bool armcmd = false;
  for (int i = 0; i < 4; i++) {
    armcmd_msg.points[0].positions[i] = armstate[i];
  }

  if (msg.buttons[msg.MSG_BTN_A]) { //minus
    armcmd_msg.points[0].positions[0] -= ARM_STEP;
    armcmd = true;
  }
  if (msg.buttons[msg.MSG_BTN_B]) { //plus
    armcmd_msg.points[0].positions[0] += ARM_STEP;
    armcmd = true;
  }

  if (armcmd && arm_enabled()) {
    armcmd_msg.header.stamp = ros::Time::now();
    armcmd_msg.points[0].time_from_start = ros::Duration(0.01);
    pub_armcmd.publish(armcmd_msg);
  }

}

void joyCallback(const sensor_msgs::Joy& msg) {

}

void armStateCallback(const control_msgs::JointTrajectoryControllerState& msg) {

  for (int i = 0; i < 4; i++) {
    armstate[i] = msg.actual.positions[i];
  }

}

////////////////// Main //////////////////

int main(int argc, char **argv) {

	ros::init(argc, argv, "robik_teleop");
	ros::NodeHandle n;

	//init messages
	armcmd_msg.points.resize(1);
	armcmd_msg.joint_names.resize(4);
	armcmd_msg.joint_names[0] = "yaw_joint";
	armcmd_msg.joint_names[1] = "shoulder_joint";
	armcmd_msg.joint_names[2] = "elbow_joint";
	armcmd_msg.joint_names[3] = "roll_joint";
	armcmd_msg.points[0].positions.resize(4);

	//advertise publishing topics
	pub_cmdvel = n.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 2);

	pub_armcmd = n.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 2);

	//subscribe
	ros::Subscriber sub_wiimote_state = n.subscribe("/wiimote/state", 100, wiimoteStateCallback);
	ros::Subscriber sub_joy = n.subscribe("/joy", 100, joyCallback);

	ros::Subscriber sub_arm_state = n.subscribe("/arm_controller_state", 100, armStateCallback);

	//enter spin loop
	ros::Rate loop_rate(100);
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
