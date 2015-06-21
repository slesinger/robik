#include <boost/assign/list_of.hpp>
#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "robik_api.h"
#include "robik_state.h"
#include "robik_driver.h"
#include "robik_util.h"
#include "robik_arm.h"
#include "robik_move.h"

//Messages
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include "robik/GenericStatus.h"
#include "robik/GenericControl.h"

//Services
#include "robik/setParkingPhase.h"

//towards Arduino
ros::Publisher pub_generic_control;
ros::Publisher pub_arm_control;

//towards other nodes
ros::Publisher pub_laser;
ros::Publisher pub_odom;
ros::Publisher pub_imu;
ros::Publisher pub_arm;
ros::Publisher pub_ai;

sensor_msgs::LaserScan laserscan_msg;

//local function declarations
void robik_arm_controller_joint_states_callback(const trajectory_msgs::JointTrajectory& trajectory_msg);

////////////////// Services //////////////////

bool setParkingPhase(robik::setParkingPhase::Request  &req, robik::setParkingPhase::Response &resp) {

	robik::GenericControl gen_msg;
	gen_msg.header.stamp = ros::Time::now();
	gen_msg.gen_operation = OPERATION_SET_PARKING_PHASE;
	gen_msg.gen_param1 = req.phase;
	pub_generic_control.publish(gen_msg);
	resp.status = 0;
	return true;
}


void publish_ai(const std::string &command){
  std_msgs::String ai_msg;
  ai_msg.data = command;
  pub_ai.publish(ai_msg);
}

////////////////// Callback for subscribers //////////////////

void statusCallback(const robik::GenericStatus& msg) {
checkClampAndStop();  //TODO
	//bumper
	bumperFront = msg.bumper_front;

	//menu_controls
	uint8_t mc = msg.menu_controls;
	if ((mc & MASK_MENU_MENU) == MASK_MENU_MENU)
	  publish_ai("menu menu");
	else if ((mc & MASK_MENU_OK) == MASK_MENU_OK)
	  publish_ai("menu ok");
	else if ((mc & MASK_MENU_CANCEL) == MASK_MENU_CANCEL)
	  publish_ai("menu cancel");
	else if ((mc & MASK_MENU_UP) == MASK_MENU_UP)
	  publish_ai("menu up");
	else if ((mc & MASK_MENU_DOWN) == MASK_MENU_DOWN)
	  publish_ai("menu down");
	else if ((mc & (MASK_MENU_UP | MASK_MENU_DOWN)) == (MASK_MENU_UP | MASK_MENU_DOWN))
	  publish_ai("menu updown");

	//odom
	// odomMillisSinceLastUpdate = msg.odom_millisSinceLastUpdate;

	//sonar
	laserscan_msg.header.stamp = ros::Time::now();
	laserscan_msg.header.frame_id = "laser_link";
	laserscan_msg.angle_min = -7 * M_PI / 180; //[radians], 0=straight
	laserscan_msg.angle_max = 7 * M_PI / 180;
	laserscan_msg.angle_increment = 2 * 7 * M_PI / 180;
	laserscan_msg.time_increment = 0;
	laserscan_msg.range_min = 1.0 / 100; //1cm
	laserscan_msg.range_max = 200.0 / 100; //200cm
	laserscan_msg.scan_time = 0.1;
	laserscan_msg.ranges.resize((int) 2);
	laserscan_msg.ranges[0] = msg.ultrasound_Back / 100;
	laserscan_msg.ranges[1] = msg.ultrasound_Back / 100;
	pub_laser.publish(laserscan_msg);

	//odometry
	static tf::TransformBroadcaster odom_broadcaster;
	ros::Time current_time = ros::Time::now();
	double vx = 0.0;
	double vy = 0.0;
	double vth = 0.0;
	processOdomTicks(&vx, &vy, &vth, msg.odom_ticksLeft, msg.odom_ticksRight, msg.odom_millisSinceLastUpdate);

	//ignore odom_theta calculated from wheel odometry and use IMU compass absolution heading instead
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_theta);

	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = odom_x;
	odom_trans.transform.translation.y = odom_y;
	odom_trans.transform.translation.z = 0.0;
	// odom_trans.transform.rotation.x = xmsg.imu_orientation_quaternion_x;
	// odom_trans.transform.rotation.y = xmsg.imu_orientation_quaternion_y;
	// odom_trans.transform.rotation.z = xmsg.imu_orientation_quaternion_z;
	// odom_trans.transform.rotation.w = xmsg.imu_orientation_quaternion_w;
	odom_trans.transform.rotation = odom_quat;
	//send the transform
	odom_broadcaster.sendTransform(odom_trans);

	//publish odometry message over ROS
	nav_msgs::Odometry robik_odom;
	robik_odom.header.stamp = current_time;
	robik_odom.header.frame_id = "odom";
	robik_odom.child_frame_id = "base_link";

	//set the position
	robik_odom.pose.pose.position.x = odom_x;
	robik_odom.pose.pose.position.y = odom_y;
	robik_odom.pose.pose.position.z = 0.0;

	// robik_odom.pose.pose.orientation.x = xmsg.imu_orientation_quaternion_x;
	// robik_odom.pose.pose.orientation.y = xmsg.imu_orientation_quaternion_y;
	// robik_odom.pose.pose.orientation.z = xmsg.imu_orientation_quaternion_z;
	// robik_odom.pose.pose.orientation.w = xmsg.imu_orientation_quaternion_w;
	robik_odom.pose.pose.orientation = odom_quat;

	//set the velocity
	robik_odom.twist.twist.linear.x = vx;
	robik_odom.twist.twist.linear.y = vy;
	//vth is taken from wheel odometry
	robik_odom.twist.twist.angular.z = vth;
// the values on diagonal are: X, Y, Z, Roll, Pitch, Yaw; small value means accuracy
	robik_odom.twist.covariance =  boost::assign::list_of(0.0001) (0)      (0)   (0)   (0)   (0)
                                                       (0)      (0.0001) (0)   (0)   (0)   (0)
                                                       (0)      (0)      (1e6) (0)   (0)   (0)
                                                       (0)      (0)      (0)   (1e6) (0)   (0)
                                                       (0)      (0)      (0)   (0)   (1e6) (0)
                                                       (0)      (0)      (0)   (0)   (0)   (0.003) ;

	pub_odom.publish(robik_odom);

	//arm joint state
	arm_set_joint_state(&msg);

	//IMU
	sensor_msgs::Imu robik_imu;
	robik_imu.header.stamp = current_time;
	robik_imu.header.frame_id = "base_link";
	//robik_imu.orientation.w = msg.imu_orientation_quaternion_w;
	//robik_imu.orientation.x = msg.imu_orientation_quaternion_x;
	//robik_imu.orientation.y = msg.imu_orientation_quaternion_y;
	//robik_imu.orientation.z = msg.imu_orientation_quaternion_z;
	robik_imu.angular_velocity.x = msg.imu_angular_velocity_v3_x[0];
	robik_imu.angular_velocity.y = msg.imu_angular_velocity_v3_y[0];
	robik_imu.angular_velocity.z = msg.imu_angular_velocity_v3_z[0];
	robik_imu.linear_acceleration.x = msg.imu_linear_acceleration_v3_x[0];
	robik_imu.linear_acceleration.y = msg.imu_linear_acceleration_v3_y[0];
	robik_imu.linear_acceleration.z = msg.imu_linear_acceleration_v3_z[0];
	pub_imu.publish(robik_imu);

	//Log
	//ROS_INFO("Arduino: %s", msg.log_message.c_str());

	/*ROS_INFO("RobikStatus: { bumperFront: %d, odomTicksLeft: %d, odomTicksRight: %d, sonar: %f, vx: %f, vy: %f, vth: %f, pos_x: %f, pos_y: %f, orientation: %f }",
			bumperFront, odomTicksLeft, odomTicksRight, ultrasoundBack, vx, vy,
			vth, odom_x, odom_y, odom_theta); */
}

void headCallback(const geometry_msgs::Twist& msg) {
	float pitch = msg.linear.x / 10; //correction based on teleop widget; vel to be max 1
	float yaw = msg.angular.z / 2; //correction based on teleop widget; th to ba max pi/2

	if (pitch > 1)
		pitch = 1;
	if (pitch < -1)
		pitch = -1;
	if (yaw > 1)
		yaw = 1;
	if (yaw < -1)
		yaw = -1;

	robik::GenericControl gen_msg;
	gen_msg.header.stamp = ros::Time::now();
	gen_msg.gen_operation = OPERATION_HEAD_POSE;
	gen_msg.gen_param1 = yaw * 1000;
	gen_msg.gen_param2 = pitch * 1000;
//x	pub_generic_control.publish(gen_msg);
}

void robik_arm_controller_joint_states_callback(const trajectory_msgs::JointTrajectory& trajectory_msg) {
	arm_controller_set_trajectory(trajectory_msg);
	const robik::ArmControl *arm_msg;
	arm_msg = arm_get_arm_control_command();
	pub_arm_control.publish(*arm_msg);
}

////////////////// Main //////////////////

int main(int argc, char **argv) {

	ros::init(argc, argv, "robik");

	ros::NodeHandle n;

	arm_init();

	//advertise topics to arduino
	pub_generic_control = n.advertise<robik::GenericControl>("robik_generic_control", 100);
	pub_arm_control = n.advertise<robik::ArmControl>("robik_arm_control", 100);
	pub_velocity_control = n.advertise<geometry_msgs::Twist>("robik_velocity_control", 100);

	ros::Subscriber sub_status = n.subscribe("robik_status", 1000, statusCallback);

	//advertise driver services
	ros::ServiceServer service_setParkingPhase = n.advertiseService("setParkingPhase", setParkingPhase);

	//advertise topics from driver
	pub_laser = n.advertise<sensor_msgs::LaserScan>("laser_data", 100);
	pub_odom = n.advertise<nav_msgs::Odometry>("odom", 100);
	pub_imu = n.advertise<sensor_msgs::Imu>("imu_data", 100);
	pub_arm = n.advertise<sensor_msgs::JointState>("joint_states", 100);
	pub_ai = n.advertise<std_msgs::String>("robik_ai", 100);

	ros::Subscriber sub_cmdvel = n.subscribe("cmd_vel", 10, cmdvelCallback);
//	ros::Subscriber sub_head = n.subscribe("head_twist", 10, headCallback); //TODO add camera oko
	ros::Subscriber sub_arm = n.subscribe("robik_arm_controller_joint_states", 10, robik_arm_controller_joint_states_callback);

//ros::spin();
	ros::Rate loop_rate(20);
	while (ros::ok()) {
		pub_arm.publish(*(arm_get_joint_state()));   //publish arm joint state regularly
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
