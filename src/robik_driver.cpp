#include <boost/assign/list_of.hpp>
#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "robik_api.h"
#include "robik_state.h"
#include "robik_driver.h"
#include "robik_util.h"
///#include "robik_move.h"

//Messages
#include "sensor_msgs/LaserScan.h"
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
ros::Publisher pub_lidar;
ros::Publisher pub_odom;
ros::Publisher pub_ai;

sensor_msgs::LaserScan laserscan_msg;
sensor_msgs::LaserScan lidar_msg;


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
	//pub_laser.publish(laserscan_msg);  at se mi to neplete do kinectu

	//lidar
	lidar_msg.header.stamp = ros::Time::now();
	lidar_msg.header.frame_id = "lidar_link";
	lidar_msg.angle_min = 0.0; //[radians], 0=straight
	lidar_msg.angle_max = 2.0 * M_PI / 360 * msg.lidar_data.size();
	lidar_msg.angle_increment = 2.0 * M_PI / 360.0;
	lidar_msg.time_increment = msg.lidar_speed*2; //in 2 rpm steps
	lidar_msg.range_min = 0.06; //[m]
	lidar_msg.range_max = 5.0; //[m]
	lidar_msg.ranges.reserve(msg.lidar_data.size());
	for (int i = 0; i < msg.lidar_data.size(); i++) {
		lidar_msg.ranges[i] = (float)(msg.lidar_data[i] * 2) / 100.0; //reported in 2cm steps
	}
	pub_lidar.publish(lidar_msg);

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

////////////////// Main //////////////////

int main(int argc, char **argv) {

	ros::init(argc, argv, "robik");

	ros::NodeHandle n;

	//advertise topics to arduino
	pub_generic_control = n.advertise<robik::GenericControl>("robik_generic_control", 100);

	ros::Subscriber sub_status = n.subscribe("robik_status", 1000, statusCallback);

	//advertise driver services
	ros::ServiceServer service_setParkingPhase = n.advertiseService("setParkingPhase", setParkingPhase);

	//advertise topics from driver
	pub_laser = n.advertise<sensor_msgs::LaserScan>("laser_data", 100);
	pub_lidar = n.advertise<sensor_msgs::LaserScan>("lidar_data", 100);
	pub_ai = n.advertise<std_msgs::String>("robik_ai", 100);

//	ros::Subscriber sub_head = n.subscribe("head_twist", 10, headCallback); //TODO add camera oko

//ros::spin();
	ros::Rate loop_rate(20);

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
