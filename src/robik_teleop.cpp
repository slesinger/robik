#include "ros/ros.h"

//Messages
#include "geometry_msgs/Twist.h"
#include "wiimote/State.h"
#include "sensor_msgs/Joy.h"

//towards other nodes
ros::Publisher pub_cmdvel;

geometry_msgs::Twist cmdvel_msg;

/*#define WIIBUTTON_UP
#define WIIBUTTON_DOWN
#define WIIBUTTON_LEFT
#define WIIBUTTON_RIGHT
*/
#define MOVE_LINEAR_SPEED 0.2
#define MOVE_ANGULAR_SPEED 0.5
#define MOVE_LINEAR_MULTIPLIER 2

////////////////// Callback for subscribers //////////////////

//http://docs.ros.org/jade/api/wiimote/html/msg/State.html
void wiimoteStateCallback(const wiimote::State& msg) {
  
  bool cmdvel = false;

  if (msg.buttons[msg.MSG_BTN_LEFT]) {  //UP
    cmdvel_msg.linear.x = MOVE_LINEAR_SPEED;
    cmdvel_msg.angular.z = 0;
    cmdvel = true;
  }
  if (msg.buttons[msg.MSG_BTN_RIGHT]) {  //DOWN
    cmdvel_msg.linear.x = -MOVE_LINEAR_SPEED;
    cmdvel_msg.angular.z = 0;
    cmdvel = true;
  }
  if (msg.buttons[msg.MSG_BTN_UP]) {  //LEFT
    cmdvel_msg.angular.z = -MOVE_ANGULAR_SPEED;
    if (!cmdvel) cmdvel_msg.linear.x = 0;
    cmdvel = true;
  }
  if (msg.buttons[msg.MSG_BTN_DOWN]) {  //RIGHT
    cmdvel_msg.angular.z = MOVE_ANGULAR_SPEED;
    if (!cmdvel) cmdvel_msg.linear.x = 0;
    cmdvel = true;
  }

  if (cmdvel && msg.buttons[msg.MSG_BTN_MINUS]) { //B run faster
    cmdvel_msg.linear.x *= MOVE_LINEAR_MULTIPLIER;
  }

  if (cmdvel) {
    pub_cmdvel.publish(cmdvel_msg);
  }

}

void joyCallback(const sensor_msgs::Joy& msg) {

}

////////////////// Main //////////////////

int main(int argc, char **argv) {

	ros::init(argc, argv, "robik_teleop");

	ros::NodeHandle n;

	//advertise publishing topics
	pub_cmdvel = n.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 20);

	ros::Subscriber sub_wiimote_state = n.subscribe("/wiimote/state", 100, wiimoteStateCallback);
	ros::Subscriber sub_joy = n.subscribe("/joy", 100, joyCallback);

	ros::Rate loop_rate(100);

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
