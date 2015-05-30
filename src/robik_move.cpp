#include "robik_move.h"
#include "robot_config.h"
#include "nav_msgs/Odometry.h"


ros::Publisher pub_velocity_control;


double odom_x = 0;
double odom_y = 0;
double odom_theta = 0;

void processOdomTicks(double *vx, double *vy, double *vth,
    int odomTicksLeft, int odomTicksRight,
    unsigned int odom_millisSinceLastUpdate) {
//based on
//http://simreal.com/content/Odometry
//http://naba.blogspot.cz/2008/08/two-wheel-drive-odometry.html
//http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom#Using_tf_to_Publish_an_Odometry_transform
  double dr = odomTicksRight * (TICK_LENGTH_MM / 1000); //left wheel traveled distance in meters
  double dl = odomTicksLeft * (TICK_LENGTH_MM / 1000); //right wheel

  double dDist = (dr + dl) / 2; //distance of base link traveled in meters
  double dTheta = (dr - dl) / (AXIAL_LENGTH_MM /1000); //change in orientation [radians???]

  odom_theta = odom_theta + dTheta;
  double vel_t_x = (dDist * cos(odom_theta)); //distance traveled since last update on x axis [meters]
  double vel_t_y = (dDist * sin(odom_theta)); //y axis
  odom_x = odom_x + vel_t_x; //new absolute position related to starting point of robot
  odom_y = odom_y + vel_t_y;

  *vx = vel_t_x * odom_millisSinceLastUpdate / 1000; //velocity on x axis [meters/second)
  *vy = vel_t_y * odom_millisSinceLastUpdate / 1000; //y axis
  *vth = dTheta; //speed of rotation

}

void cmdvelCallback(const geometry_msgs::Twist& msg) {

  pub_velocity_control.publish(msg);
}
