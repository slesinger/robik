#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/robot_hw.h>

#include "ros/time.h"
#include "ros/duration.h"

#include "robik/GenericStatus.h"

#define ARM_DEG_THLD 20
#define ARM_OUTL_CNT_MAX 5

class RobikControllers : public hardware_interface::RobotHW {
public:
	static RobikControllers& get_instance() {
		static RobikControllers instance;
		return instance;
	}
	void init();
	void read_from_hw(const robik::GenericStatus& msg);
	void write_to_hw();
	ros::Time get_time();
	ros::Duration get_period();

private:
	RobikControllers() {}; 
	hardware_interface::JointStateInterface jnt_state_interface;
	hardware_interface::PositionJointInterface jnt_pos_interface;
	hardware_interface::VelocityJointInterface jnt_vel_interface;
	ros::Time last_time;
	ros::Duration period;
	double cmd[7];
	double pos[7];
	double vel[7];
	double eff[7];

	// IMU
	hardware_interface::ImuSensorInterface imu_interface;
	hardware_interface::ImuSensorHandle::Data imu_data;
	double imu_orientation[4];
	double imu_angular_velocity[3];
	double imu_linear_acceleration[3];
};




