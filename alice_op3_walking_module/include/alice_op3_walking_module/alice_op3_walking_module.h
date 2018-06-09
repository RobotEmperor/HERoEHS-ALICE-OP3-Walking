/*
 * alice_op3_walking_module.h
 *
 *  Created on: Jun 6, 2018
 *      Author: robotemperor
 */

#ifndef ALICE_HEROEHS_ALICE_OP3_WALKING_ALICE_OP3_WALKING_MODULE_INCLUDE_ALICE_OP3_WALKING_MODULE_ALICE_OP3_WALKING_MODULE_H_
#define ALICE_HEROEHS_ALICE_OP3_WALKING_ALICE_OP3_WALKING_MODULE_INCLUDE_ALICE_OP3_WALKING_MODULE_ALICE_OP3_WALKING_MODULE_H_
#include <stdio.h>
#include <math.h>
#include <fstream>
#include <boost/thread.hpp>

//#include <eigen3/Eigen/Eigen> // 이거 왜 넣놓은거임

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>

#include "robotis_controller_msgs/StatusMsg.h"
#include "op3_walking_module_msgs/WalkingParam.h"
#include "op3_walking_module_msgs/GetWalkingParam.h"
#include "op3_walking_module_msgs/SetWalkingParam.h"

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"
#include "robotis_math/robotis_trajectory_calculator.h"
#include "heroehs_math/kinematics.h"
#include "alice_balance_control/zmp_calculation_function.h"

//personal message
#include "alice_msgs/ForceTorque.h"

namespace alice_walking
{

typedef struct
{
	double x, y, z;
} Position3D;

typedef struct
{
	double x, y, z, roll, pitch, yaw;
} Pose3D;

class WalkingModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<WalkingModule>
{
public:
	enum
	{
		PHASE0 = 0,
		PHASE1 = 1,
		PHASE2 = 2,
		PHASE3 = 3
	};

	WalkingModule();
	virtual ~WalkingModule();

	void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
	void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
	void stop();
	bool isRunning();
	void onModuleEnable();
	void onModuleDisable();

	int getCurrentPhase()
	{
		return phase_;
	}
	double getBodySwingY()
	{
		return body_swing_y;
	}
	double getBodySwingZ()
	{
		return body_swing_z;
	}

private:
	enum
	{
		WalkingDisable = 0,
		WalkingEnable = 1,
		WalkingInitPose = 2,
		WalkingReady = 3
	};

	const bool DEBUG;

	void queueThread();

	/* ROS Topic Callback Functions */

	// publisher
	ros::Publisher l_leg_point_xyz_pub;
	ros::Publisher r_leg_point_xyz_pub;
	ros::Publisher zmp_fz_pub;

	geometry_msgs::Vector3 l_leg_point_xyz_msg_;
	geometry_msgs::Vector3 r_leg_point_xyz_msg_;
	std_msgs::Float64MultiArray zmp_fz_msg_;


	//subscriber
	ros::Subscriber get_imu_data_sub_;
	ros::Subscriber get_ft_data_sub_;

	void walkingCommandCallback(const std_msgs::String::ConstPtr &msg);
	void walkingParameterCallback(const op3_walking_module_msgs::WalkingParam::ConstPtr &msg);
	bool getWalkigParameterCallback(op3_walking_module_msgs::GetWalkingParam::Request &req,
			op3_walking_module_msgs::GetWalkingParam::Response &res);
	void imuDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg);
	void ftDataMsgCallback(const alice_msgs::ForceTorque::ConstPtr& msg);

	void gyroRotationTransformation(double gyro_z, double gyro_y, double gyro_x);
	double currentGyroX,currentGyroY,currentGyroZ;
	double tf_current_gyro_x, tf_current_gyro_y, tf_current_gyro_z;
	double currentFX_l,currentFY_l,currentFZ_l,currentTX_l,currentTY_l,currentTZ_l;
	double currentFX_r,currentFY_r,currentFZ_r,currentTX_r,currentTY_r,currentTZ_r;


	/* ROS Service Callback Functions */
	void processPhase(const double &time_unit);
	void computeLegAngle();
	void sensoryFeedback(double rlGyroErr,double fbGyroErr);

	void publishStatusMsg(unsigned int type, std::string msg);
	double wSin(double time, double period, double period_shift, double mag, double mag_shift);

	void updateTimeParam();
	void updateMovementParam();
	void updatePoseParam();
	void startWalking();
	void loadWalkingParam(const std::string &path);
	void saveWalkingParam(std::string &path);
	void iniPoseTraGene(double mov_time);


	int control_cycle_msec_;
	std::string param_path_;
	boost::thread queue_thread_;

	/* ROS Topic Publish Functions */
	ros::Publisher robot_pose_pub_;
	ros::Publisher status_msg_pub_;

	Eigen::MatrixXd calc_joint_tra_;

	Eigen::MatrixXd target_position_;
	Eigen::MatrixXd init_position_;

	std::map<std::string, int> joint_table_;
	std::map<int, std::string> joint_id_to_name;
	int walking_state_;
	int init_pose_count_;
	op3_walking_module_msgs::WalkingParam walking_param_;
	double previous_x_move_amplitude_;

	// variable for walking
	double period_time_;
	double dsp_ratio_;
	double ssp_ratio_;
	double x_swap_period_time_;
	double x_move_period_time_;
	double y_swap_period_time_;
	double y_move_period_time_;
	double z_swap_period_time_;
	double z_move_period_time_;
	double a_move_period_time_;
	double ssp_time_;
	double l_ssp_start_time_;
	double l_ssp_end_time_;
	double r_ssp_start_time_;
	double r_ssp_end_time_;
	double phase1_time_;
	double phase2_time_;
	double phase3_time_;

	double x_offset_;
	double y_offset_;
	double z_offset_;
	double r_offset_;
	double p_offset_;
	double a_offset_;

	double x_swap_phase_shift_;
	double x_swap_amplitude_;
	double x_swap_amplitude_shift_;
	double x_move_phase_shift_;
	double x_move_amplitude_;
	double x_move_amplitude_shift_;
	double y_swap_phase_shift_;
	double y_swap_amplitude_;
	double y_swap_amplitude_shift_;
	double y_move_phase_shift_;
	double y_move_amplitude_;
	double y_move_amplitude_shift_;
	double z_swap_phase_shift_;
	double z_swap_amplitude_;
	double z_swap_amplitude_shift_;
	double z_move_phase_shift_;
	double z_move_amplitude_;
	double z_move_amplitude_shift_;
	double a_move_phase_shift_;
	double a_move_amplitude_;
	double a_move_amplitude_shift_;

	double pelvis_offset_;
	double pelvis_swing_;
	double hit_pitch_offset_;
	double arm_swing_gain_;

	bool ctrl_running_;
	bool real_running_;
	double time_;

	int phase_;
	double body_swing_y;
	double body_swing_z;

	double angle_[12];
	double balance_angle_[12];
	double rl_gyro_err;
	double fb_gyro_err;
	double internal_gain;

	double goal_position;


	//alice kinematics

	heroehs_math::Kinematics *l_kinematics_;
	heroehs_math::Kinematics *r_kinematics_;

	// zmp cal
	alice::ZmpCalculationFunc *zmp_cal;
};
}




#endif /* ALICE_HEROEHS_ALICE_OP3_WALKING_ALICE_OP3_WALKING_MODULE_INCLUDE_ALICE_OP3_WALKING_MODULE_ALICE_OP3_WALKING_MODULE_H_ */
