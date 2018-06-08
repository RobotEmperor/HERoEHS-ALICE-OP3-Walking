/*
 * alice_op3_walking_module.cpp
 *
 *  Created on: Jun 6, 2018
 *      Author: robotemperor
 */

#include "alice_op3_walking_module/alice_op3_walking_module.h"

namespace  alice_walking
{

WalkingModule::WalkingModule()
: control_cycle_msec_(8),
  DEBUG(false)
{
	enable_ = false;
	module_name_ = "walking_module";
	control_mode_ = robotis_framework::PositionControl;

	init_pose_count_ = 0;
	walking_state_ = WalkingReady;
	previous_x_move_amplitude_ = 0.0;

	// result
	result_["l_hip_pitch"]      = new robotis_framework::DynamixelState();  // joint 11
	result_["l_hip_roll"]       = new robotis_framework::DynamixelState();  // joint 13
	result_["l_hip_yaw"]        = new robotis_framework::DynamixelState();  // joint 15
	result_["l_knee_pitch"]     = new robotis_framework::DynamixelState();  // joint 17
	result_["l_ankle_pitch"]    = new robotis_framework::DynamixelState();  // joint 19
	result_["l_ankle_roll"]     = new robotis_framework::DynamixelState();  // joint 21

	result_["r_hip_pitch"]      = new robotis_framework::DynamixelState();  // joint 12
	result_["r_hip_roll"]       = new robotis_framework::DynamixelState();  // joint 14
	result_["r_hip_yaw"]        = new robotis_framework::DynamixelState();  // joint 16
	result_["r_knee_pitch"]     = new robotis_framework::DynamixelState();  // joint 18
	result_["r_ankle_pitch"]    = new robotis_framework::DynamixelState();  // joint 20
	result_["r_ankle_roll"]     = new robotis_framework::DynamixelState();  // joint 22

	//result_["r_sho_pitch"] = new robotis_framework::DynamixelState();
	//result_["l_sho_pitch"] = new robotis_framework::DynamixelState();

	// joint table
	joint_table_["r_hip_yaw"] = 0;
	joint_table_["r_hip_roll"] = 1;
	joint_table_["r_hip_pitch"] = 2;
	joint_table_["r_knee_pitch"] = 3;
	joint_table_["r_ankle_pitch"] = 4;
	joint_table_["r_ankle_roll"] = 5;

	joint_table_["l_hip_yaw"] = 6;
	joint_table_["l_hip_roll"] = 7;
	joint_table_["l_hip_pitch"] = 8;
	joint_table_["l_knee_pitch"] = 9;
	joint_table_["l_ankle_pitch"] = 10;
	joint_table_["l_ankle_roll"] = 11;

	joint_id_to_name[0] = "r_hip_yaw";
	joint_id_to_name[1] = "r_hip_roll";
	joint_id_to_name[2] = "r_hip_pitch";
	joint_id_to_name[3] = "r_knee_pitch";
	joint_id_to_name[4] = "r_ankle_pitch";
	joint_id_to_name[5] = "r_ankle_roll";

	joint_id_to_name[6]  = "l_hip_yaw";
	joint_id_to_name[7]  = "l_hip_roll";
	joint_id_to_name[8]  = "l_hip_pitch";
	joint_id_to_name[9]  = "l_knee_pitch";
	joint_id_to_name[10] = "l_ankle_pitch";
	joint_id_to_name[11] = "l_ankle_roll";

	//joint_table_["r_sho_pitch"] = 12;
	//joint_table_["l_sho_pitch"] = 13;

	period_time_= 0;
	dsp_ratio_= 0;
	ssp_ratio_= 0;
	x_swap_period_time_= 0;
	x_move_period_time_= 0;
	y_swap_period_time_= 0;
	y_move_period_time_= 0;
	z_swap_period_time_= 0;
	z_move_period_time_= 0;
	a_move_period_time_= 0;
	ssp_time_= 0;
	l_ssp_start_time_= 0;
	l_ssp_end_time_= 0;
	r_ssp_start_time_= 0;
	r_ssp_end_time_= 0;
	phase1_time_= 0;
	phase2_time_= 0;
	phase3_time_= 0;
	x_offset_= 0;
	y_offset_= 0;
	z_offset_= 0;
	r_offset_= 0;
	p_offset_= 0;
	a_offset_= 0;
	x_swap_phase_shift_ = 0;
	x_swap_amplitude_= 0;
	x_swap_amplitude_shift_= 0;
	x_move_phase_shift_= 0;
	x_move_amplitude_= 0;
	x_move_amplitude_shift_= 0;
	y_swap_phase_shift_= 0;
	y_swap_amplitude_= 0;
	y_swap_amplitude_shift_= 0;
	y_move_phase_shift_= 0;
	y_move_amplitude_= 0;
	y_move_amplitude_shift_= 0;
	z_swap_phase_shift_= 0;
	z_swap_amplitude_= 0;
	z_swap_amplitude_shift_= 0;
	z_move_phase_shift_= 0;
	z_move_amplitude_= 0;
	z_move_amplitude_shift_= 0;
	a_move_phase_shift_= 0;
	a_move_amplitude_= 0;
	a_move_amplitude_shift_= 0;

	pelvis_offset_= 0;
	pelvis_swing_= 0;
	hit_pitch_offset_= 0;
	arm_swing_gain_= 0;

	ctrl_running_ = 0;
	real_running_ = 0;
	time_ = 0;

	phase_ = 0;
	body_swing_y = 0;
	body_swing_z = 0;

	target_position_ = Eigen::MatrixXd::Zero(1, 12);
	goal_position_ = Eigen::MatrixXd::Zero(1, 12);
	init_position_ = Eigen::MatrixXd::Zero(1, 12);
	joint_axis_direction_ = Eigen::MatrixXi::Zero(1, 12);

	// Alice kinematics

	l_kinematics_ = new heroehs_math::Kinematics;
	r_kinematics_ = new heroehs_math::Kinematics;


	for(int i = 0;i++; i<12)
	{
		balance_angle[i] = 0;
		angle[i] = 0;
	}

	currentGyroX = 0;
	currentGyroY = 0;
	currentGyroZ = 0;
	tf_current_gyro_x = 0;
	tf_current_gyro_y = 0;
	tf_current_gyro_z = 0;
	rl_gyro_err = 0.0;
	fb_gyro_err = 0,0;
	goal_position = 0.0;
}

WalkingModule::~WalkingModule()
{
	queue_thread_.join();
}

void WalkingModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
	queue_thread_ = boost::thread(boost::bind(&WalkingModule::queueThread, this));
	control_cycle_msec_ = control_cycle_msec;

	// m, s, rad
	// init pose
	walking_param_.init_x_offset = 0;
	walking_param_.init_y_offset = 0;
	walking_param_.init_z_offset = 0.04;
	walking_param_.init_roll_offset = 0.0;
	walking_param_.init_pitch_offset = 0.0 * DEGREE2RADIAN;
	walking_param_.init_yaw_offset = 0.0 * DEGREE2RADIAN;
	walking_param_.hip_pitch_offset = 0.0 * DEGREE2RADIAN;
	// time
	walking_param_.period_time = 600 * 0.001;
	walking_param_.dsp_ratio = 0.1;
	walking_param_.step_fb_ratio = 0.28;
	// walking
	walking_param_.x_move_amplitude = 0.0;
	walking_param_.y_move_amplitude = 0.0;
	walking_param_.z_move_amplitude = 0.0;    // foot height
	walking_param_.angle_move_amplitude = 0.0;
	// balance
	walking_param_.balance_enable = false;
	walking_param_.balance_hip_roll_gain = 0.5;
	walking_param_.balance_knee_gain = 0.3;
	walking_param_.balance_ankle_roll_gain = 1.0;
	walking_param_.balance_ankle_pitch_gain = 0.9;
	walking_param_.y_swap_amplitude = 0.020;
	walking_param_.z_swap_amplitude = 0.005;
	walking_param_.pelvis_offset = 3.0 * DEGREE2RADIAN;
	walking_param_.arm_swing_gain = 1.5;

	// member variable
	body_swing_y = 0;
	body_swing_z = 0;

	x_swap_phase_shift_ = M_PI;
	x_swap_amplitude_shift_ = 0;
	x_move_phase_shift_ = M_PI / 2;
	x_move_amplitude_shift_ = 0;
	y_swap_phase_shift_ = 0;
	y_swap_amplitude_shift_ = 0;
	y_move_phase_shift_ = M_PI / 2;
	z_swap_phase_shift_ = M_PI * 3 / 2;
	z_move_phase_shift_ = M_PI / 2;
	a_move_phase_shift_ = M_PI / 2;

	ctrl_running_ = false;
	real_running_ = false;
	time_ = 0;

	//                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL,
	//                     L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL,
	//                     R_ARM_SWING, L_ARM_SWING
	/*joint_axis_direction_ <<      -1,         -1,          -1,     -1,             1,            1,
			-1,         -1,           1,      1,            -1,            1,
			1,         -1;*/
	init_position_        <<     0.0,        0.0,         -22.6437,    45.2875,           22.6437,          0.0,
			0.0,        0.0,         22.6437,    -45.2875,           -22.6437,          0.0;

	init_position_ *= DEGREE2RADIAN;

	for (int id = 0; id <= 11; id++)
		target_position_.coeffRef(0, id) = init_position_(0, id);

	ros::NodeHandle ros_node;

	std::string default_param_path = ros::package::getPath("alice_op3_walking_module") + "/config/param.yaml";
	ros_node.param<std::string>("walking_param_path", param_path_, default_param_path);

	loadWalkingParam(param_path_);

	updateTimeParam();
	updateMovementParam();
}

void WalkingModule::queueThread()
{
	ros::NodeHandle ros_node;
	ros::CallbackQueue callback_queue;

	ros_node.setCallbackQueue(&callback_queue);

	/* publish topics */
	status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("robotis/status", 1);

	/* ROS Service Callback Functions */
	ros::ServiceServer get_walking_param_server = ros_node.advertiseService("/robotis/walking/get_params",
			&WalkingModule::getWalkigParameterCallback,
			this);

	/* sensor topic subscribe */
	ros::Subscriber walking_command_sub = ros_node.subscribe("/robotis/walking/command", 0,
			&WalkingModule::walkingCommandCallback, this);
	ros::Subscriber walking_param_sub = ros_node.subscribe("/robotis/walking/set_params", 0,
			&WalkingModule::walkingParameterCallback, this);

	get_imu_data_sub_ = ros_node.subscribe("/imu/data", 100, &WalkingModule::imuDataMsgCallback, this);
	get_ft_data_sub_ = ros_node.subscribe("/alice/force_torque_data", 100, &WalkingModule::ftDataMsgCallback, this);

	ros::WallDuration duration(control_cycle_msec_ / 1000.0);
	while(ros_node.ok())
		callback_queue.callAvailable(duration);
}

void WalkingModule::publishStatusMsg(unsigned int type, std::string msg)
{
	robotis_controller_msgs::StatusMsg status_msg;
	status_msg.header.stamp = ros::Time::now();
	status_msg.type = type;
	status_msg.module_name = "Walking";
	status_msg.status_msg = msg;

	status_msg_pub_.publish(status_msg);
}

void WalkingModule::walkingCommandCallback(const std_msgs::String::ConstPtr &msg)
{
	if(enable_ == false)
	{
		ROS_WARN("walking module is not ready.");
		return;
	}

	if (msg->data == "start")
		startWalking();
	else if (msg->data == "stop")
		stop();
	else if (msg->data == "balance on")
		walking_param_.balance_enable = true;
	else if (msg->data == "balance off")
		walking_param_.balance_enable = false;
	else if (msg->data == "save")
		saveWalkingParam(param_path_);
}

void WalkingModule::walkingParameterCallback(const op3_walking_module_msgs::WalkingParam::ConstPtr& msg)
{
	walking_param_ = *msg;
}

bool WalkingModule::getWalkigParameterCallback(op3_walking_module_msgs::GetWalkingParam::Request &req,
		op3_walking_module_msgs::GetWalkingParam::Response &res)
{
	res.parameters = walking_param_;

	return true;
}

double WalkingModule::wSin(double time, double period, double period_shift, double mag, double mag_shift)
{
	return mag * sin(2 * M_PI / period * time - period_shift) + mag_shift;
}


void WalkingModule::updateTimeParam()
{
	period_time_ = walking_param_.period_time;  // * 1000;   // s -> ms
	dsp_ratio_ = walking_param_.dsp_ratio;
	ssp_ratio_ = 1 - dsp_ratio_;

	x_swap_period_time_ = period_time_ / 2;
	x_move_period_time_ = period_time_ * ssp_ratio_;
	y_swap_period_time_ = period_time_;
	y_move_period_time_ = period_time_ * ssp_ratio_;
	z_swap_period_time_ = period_time_ / 2;
	z_move_period_time_ = period_time_ * ssp_ratio_ / 2;
	a_move_period_time_ = period_time_ * ssp_ratio_;

	ssp_time_ = period_time_ * ssp_ratio_;
	l_ssp_start_time_ = (1 - ssp_ratio_) * period_time_ / 4;
	l_ssp_end_time_ = (1 + ssp_ratio_) * period_time_ / 4;
	r_ssp_start_time_ = (3 - ssp_ratio_) * period_time_ / 4;
	r_ssp_end_time_ = (3 + ssp_ratio_) * period_time_ / 4;

	phase1_time_ = (l_ssp_start_time_ + l_ssp_end_time_) / 2;
	phase2_time_ = (l_ssp_end_time_ + r_ssp_start_time_) / 2;
	phase3_time_ = (r_ssp_start_time_ + r_ssp_end_time_) / 2;

	pelvis_offset_ = walking_param_.pelvis_offset;
	pelvis_swing_ = pelvis_offset_ * 0.35;
	arm_swing_gain_ = walking_param_.arm_swing_gain;
}

void WalkingModule::updateMovementParam()
{
	// Forward/Back
	x_move_amplitude_ = walking_param_.x_move_amplitude;
	x_swap_amplitude_ = walking_param_.x_move_amplitude * walking_param_.step_fb_ratio;

	if (previous_x_move_amplitude_ == 0)
	{
		x_move_amplitude_ *= 0.5;
		x_swap_amplitude_ *= 0.5;
	}

	// Right/Left
	y_move_amplitude_ = walking_param_.y_move_amplitude / 2;
	if (y_move_amplitude_ > 0)
		y_move_amplitude_shift_ = y_move_amplitude_;
	else
		y_move_amplitude_shift_ = -y_move_amplitude_;
	y_swap_amplitude_ = walking_param_.y_swap_amplitude + y_move_amplitude_shift_ * 0.04;

	z_move_amplitude_ = walking_param_.z_move_amplitude / 2;
	z_move_amplitude_shift_ = z_move_amplitude_ / 2;
	z_swap_amplitude_ = walking_param_.z_swap_amplitude;
	z_swap_amplitude_shift_ = z_swap_amplitude_;

	// Direction
	if (walking_param_.move_aim_on == false)
	{
		a_move_amplitude_ = walking_param_.angle_move_amplitude / 2;
		if (a_move_amplitude_ > 0)
			a_move_amplitude_shift_ = a_move_amplitude_;
		else
			a_move_amplitude_shift_ = -a_move_amplitude_;
	}
	else
	{
		a_move_amplitude_ = -walking_param_.angle_move_amplitude / 2;
		if (a_move_amplitude_ > 0)
			a_move_amplitude_shift_ = -a_move_amplitude_;
		else
			a_move_amplitude_shift_ = a_move_amplitude_;
	}
}

void WalkingModule::updatePoseParam()
{
	x_offset_ = walking_param_.init_x_offset;
	y_offset_ = walking_param_.init_y_offset;
	z_offset_ = walking_param_.init_z_offset;
	r_offset_ = walking_param_.init_roll_offset;
	p_offset_ = walking_param_.init_pitch_offset;
	a_offset_ = walking_param_.init_yaw_offset;
	hit_pitch_offset_ = walking_param_.hip_pitch_offset;
}

void WalkingModule::startWalking()
{
	ctrl_running_ = true;
	real_running_ = true;

	publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start walking");
}

void WalkingModule::stop()
{
	ctrl_running_ = false;
	publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Stop walking");
}

bool WalkingModule::isRunning()
{
	return real_running_ || (walking_state_ == WalkingInitPose);
}

// default [angle : radian, length : m]
void WalkingModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
		std::map<std::string, double> sensors)
{
	if (enable_ == false)
		return;

	const double time_unit = control_cycle_msec_ * 0.001;  // ms -> s

	if (walking_state_ == WalkingInitPose)
	{
		/*	int total_count = calc_joint_tra_.rows();
		for (int id = 0; id <= 11; id++)
			target_position_.coeffRef(0, id) = calc_joint_tra_(init_pose_count_, id);

		init_pose_count_ += 1;
		if (init_pose_count_ >= total_count)
		{
			walking_state_ = WalkingReady;
			if (DEBUG)
				std::cout << "End moving to Init : " << init_pose_count_ << std::endl;
		}*/
		for (int id = 0; id <= 11; id++)
			target_position_.coeffRef(0, id) = init_position_(0, id);

	}
	else if (walking_state_ == WalkingReady || walking_state_ == WalkingEnable)
	{
		// present angle
		/*for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
				state_iter != result_.end(); state_iter++)
		{
			std::string _joint_name = state_iter->first;
			int joint_index = joint_table_[_joint_name];

			robotis_framework::Dynamixel *dxl = NULL;
			std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(_joint_name);
			if (dxl_it != dxls.end())
				dxl = dxl_it->second;
			else
				continue;

			goal_position_.coeffRef(0, joint_index) = dxl->dxl_state_->goal_position_;
		}*/

		processPhase(time_unit);

		computeLegAngle();

		rl_gyro_err = 0.0 - tf_current_gyro_x;
		fb_gyro_err = 0.0 - tf_current_gyro_y;


		sensoryFeedback(rl_gyro_err, fb_gyro_err);

		double err_total = 0.0, err_max = 0.0;
		// set goal position
		for (int idx = 0; idx < 12; idx++)
		{

			//goal_position = init_position_.coeff(0, idx) + angle[idx] + balance_angle[idx];
			goal_position = angle[idx] + balance_angle[idx];

			target_position_.coeffRef(0, idx) = goal_position;

			/*double err = fabs(target_position_.coeff(0, idx) - goal_position_.coeff(0, idx)) * RADIAN2DEGREE;
			if (err > err_max)
				err_max = err;
			err_total += err;*/
		}
		// Check Enable
		/*if (walking_state_ == WalkingEnable && err_total > 5.0)
		{
			if (DEBUG)
				std::cout << "Check Err : " << err_max << std::endl;

			// make trajecotry for init pose
			int mov_time = err_max / 30;
			iniPoseTraGene(mov_time < 1 ? 1 : mov_time);

			// set target to goal
			target_position_ = goal_position_;

			walking_state_ = WalkingInitPose;

			ROS_WARN_STREAM_COND(DEBUG, "x_offset: " << walking_param_.init_x_offset);
			ROS_WARN_STREAM_COND(DEBUG, "y_offset: " << walking_param_.init_y_offset);
			ROS_WARN_STREAM_COND(DEBUG, "z_offset: " << walking_param_.init_z_offset);
			ROS_WARN_STREAM_COND(DEBUG, "roll_offset: " << walking_param_.init_roll_offset * RADIAN2DEGREE);
			ROS_WARN_STREAM_COND(DEBUG, "pitch_offset: " << walking_param_.init_pitch_offset * RADIAN2DEGREE);
			ROS_WARN_STREAM_COND(DEBUG, "yaw_offset: " << walking_param_.init_yaw_offset * RADIAN2DEGREE);
			ROS_WARN_STREAM_COND(DEBUG, "hip_pitch_offset: " << walking_param_.hip_pitch_offset * RADIAN2DEGREE);
			ROS_WARN_STREAM_COND(DEBUG, "period_time: " << walking_param_.period_time * 1000);
			ROS_WARN_STREAM_COND(DEBUG, "dsp_ratio: " << walking_param_.dsp_ratio);
			ROS_WARN_STREAM_COND(DEBUG, "step_forward_back_ratio: " << walking_param_.step_fb_ratio);
			ROS_WARN_STREAM_COND(DEBUG, "foot_height: " << walking_param_.z_move_amplitude);
			ROS_WARN_STREAM_COND(DEBUG, "swing_right_left: " << walking_param_.y_swap_amplitude);
			ROS_WARN_STREAM_COND(DEBUG, "swing_top_down: " << walking_param_.z_swap_amplitude);
			ROS_WARN_STREAM_COND(DEBUG, "pelvis_offset: " << walking_param_.pelvis_offset * RADIAN2DEGREE);
			ROS_WARN_STREAM_COND(DEBUG, "arm_swing_gain: " << walking_param_.arm_swing_gain);
			ROS_WARN_STREAM_COND(DEBUG, "balance_hip_roll_gain: " << walking_param_.balance_hip_roll_gain);
			ROS_WARN_STREAM_COND(DEBUG, "balance_knee_gain: " << walking_param_.balance_knee_gain);
			ROS_WARN_STREAM_COND(DEBUG, "balance_ankle_roll_gain: " << walking_param_.balance_ankle_roll_gain);
			ROS_WARN_STREAM_COND(DEBUG, "balance_ankle_pitch_gain: " << walking_param_.balance_ankle_pitch_gain);
			ROS_WARN_STREAM_COND(DEBUG, "balance : " << (walking_param_.balance_enable ? "TRUE" : "FALSE"));
		}
		else
		{
			walking_state_ = WalkingReady;
		}*/
	}

	for (int idx = 0; idx < 12; idx++)
	{
		result_[joint_id_to_name[idx]]->goal_position_ = target_position_.coeff(0, idx);
		printf("%d  ::    %f \n", idx, result_[joint_id_to_name[idx]]->goal_position_);


	}



	// time
	if (real_running_ == true)
	{
		time_ += time_unit;
		if (time_ >= period_time_)
		{
			time_ = 0;
			previous_x_move_amplitude_ = walking_param_.x_move_amplitude * 0.5;
		}
	}
}

void WalkingModule::processPhase(const double &time_unit)
{
	// Update walk parameters
	if (time_ == 0)
	{
		updateTimeParam();
		phase_ = PHASE0;
		if (ctrl_running_ == false)
		{
			if (x_move_amplitude_ == 0 && y_move_amplitude_ == 0 && a_move_amplitude_ == 0)
			{
				real_running_ = false;
			}
			else
			{
				// set walking param to init
				walking_param_.x_move_amplitude = 0;
				walking_param_.y_move_amplitude = 0;
				//walking_param_.z_move_amplitude = 0;
				walking_param_.angle_move_amplitude = 0;

				previous_x_move_amplitude_ = 0;
			}
		}
	}
	else if (time_ >= (phase1_time_ - time_unit / 2) && time_ < (phase1_time_ + time_unit / 2))  // the position of left foot is the highest.
	{
		updateMovementParam();
		phase_ = PHASE1;
	}
	else if (time_ >= (phase2_time_ - time_unit / 2) && time_ < (phase2_time_ + time_unit / 2))  // middle of double support state
	{
		updateTimeParam();

		time_ = phase2_time_;
		phase_ = PHASE2;
		if (ctrl_running_ == false)
		{
			if (x_move_amplitude_ == 0 && y_move_amplitude_ == 0 && a_move_amplitude_ == 0)
			{
				real_running_ = false;
			}
			else
			{
				// set walking param to init
				walking_param_.x_move_amplitude = previous_x_move_amplitude_;
				walking_param_.y_move_amplitude = 0;
				//walking_param_.z_move_amplitude = 0;
				walking_param_.angle_move_amplitude = 0;
			}
		}
	}
	else if (time_ >= (phase3_time_ - time_unit / 2) && time_ < (phase3_time_ + time_unit / 2))  // the position of right foot is the highest.
	{
		updateMovementParam();
		phase_ = PHASE3;
	}
}

bool WalkingModule::computeLegAngle()
{
	Pose3D swap, right_leg_move, left_leg_move;
	double pelvis_offset_r, pelvis_offset_l;
	double ep[12];

	pelvis_offset_r = 0.0;
	pelvis_offset_l = 0.0;
	for(int i = 0;i++; i<12)
	{
		ep[i] = 0.0;
	}
	swap.x = 0.0;
	swap.y = 0.0;
	swap.z = 0.0;
	swap.roll = 0.0;
	swap.pitch = 0.0;
	swap.yaw = 0.0;


	updatePoseParam();

	// Compute endpoints
	swap.x = wSin(time_, x_swap_period_time_, x_swap_phase_shift_, x_swap_amplitude_, x_swap_amplitude_shift_);
	swap.y = wSin(time_, y_swap_period_time_, y_swap_phase_shift_, y_swap_amplitude_, y_swap_amplitude_shift_);
	swap.z = wSin(time_, z_swap_period_time_, z_swap_phase_shift_, z_swap_amplitude_, z_swap_amplitude_shift_);
	swap.roll = 0.0;
	swap.pitch = 0.0;
	swap.yaw = 0.0;

	if (time_ <= l_ssp_start_time_)
	{
		left_leg_move.x = wSin(l_ssp_start_time_, x_move_period_time_,
				x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_, x_move_amplitude_,
				x_move_amplitude_shift_);
		left_leg_move.y = wSin(l_ssp_start_time_, y_move_period_time_,
				y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_, y_move_amplitude_,
				y_move_amplitude_shift_);
		left_leg_move.z = wSin(l_ssp_start_time_, z_move_period_time_,
				z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
				z_move_amplitude_shift_);
		left_leg_move.yaw = wSin(l_ssp_start_time_, a_move_period_time_,
				a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
				a_move_amplitude_, a_move_amplitude_shift_);
		right_leg_move.x = wSin(l_ssp_start_time_, x_move_period_time_,
				x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_,
				-x_move_amplitude_, -x_move_amplitude_shift_);
		right_leg_move.y = wSin(l_ssp_start_time_, y_move_period_time_,
				y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_,
				-y_move_amplitude_, -y_move_amplitude_shift_);
		right_leg_move.z = wSin(r_ssp_start_time_, z_move_period_time_,
				z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, z_move_amplitude_,
				z_move_amplitude_shift_);
		right_leg_move.yaw = wSin(l_ssp_start_time_, a_move_period_time_,
				a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
				-a_move_amplitude_, -a_move_amplitude_shift_);
		pelvis_offset_l = 0;
		pelvis_offset_r = 0;
	}
	else if (time_ <= l_ssp_end_time_)
	{
		left_leg_move.x = wSin(time_, x_move_period_time_,
				x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_, x_move_amplitude_,
				x_move_amplitude_shift_);
		left_leg_move.y = wSin(time_, y_move_period_time_,
				y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_, y_move_amplitude_,
				y_move_amplitude_shift_);
		left_leg_move.z = wSin(time_, z_move_period_time_,
				z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
				z_move_amplitude_shift_);
		left_leg_move.yaw = wSin(time_, a_move_period_time_,
				a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
				a_move_amplitude_, a_move_amplitude_shift_);
		right_leg_move.x = wSin(time_, x_move_period_time_,
				x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_,
				-x_move_amplitude_, -x_move_amplitude_shift_);
		right_leg_move.y = wSin(time_, y_move_period_time_,
				y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_,
				-y_move_amplitude_, -y_move_amplitude_shift_);
		right_leg_move.z = wSin(r_ssp_start_time_, z_move_period_time_,
				z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, z_move_amplitude_,
				z_move_amplitude_shift_);
		right_leg_move.yaw = wSin(time_, a_move_period_time_,
				a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
				-a_move_amplitude_, -a_move_amplitude_shift_);
		pelvis_offset_l = wSin(time_, z_move_period_time_,
				z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, pelvis_swing_ / 2,
				pelvis_swing_ / 2);
		pelvis_offset_r = wSin(time_, z_move_period_time_,
				z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_,
				-pelvis_offset_ / 2, -pelvis_offset_ / 2);
	}
	else if (time_ <= r_ssp_start_time_)
	{
		left_leg_move.x = wSin(l_ssp_end_time_, x_move_period_time_,
				x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_, x_move_amplitude_,
				x_move_amplitude_shift_);
		left_leg_move.y = wSin(l_ssp_end_time_, y_move_period_time_,
				y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_, y_move_amplitude_,
				y_move_amplitude_shift_);
		left_leg_move.z = wSin(l_ssp_end_time_, z_move_period_time_,
				z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
				z_move_amplitude_shift_);
		left_leg_move.yaw = wSin(l_ssp_end_time_, a_move_period_time_,
				a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
				a_move_amplitude_, a_move_amplitude_shift_);
		right_leg_move.x = wSin(l_ssp_end_time_, x_move_period_time_,
				x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_,
				-x_move_amplitude_, -x_move_amplitude_shift_);
		right_leg_move.y = wSin(l_ssp_end_time_, y_move_period_time_,
				y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_,
				-y_move_amplitude_, -y_move_amplitude_shift_);
		right_leg_move.z = wSin(r_ssp_start_time_, z_move_period_time_,
				z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, z_move_amplitude_,
				z_move_amplitude_shift_);
		right_leg_move.yaw = wSin(l_ssp_end_time_, a_move_period_time_,
				a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
				-a_move_amplitude_, -a_move_amplitude_shift_);
		pelvis_offset_l = 0;
		pelvis_offset_r = 0;
	}
	else if (time_ <= r_ssp_end_time_)
	{
		left_leg_move.x = wSin(time_, x_move_period_time_,
				x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
				x_move_amplitude_, x_move_amplitude_shift_);
		left_leg_move.y = wSin(time_, y_move_period_time_,
				y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
				y_move_amplitude_, y_move_amplitude_shift_);
		left_leg_move.z = wSin(l_ssp_end_time_, z_move_period_time_,
				z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
				z_move_amplitude_shift_);
		left_leg_move.yaw = wSin(time_, a_move_period_time_,
				a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
				a_move_amplitude_, a_move_amplitude_shift_);
		right_leg_move.x = wSin(time_, x_move_period_time_,
				x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
				-x_move_amplitude_, -x_move_amplitude_shift_);
		right_leg_move.y = wSin(time_, y_move_period_time_,
				y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
				-y_move_amplitude_, -y_move_amplitude_shift_);
		right_leg_move.z = wSin(time_, z_move_period_time_,
				z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, z_move_amplitude_,
				z_move_amplitude_shift_);
		right_leg_move.yaw = wSin(time_, a_move_period_time_,
				a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
				-a_move_amplitude_, -a_move_amplitude_shift_);
		pelvis_offset_l = wSin(time_, z_move_period_time_,
				z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, pelvis_offset_ / 2,
				pelvis_offset_ / 2);
		pelvis_offset_r = wSin(time_, z_move_period_time_,
				z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, -pelvis_swing_ / 2,
				-pelvis_swing_ / 2);
	}
	else
	{
		left_leg_move.x = wSin(r_ssp_end_time_, x_move_period_time_,
				x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
				x_move_amplitude_, x_move_amplitude_shift_);
		left_leg_move.y = wSin(r_ssp_end_time_, y_move_period_time_,
				y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
				y_move_amplitude_, y_move_amplitude_shift_);
		left_leg_move.z = wSin(l_ssp_end_time_, z_move_period_time_,
				z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
				z_move_amplitude_shift_);
		left_leg_move.yaw = wSin(r_ssp_end_time_, a_move_period_time_,
				a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
				a_move_amplitude_, a_move_amplitude_shift_);
		right_leg_move.x = wSin(r_ssp_end_time_, x_move_period_time_,
				x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
				-x_move_amplitude_, -x_move_amplitude_shift_);
		right_leg_move.y = wSin(r_ssp_end_time_, y_move_period_time_,
				y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
				-y_move_amplitude_, -y_move_amplitude_shift_);
		right_leg_move.z = wSin(r_ssp_end_time_, z_move_period_time_,
				z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, z_move_amplitude_,
				z_move_amplitude_shift_);
		right_leg_move.yaw = wSin(r_ssp_end_time_, a_move_period_time_,
				a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
				-a_move_amplitude_, -a_move_amplitude_shift_);
		pelvis_offset_l = 0;
		pelvis_offset_r = 0;
	}

	left_leg_move.roll = 0;
	left_leg_move.pitch = 0;
	right_leg_move.roll = 0;
	right_leg_move.pitch = 0;

	double leg_length = l_kinematics_->thigh_length_m + l_kinematics_->calf_length_m + l_kinematics_->ankle_length_m;


	// mm, rad
	ep[0] = swap.x + right_leg_move.x + x_offset_;
	ep[1] = swap.y + right_leg_move.y - y_offset_ / 2;
	ep[2] = swap.z + right_leg_move.z + z_offset_ - leg_length;

	ep[5] = swap.roll + right_leg_move.roll - r_offset_ / 2;
	ep[3] = swap.pitch + right_leg_move.pitch + p_offset_;
	ep[4] = swap.yaw + right_leg_move.yaw - a_offset_ / 2;

	ep[6] = swap.x + left_leg_move.x + x_offset_;
	ep[7] = swap.y + left_leg_move.y + y_offset_ / 2;
	ep[8] = swap.z + left_leg_move.z + z_offset_ - leg_length;

	ep[11] = swap.roll + left_leg_move.roll + r_offset_ / 2;
	ep[9] = swap.pitch + left_leg_move.pitch + p_offset_;
	ep[10] = swap.yaw + left_leg_move.yaw + a_offset_ / 2;

	// Compute body swing
	if (time_ <= l_ssp_end_time_)
	{
		body_swing_y = -ep[7];
		body_swing_z = ep[8];
	}
	else
	{
		body_swing_y = -ep[1];
		body_swing_z = ep[2];
	}
	body_swing_z -= leg_length;


	r_kinematics_->InverseKinematics(ep[0], ep[1], ep[2], ep[5], ep[4], ep[3]);
	l_kinematics_->InverseKinematics(ep[6], ep[7], ep[8], ep[11], ep[10], ep[9]);




	angle[0]  = r_kinematics_->joint_radian(3,0); // yaw
	angle[1]  = -r_kinematics_->joint_radian(2,0); // roll
	angle[2]  = r_kinematics_->joint_radian(1,0);  // pitch

	angle[3]  = r_kinematics_->joint_radian(4,0);
	angle[4]  = -r_kinematics_->joint_radian(5,0);
	angle[5]  = r_kinematics_->joint_radian(6,0);

	angle[6]  = l_kinematics_->joint_radian(3,0);//yaw
	angle[7]  = -l_kinematics_->joint_radian(2,0);//roll
	angle[8]  = -l_kinematics_->joint_radian(1,0); //pitch

	angle[9]  = -l_kinematics_->joint_radian(4,0);
	angle[10] = l_kinematics_->joint_radian(5,0);
	angle[11] = l_kinematics_->joint_radian(6,0);


	// Compute dxls angle
	for (int i = 0; i < 12; i++)
	{
		// offset : rad
		double offset = 0;

		if (i == 1)  // R_HIP_ROLL
			offset += -1 * pelvis_offset_r;
		else if (i == 7)  // L_HIP_ROLL
			offset += -1 * pelvis_offset_l;
		else if (i == 0)
			offset -= 1 * hit_pitch_offset_;
		else if (i == 6)  // R_HIP_PITCH or L_HIP_PITCH
			offset -= -1 * hit_pitch_offset_;

		angle[i] += offset;
	}

	return true;
}

void WalkingModule::computeArmAngle(double *arm_angle)
{
	/*// Compute arm swing
	if (x_move_amplitude_ == 0)
	{
		arm_angle[0] = 0;  // Right
		arm_angle[1] = 0;  // Left
	}
	else
	{
		arm_angle[0] = wSin(time_, period_time_, M_PI * 1.5, -x_move_amplitude_ * arm_swing_gain_ * 1000,
				0) * op3_kd_->getJointDirection("r_sho_pitch") * DEGREE2RADIAN;
		arm_angle[1] = wSin(time_, period_time_, M_PI * 1.5, x_move_amplitude_ * arm_swing_gain_ * 1000,
				0) * op3_kd_->getJointDirection("l_sho_pitch") * DEGREE2RADIAN;
	}*/
}

void WalkingModule::sensoryFeedback(double rlGyroErr,double fbGyroErr)
{
	// adjust balance offset
	if (walking_param_.balance_enable == false)
		return;

	// max min check !
	if(rlGyroErr > 0.26)
	{
		rlGyroErr = 0.26;
	}
	if(rlGyroErr < -0.26)
	{
		rlGyroErr = -0.26;
	}
	if(fbGyroErr > 0.26)
	{
		fbGyroErr = 0.26;
	}
	if(fbGyroErr < -0.26)
	{
		fbGyroErr = -0.26;
	}

	static double internal_gain = 0.05;

	/*balance_angle[joint_table_["r_hip_roll"]] =  op3_kd_->getJointDirection("r_hip_roll") * internal_gain
	 * rlGyroErr * walking_param_.balance_hip_roll_gain;  // R_HIP_ROLL
	balance_angle[joint_table_["l_hip_roll"]] =  op3_kd_->getJointDirection("l_hip_roll") * internal_gain
	 * rlGyroErr * walking_param_.balance_hip_roll_gain;  // L_HIP_ROLL

	balance_angle[joint_table_["r_knee"]] = - op3_kd_->getJointDirection("r_knee") * internal_gain
	 * fbGyroErr * walking_param_.balance_knee_gain;  // R_KNEE
	balance_angle[joint_table_["l_knee"]] = - op3_kd_->getJointDirection("l_knee") * internal_gain
	 * fbGyroErr * walking_param_.balance_knee_gain;  // L_KNEE

	balance_angle[joint_table_["r_ank_pitch"]] = - op3_kd_->getJointDirection("r_ank_pitch")
	 * internal_gain * fbGyroErr * walking_param_.balance_ankle_pitch_gain;  // R_ANKLE_PITCH
	balance_angle[joint_table_["l_ank_pitch"]] = - op3_kd_->getJointDirection("l_ank_pitch")
	 * internal_gain * fbGyroErr * walking_param_.balance_ankle_pitch_gain;  // L_ANKLE_PITCH

	balance_angle[joint_table_["r_ank_roll"]] = - op3_kd_->getJointDirection("r_ank_roll") * internal_gain
	 * rlGyroErr * walking_param_.balance_ankle_roll_gain;  // R_ANKLE_ROLL
	balance_angle[joint_table_["l_ank_roll"]] = - op3_kd_->getJointDirection("l_ank_roll") * internal_gain
	 * rlGyroErr * walking_param_.balance_ankle_roll_gain;  // L_ANKLE_ROLL*/


	balance_angle[joint_table_["r_hip_roll"]] =  -1* internal_gain
			* rlGyroErr * walking_param_.balance_hip_roll_gain;  // R_HIP_ROLL
	balance_angle[joint_table_["l_hip_roll"]] =  -1 * internal_gain
			* rlGyroErr * walking_param_.balance_hip_roll_gain;  // L_HIP_ROLL

	balance_angle[joint_table_["r_knee_pitch"]] = - 1 * internal_gain
			* fbGyroErr * walking_param_.balance_knee_gain;  // R_KNEE
	balance_angle[joint_table_["l_knee_pitch"]] = -1 * -1 * internal_gain
			* fbGyroErr * walking_param_.balance_knee_gain;  // L_KNEE

	balance_angle[joint_table_["r_ankle_pitch"]] = -1 *-1
			* internal_gain * fbGyroErr * walking_param_.balance_ankle_pitch_gain;  // R_ANKLE_PITCH
	balance_angle[joint_table_["l_ankle_pitch"]] = - 1
			* internal_gain * fbGyroErr * walking_param_.balance_ankle_pitch_gain;  // L_ANKLE_PITCH

	balance_angle[joint_table_["r_ankle_roll"]] = - 1 * internal_gain
			* rlGyroErr * walking_param_.balance_ankle_roll_gain;  // R_ANKLE_ROLL
	balance_angle[joint_table_["l_ankle_roll"]] = - 1 * internal_gain
			* rlGyroErr * walking_param_.balance_ankle_roll_gain;  // L_ANKLE_ROLL

}

void WalkingModule::loadWalkingParam(const std::string &path)
{
	YAML::Node doc;
	try
	{
		// load yaml
		doc = YAML::LoadFile(path.c_str());
	} catch (const std::exception& e)
	{
		ROS_ERROR("Fail to load yaml file.");
		return;
	}

	// parse movement time
	walking_param_.init_x_offset = doc["x_offset"].as<double>();
	walking_param_.init_y_offset = doc["y_offset"].as<double>();
	walking_param_.init_z_offset = doc["z_offset"].as<double>();
	walking_param_.init_roll_offset = doc["roll_offset"].as<double>() * DEGREE2RADIAN;
	walking_param_.init_pitch_offset = doc["pitch_offset"].as<double>() * DEGREE2RADIAN;
	walking_param_.init_yaw_offset = doc["yaw_offset"].as<double>() * DEGREE2RADIAN;
	walking_param_.hip_pitch_offset = doc["hip_pitch_offset"].as<double>() * DEGREE2RADIAN;
	// time
	walking_param_.period_time = doc["period_time"].as<double>() * 0.001;    // ms -> s
	walking_param_.dsp_ratio = doc["dsp_ratio"].as<double>();
	walking_param_.step_fb_ratio = doc["step_forward_back_ratio"].as<double>();
	// walking
	// walking_param_.x_move_amplitude
	// walking_param_.y_move_amplitude
	walking_param_.z_move_amplitude = doc["foot_height"].as<double>();
	// walking_param_.angle_move_amplitude
	// walking_param_.move_aim_on

	// balance
	// walking_param_.balance_enable
	walking_param_.balance_hip_roll_gain = doc["balance_hip_roll_gain"].as<double>();
	walking_param_.balance_knee_gain = doc["balance_knee_gain"].as<double>();
	walking_param_.balance_ankle_roll_gain = doc["balance_ankle_roll_gain"].as<double>();
	walking_param_.balance_ankle_pitch_gain = doc["balance_ankle_pitch_gain"].as<double>();
	walking_param_.y_swap_amplitude = doc["swing_right_left"].as<double>();
	walking_param_.z_swap_amplitude = doc["swing_top_down"].as<double>();
	walking_param_.pelvis_offset = doc["pelvis_offset"].as<double>() * DEGREE2RADIAN;
	walking_param_.arm_swing_gain = doc["arm_swing_gain"].as<double>();

	// gain
	walking_param_.p_gain = doc["p_gain"].as<int>();
	walking_param_.i_gain = doc["i_gain"].as<int>();
	walking_param_.d_gain = doc["d_gain"].as<int>();
}

void WalkingModule::saveWalkingParam(std::string &path)
{
	YAML::Emitter out_emitter;

	out_emitter << YAML::BeginMap;
	out_emitter << YAML::Key << "x_offset" << YAML::Value << walking_param_.init_x_offset;
	out_emitter << YAML::Key << "y_offset" << YAML::Value << walking_param_.init_y_offset;
	out_emitter << YAML::Key << "z_offset" << YAML::Value << walking_param_.init_z_offset;
	out_emitter << YAML::Key << "roll_offset" << YAML::Value << walking_param_.init_roll_offset * RADIAN2DEGREE;
	out_emitter << YAML::Key << "pitch_offset" << YAML::Value << walking_param_.init_pitch_offset * RADIAN2DEGREE;
	out_emitter << YAML::Key << "yaw_offset" << YAML::Value << walking_param_.init_yaw_offset * RADIAN2DEGREE;
	out_emitter << YAML::Key << "hip_pitch_offset" << YAML::Value << walking_param_.hip_pitch_offset * RADIAN2DEGREE;
	out_emitter << YAML::Key << "period_time" << YAML::Value << walking_param_.period_time * 1000;
	out_emitter << YAML::Key << "dsp_ratio" << YAML::Value << walking_param_.dsp_ratio;
	out_emitter << YAML::Key << "step_forward_back_ratio" << YAML::Value << walking_param_.step_fb_ratio;
	out_emitter << YAML::Key << "foot_height" << YAML::Value << walking_param_.z_move_amplitude;
	out_emitter << YAML::Key << "swing_right_left" << YAML::Value << walking_param_.y_swap_amplitude;
	out_emitter << YAML::Key << "swing_top_down" << YAML::Value << walking_param_.z_swap_amplitude;
	out_emitter << YAML::Key << "pelvis_offset" << YAML::Value << walking_param_.pelvis_offset * RADIAN2DEGREE;
	out_emitter << YAML::Key << "arm_swing_gain" << YAML::Value << walking_param_.arm_swing_gain;
	out_emitter << YAML::Key << "balance_hip_roll_gain" << YAML::Value << walking_param_.balance_hip_roll_gain;
	out_emitter << YAML::Key << "balance_knee_gain" << YAML::Value << walking_param_.balance_knee_gain;
	out_emitter << YAML::Key << "balance_ankle_roll_gain" << YAML::Value << walking_param_.balance_ankle_roll_gain;
	out_emitter << YAML::Key << "balance_ankle_pitch_gain" << YAML::Value << walking_param_.balance_ankle_pitch_gain;

	out_emitter << YAML::Key << "p_gain" << YAML::Value << walking_param_.p_gain;
	out_emitter << YAML::Key << "i_gain" << YAML::Value << walking_param_.i_gain;
	out_emitter << YAML::Key << "d_gain" << YAML::Value << walking_param_.d_gain;
	out_emitter << YAML::EndMap;

	// output to file
	std::ofstream fout(path.c_str());
	fout << out_emitter.c_str();
}

void WalkingModule::onModuleEnable()
{
	walking_state_ = WalkingEnable;
	ROS_INFO("Walking Enable");
}

void WalkingModule::onModuleDisable()
{
	ROS_INFO("Walking Disable");
	walking_state_ = WalkingDisable;
}

void WalkingModule::iniPoseTraGene(double mov_time)
{
	double smp_time = control_cycle_msec_ * 0.001;
	int all_time_steps = int(mov_time / smp_time) + 1;
	calc_joint_tra_.resize(all_time_steps, 12 + 1);

	for (int id = 0; id <= 11; id++)
	{
		double ini_value = goal_position_.coeff(0, id);
		double tar_value = target_position_.coeff(0, id);

		Eigen::MatrixXd tra;

		tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0, smp_time, mov_time);

		calc_joint_tra_.block(0, id, all_time_steps, 1) = tra;
	}

	if(DEBUG)
		std::cout << "Generate Trajecotry : " << mov_time << "s [" << all_time_steps << "]" << std::endl;

	init_pose_count_ = 0;
}
void WalkingModule::imuDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg) // gyro data get
{
	currentGyroX = (double) msg->angular_velocity.x;
	currentGyroY = (double) msg->angular_velocity.y;
	currentGyroZ = (double) msg->angular_velocity.z;
	gyroRotationTransformation(currentGyroZ, currentGyroY, currentGyroX);
	//balance_ctrl_.setCurrentGyroSensorOutput(tf_current_gyro_x, tf_current_gyro_y);
}

void WalkingModule::ftDataMsgCallback(const alice_msgs::ForceTorque::ConstPtr& msg)// force torque sensor data get
{
	/*currentFX_l = (double) msg->force_x_raw_l;
	currentFY_l = (double) msg->force_y_raw_l;
	currentFZ_l = (double) msg->force_z_raw_l;


	currentTX_l = (double) msg->torque_x_raw_l;
	currentTY_l = (double) msg->torque_y_raw_l;
	currentTZ_l = (double) msg->torque_z_raw_l;

	currentFX_r = (double) msg->force_x_raw_r;
	currentFY_r = (double) msg->force_y_raw_r;
	currentFZ_r = (double) msg->force_z_raw_r;


	currentTX_r = (double) msg->torque_x_raw_r;
	currentTY_r = (double) msg->torque_y_raw_r;
	currentTZ_r = (double) msg->torque_z_raw_r;

	if(currentFX_l && currentFY_l && currentFZ_l && currentTX_l && currentTY_l && currentTZ_l)
	{
		copFz_p_gain = 0;
		copFz_d_gain = 0;
	} // cop control disable

	cop_cal->ftSensorDataLeftGet(currentFX_l, currentFY_l, currentFZ_l, currentTX_l, currentTY_l, currentTZ_l);
	cop_cal->ftSensorDataRightGet(currentFX_r, currentFY_r, currentFZ_r, currentTX_r, currentTY_r, currentTZ_r);

	cop_cal->jointStateGetForTransForm(l_kinematics_->joint_radian, r_kinematics_->joint_radian);
	cop_cal->copCalculationResult();
	cop_compensation->centerOfPressureReferencePoint(temp_turn_type,   cop_cal->cf_px_l, cop_cal->cf_py_l, cop_cal->cf_pz_l,
			cop_cal->cf_px_r, cop_cal->cf_py_r, cop_cal->cf_pz_r, temp_change_value_center);*/
}
void WalkingModule::gyroRotationTransformation(double gyro_z, double gyro_y, double gyro_x)
{
	Eigen::MatrixXd tf_gyro_value;
	tf_gyro_value.resize(3,1);
	tf_gyro_value.fill(0);
	tf_gyro_value(0,0) =  gyro_x;
	tf_gyro_value(1,0) =  gyro_y;
	tf_gyro_value(2,0) =  gyro_z;

	tf_gyro_value = (robotis_framework::getRotationZ(-M_PI/2)*robotis_framework::getRotationY(-M_PI))*tf_gyro_value;
	tf_current_gyro_x = tf_gyro_value(0,0);
	tf_current_gyro_y = tf_gyro_value(1,0);
	tf_current_gyro_z = tf_gyro_value(2,0);
}

}


