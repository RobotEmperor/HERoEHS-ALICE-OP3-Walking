/*
 * alice_op3_walking_module.cpp
 *
 *  Created on: Jun 6, 2018
 *      Author: robotemperor
 */

#include "alice_op3_walking_module/alice_op3_walking_module.h"

using namespace  alice_walking;

void WalkingModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
	target_position_.resize(1,12);
	target_position_.fill(0);
	init_position_.resize(1,12);
	init_position_.fill(0);

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

	init_position_        <<     0.0,        0.0,         -22.6437,    45.2875,           22.6437,          0.0,
			0.0,        0.0,         22.6437,    -45.2875,           -22.6437,          0.0;

	init_position_ *= DEGREE2RADIAN;

	for (int id = 0; id <= 11; id++)
		target_position_(0, id) = init_position_(0, id);

	ros::NodeHandle ros_node;

	std::string default_param_path = ros::package::getPath("alice_op3_walking_module") + "/config/param.yaml";
	ros_node.param<std::string>("walking_param_path", param_path_, default_param_path);

	loadWalkingParam(param_path_);

	updateTimeParam();
	updateMovementParam();
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
		for (int id = 0; id <= 11; id++)
			target_position_(0, id) = init_position_(0, id);

	}
	else if (walking_state_ == WalkingReady || walking_state_ == WalkingEnable)
	{
		processPhase(time_unit);

		computeLegAngle();

		rl_gyro_err = 0.0 - tf_current_gyro_x;
		fb_gyro_err = 0.0 - tf_current_gyro_y;

		sensoryFeedback(rl_gyro_err, fb_gyro_err);

		// set goal position
		for (int idx = 0; idx < 12; idx++)
		{
			goal_position = angle_[idx] + balance_angle_[idx];
			target_position_(0, idx) = goal_position;
		}
	}


	for (int idx = 0; idx < 12; idx++)
	{
		//printf("%d  ::    %f \n", idx, target_position_(0, idx));
		result_[joint_id_to_name[idx]]->goal_position_ = target_position_(0, idx);
		//printf("%d  ::    %f \n", idx, result_[joint_id_to_name[idx]]->goal_position_);
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
	// xyz point pub
	// l_ endpoint pub
	l_leg_point_xyz_pub.publish(l_leg_point_xyz_msg_);

	// r_ endpoint xyz
	r_leg_point_xyz_pub.publish(r_leg_point_xyz_msg_);

	//zmp pub
	zmp_fz_msg_.data.push_back(zmp_cal->zmp_fz_point_x); // current cop value
	zmp_fz_msg_.data.push_back(zmp_cal->zmp_fz_point_y);
	zmp_fz_pub.publish(zmp_fz_msg_);
	zmp_fz_msg_.data.clear();
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

void WalkingModule::computeLegAngle()
{
	Pose3D swap, right_leg_move, left_leg_move;
	double pelvis_offset_r, pelvis_offset_l;
	double ep[12];

	pelvis_offset_r = 0.0;
	pelvis_offset_l = 0.0;
	for(int i = 0;i<12; i++)
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

	// l_ endpoint xyz
	l_leg_point_xyz_msg_.x=  ep[6];
	l_leg_point_xyz_msg_.y=  ep[7]-0.09;
	l_leg_point_xyz_msg_.z=  ep[8];

	// r_ endpoint xyz
	r_leg_point_xyz_msg_.x=  ep[0];
	r_leg_point_xyz_msg_.y=  ep[1]+0.09;
	r_leg_point_xyz_msg_.z=  ep[2];


	angle_[0]  = r_kinematics_->joint_radian(3,0); // yaw
	angle_[1]  = -r_kinematics_->joint_radian(2,0); // roll
	angle_[2]  = r_kinematics_->joint_radian(1,0);  // pitch

	angle_[3]  = r_kinematics_->joint_radian(4,0);
	angle_[4]  = -r_kinematics_->joint_radian(5,0);
	angle_[5]  = r_kinematics_->joint_radian(6,0);

	angle_[6]  = l_kinematics_->joint_radian(3,0);//yaw
	angle_[7]  = -l_kinematics_->joint_radian(2,0);//roll
	angle_[8]  = -l_kinematics_->joint_radian(1,0); //pitch

	angle_[9]  = -l_kinematics_->joint_radian(4,0);
	angle_[10] = l_kinematics_->joint_radian(5,0);
	angle_[11] = l_kinematics_->joint_radian(6,0);


	// Compute dxls angle
	for (int i = 0; i < 12; i++)
	{
		// offset : rad
		double offset = 0;

		if (i == 1)  // R_HIP_ROLL
			offset += -1 * pelvis_offset_r;
		else if (i == 7)  // L_HIP_ROLL
			offset += -1 * pelvis_offset_l;
		else if (i == 2)
			offset -= 1 * hit_pitch_offset_;
		else if (i == 8)  // R_HIP_PITCH or L_HIP_PITCH
			offset -= -1 * hit_pitch_offset_;

		angle_[i] += offset;



	}

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


	balance_angle_[joint_table_["r_hip_roll"]] =  -1* internal_gain
			* rlGyroErr * walking_param_.balance_hip_roll_gain;  // R_HIP_ROLL
	balance_angle_[joint_table_["l_hip_roll"]] =  -1 * internal_gain
			* rlGyroErr * walking_param_.balance_hip_roll_gain;  // L_HIP_ROLL

	balance_angle_[joint_table_["r_knee_pitch"]] = - 1 * internal_gain
			* fbGyroErr * walking_param_.balance_knee_gain;  // R_KNEE
	balance_angle_[joint_table_["l_knee_pitch"]] = -1 * -1 * internal_gain
			* fbGyroErr * walking_param_.balance_knee_gain;  // L_KNEE

	balance_angle_[joint_table_["r_ankle_pitch"]] = -1 *-1
			* internal_gain * fbGyroErr * walking_param_.balance_ankle_pitch_gain;  // R_ANKLE_PITCH
	balance_angle_[joint_table_["l_ankle_pitch"]] = - 1
			* internal_gain * fbGyroErr * walking_param_.balance_ankle_pitch_gain;  // L_ANKLE_PITCH

	balance_angle_[joint_table_["r_ankle_roll"]] = - 1 * internal_gain
			* rlGyroErr * walking_param_.balance_ankle_roll_gain;  // R_ANKLE_ROLL
	balance_angle_[joint_table_["l_ankle_roll"]] = - 1 * internal_gain
			* rlGyroErr * walking_param_.balance_ankle_roll_gain;  // L_ANKLE_ROLL
}






