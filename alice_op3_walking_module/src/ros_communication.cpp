/*
 * ros_communication.cpp
 *
 *  Created on: Jun 9, 2018
 *      Author: robotemperor
 */
#include "alice_op3_walking_module/alice_op3_walking_module.h"

using namespace alice_walking;


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

	target_position_.resize(1,12);
	target_position_.fill(0);
	init_position_.resize(1,12);
	init_position_.fill(0);

	// Alice kinematics

	l_kinematics_ = new heroehs_math::Kinematics;
	r_kinematics_ = new heroehs_math::Kinematics;


	for(int i = 0;i<12; i++)
	{
		balance_angle_[i] = 0.0;
		angle_[i] = 0.0;
	}




	currentGyroX = 0;
	currentGyroY = 0;
	currentGyroZ = 0;
	tf_current_gyro_x = 0;
	tf_current_gyro_y = 0;
	tf_current_gyro_z = 0;
	rl_gyro_err = 0.0;
	fb_gyro_err = 0,0;
	internal_gain = 0.0;
	goal_position = 0.0;

//zmp
	zmp_cal = new alice::ZmpCalculationFunc;
	currentFX_l=0.0;
	currentFY_l=0.0;
	currentFZ_l=0.0;
	currentTX_l=0.0;
	currentTY_l=0.0;
	currentTZ_l=0.0;
	currentFX_r=0.0;
	currentFY_r=0.0;
	currentFZ_r=0.0;
	currentTX_r=0.0;
	currentTY_r=0.0;
	currentTZ_r=0.0;
}

WalkingModule::~WalkingModule()
{
	queue_thread_.join();
}

void WalkingModule::queueThread()
{
	ros::NodeHandle ros_node;
	ros::CallbackQueue callback_queue;

	ros_node.setCallbackQueue(&callback_queue);

	/* publish topics */
	status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("robotis/status", 1);
	zmp_fz_pub = ros_node.advertise<std_msgs::Float64MultiArray>("/zmp",100);
	l_leg_point_xyz_pub = ros_node.advertise<geometry_msgs::Vector3>("/l_leg_point_xyz",100);
	r_leg_point_xyz_pub = ros_node.advertise<geometry_msgs::Vector3>("/r_leg_point_xyz",100);

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
	currentFX_l = (double) msg->force_x_raw_l;
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

	zmp_cal->ftSensorDataLeftGet(currentFX_l, currentFY_l, currentFZ_l, currentTX_l, currentTY_l, currentTZ_l);
	zmp_cal->ftSensorDataRightGet(currentFX_r, currentFY_r, currentFZ_r, currentTX_r, currentTY_r, currentTZ_r);
	zmp_cal->jointStateGetForTransForm(l_kinematics_->joint_radian, r_kinematics_->joint_radian);
	zmp_cal->ZmpCalculationResult();
}
bool WalkingModule::getWalkigParameterCallback(op3_walking_module_msgs::GetWalkingParam::Request &req,
		op3_walking_module_msgs::GetWalkingParam::Response &res)
{
	res.parameters = walking_param_;

	return true;
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
	/*double smp_time = control_cycle_msec_ * 0.001;
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

	init_pose_count_ = 0;*/
}
void WalkingModule::gyroRotationTransformation(double gyro_z, double gyro_y, double gyro_x)
{
	Eigen::MatrixXd tf_gyro_value_;
	tf_gyro_value_.resize(3,1);
	tf_gyro_value_.fill(0);
	tf_gyro_value_(0,0) =  gyro_x;
	tf_gyro_value_(1,0) =  gyro_y;
	tf_gyro_value_(2,0) =  gyro_z;

	tf_gyro_value_ = (robotis_framework::getRotationZ(-M_PI/2)*robotis_framework::getRotationY(-M_PI))*tf_gyro_value_;
	tf_current_gyro_x = tf_gyro_value_(0,0);
	tf_current_gyro_y = tf_gyro_value_(1,0);
	tf_current_gyro_z = tf_gyro_value_(2,0);
}

