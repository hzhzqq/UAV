/**
 * @brief Offboard control test
 * @file offboard_control.h
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Andre Nguyen <andre-phu-van.nguyen@polymtl.ca>
 *
 * @addtogroup sitl_test
 * @{
 */
/*
 * Copyright 2015 Nuno Marques, Andre Nguyen.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#ifndef OFFBOARD_CONTROL_HPP
#define OFFBOARD_CONTROL_HPP

#include <ros/ros.h>

#include <array>
#include <random>
#include <angles/angles.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>
// #include <test_mavros/sitl_test/sitl_test.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <spirecv_msgs/TargetsInFrame.h>
#include <spirecv_msgs/Target.h>
#include <nav_msgs/Odometry.h>
#include <pid_controller.hpp>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#include <std_msgs/Float64.h>

#include <helper_functions.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
/**
 * @brief Offboard controller tester
 *
 * Tests offboard position, velocity and acceleration control
 *
 */
using namespace Eigen;

typedef enum {
	POSITION,
	VELOCITY,
	ACCELERATION
} control_mode;

typedef enum {
	CIRCLE_PASS,
	PRECISE_LAND,
	FOLLOW_LINE,
	FOLLOW_VEHICLE
} action;


typedef enum {
	TAKEOFFING,
	TAKEOFF_DONE,
	FIRST_O_DONE,
	SECONDE_O_DONE
} circle_pass_phase;

static constexpr float SIGMA_NORM	= 0.001f;

typedef enum {
	POSITION_TARGET_TYPEMASK_X_IGNORE	        = 1 << 0 , 
	POSITION_TARGET_TYPEMASK_Y_IGNORE	        = 1 << 1 , 
	POSITION_TARGET_TYPEMASK_Z_IGNORE	        = 1 << 2 , 
	POSITION_TARGET_TYPEMASK_VX_IGNORE	        = 1 << 3 , 
	POSITION_TARGET_TYPEMASK_VY_IGNORE	        = 1 << 4 , 
	POSITION_TARGET_TYPEMASK_VZ_IGNORE	        = 1 << 5 , 
	POSITION_TARGET_TYPEMASK_AX_IGNORE	        = 1 << 6 , 
	POSITION_TARGET_TYPEMASK_AY_IGNORE	        = 1 << 7 , 
	POSITION_TARGET_TYPEMASK_AZ_IGNORE	        = 1 << 8 , 
	POSITION_TARGET_TYPEMASK_FORCE_SET	        = 1 << 9 , 
	POSITION_TARGET_TYPEMASK_YAW_IGNORE         = 1 << 10,
	POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE    = 1 << 11
} POSITION_TARGET_TYPEMASK;

class OffboardControl {
public:
	OffboardControl(ros::NodeHandle nh ) :
		nh_sp(nh),
		local_pos_sp_pub(nh_sp.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10)),
		local_pos_sp_raw_pub(nh_sp.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10)),

		vel_sp_pub(nh_sp.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10)), // TODO, do not have this topic
		
		local_pos_sub(nh_sp.subscribe("/mavros/local_position/pose", 10, &OffboardControl::local_pos_cb, this)),
    	local_velocity_sub(nh_sp.subscribe("/mavros/local_position/velocity_local", 10, &OffboardControl::LocalVelocityCB,this)),
		//yaw_deg_sub(nh_sp.subscribe("/mavros/global_position/compass_hdg", 10, &OffboardControl::yaw_deg_cb,this)), 
		yaw_deg_sub(nh_sp.subscribe("/camera/odom/sample", 10, &OffboardControl::yaw_deg_cb,this)),
		/* https://www.wolai.com/qRqAyE8zo1AQCYTH89k8ci 圆环检测topic*/ 
		// 如果环境中有两个圆环
		// circle_targets_sub(nh_sp.subscribe("/mavros/local_position/local", 10, &OffboardControl::two_circle_targets_cb, this)),
		circle_targets_sub(nh_sp.subscribe("/uav1/spirecv/ellipse_detection", 10, &OffboardControl::one_circle_targets_cb, this)),
		
		// land_H_targets_sub(nh_sp.subscribe("/mavros/local_position/local", 10, &OffboardControl::, this)),
		// land_X_targets_sub(nh_sp.subscribe("/mavros/local_position/local", 10, &OffboardControl::, this)),
		// land_QR_targets_sub(nh_sp.subscribe("/uav1/spirevision/landing_marker_detection", 10, &OffboardControl::, this)),
		// follow_line_targets_sub(nh_sp.subscribe("/mavros/local_position/local", 10, &OffboardControl::, this)),
		// vehicle_targets_sub(nh_sp.subscribe("/mavros/local_position/local", 10, &OffboardControl::, this)),

		state_sub(nh_sp.subscribe("mavros/state", 10, &OffboardControl::state_cb, this)),

		threshold(threshold_definition())
	{ 
		_init_done = false;
		finish_offboard_takeoff = false;
		_real_targets_num = 0;
		arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
		set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
		_yaw_lock = false;
	};
	void init();
	void do_offboard_takeoff();
	void do_land();
	void spin(int argc, char *argv[]) ;
	void precise_land_path_motion(ros::Rate loop_rate, control_mode mode);
	void circle_pass_path_motion(ros::Rate loop_rate, control_mode mode);

	void one_circle_pass_path_motion(ros::Rate loop_rate, control_mode mode);
	void wait_on_location(double wait_seconds);
	void wait_and_move(geometry_msgs::PoseStamped target, bool set_yaw_flag = false);
	void local_pos_cb(const geometry_msgs::PoseStampedConstPtr& msg);
	void LocalVelocityCB(const geometry_msgs::TwistStampedConstPtr& msg);
	//void yaw_deg_cb(const std_msgs::Float64ConstPtr& msg);
	void yaw_deg_cb(const nav_msgs::Odometry::ConstPtr& msg);
	void one_circle_targets_cb(const spirecv_msgs::TargetsInFrameConstPtr& msg);
	void two_circle_targets_cb(const spirecv_msgs::TargetsInFrameConstPtr& msg);
	Vector3d longer_away_pos(Vector3d pos);
	Vector3d goto_close_pos(Vector3d pos,float dist = 2.5);

	bool _compute_heading_from_2D_vector(float &heading, Vector2f v);

	void distinguish_2_circles_pos();
	void state_cb(const mavros_msgs::State::ConstPtr& msg);

	Vector3d cam_pos_trans_enu(Vector3d vehicle_local_pos, Vector3d cam_pos);
	double to_radians(double degrees){ return degrees / (180.0/M_PI);}
	double to_degrees(double radians){ return radians * (180.0/M_PI);}

private:
	// TestSetup test;
	pidcontroller::PIDController pid;

	double rate;
	bool use_pid;
	bool _init_done;

	double linvel_p_gain;
	double linvel_i_gain;
	double linvel_d_gain;
	double linvel_i_max;
	double linvel_i_min;

	double yawrate_p_gain;
	double yawrate_i_gain;
	double yawrate_d_gain;
	double yawrate_i_max;
	double yawrate_i_min;

	control_mode mode;
	action action_mode;

	ros::NodeHandle nh_sp;
	ros::Publisher local_pos_sp_pub;
	ros::Publisher local_pos_sp_raw_pub;
	ros::Publisher vel_sp_pub;

    ros::Subscriber state_sub ;
    ros::ServiceClient arming_client ;
    ros::ServiceClient set_mode_client ;

	//本地坐标系(ENU)位置数据订阅者
	ros::Subscriber local_pos_sub;
	//本地坐标系(ENU)速度数据订阅者
	ros::Subscriber local_velocity_sub;
	ros::Subscriber yaw_deg_sub;  /* 订阅偏航角*/

	ros::Subscriber circle_targets_sub; 		/* 圆环识别结果订阅 */ //TODO
	ros::Subscriber land_H_targets_sub; 		/* 圆环识别结果订阅 */
	ros::Subscriber land_X_targets_sub; 		/* 圆环识别结果订阅 */
	ros::Subscriber land_QR_targets_sub; 		/* 圆环识别结果订阅 */
	ros::Subscriber follow_line_targets_sub;	/* 圆环识别结果订阅 */
	ros::Subscriber vehicle_targets_sub;		/* 圆环识别结果订阅 */

    bool finish_offboard_takeoff;
	bool _finished_land;

	geometry_msgs::PoseStamped localpos, ps;
	geometry_msgs::TwistStamped vs;
	double _yaw_state;
	std_msgs::Float64 _yaw_deg;   /* 偏航角数据*/
	float _yaw_setpoint; // 期望偏航角
	bool _yaw_lock; // 偏航角锁定flag

	//速度数据
	geometry_msgs::TwistStamped local_velocity;

	spirecv_msgs::TargetsInFrame circle_targets; //TODO 
	// spirecv_msgs::TargetsInFrame circle_targets; //TODO spirecv_msgs
	spirecv_msgs::Target circle_targets_1; //近的那个
	spirecv_msgs::Target circle_targets_2; //TODO
    int _real_targets_num;  //实际目标有几个，只可能是0，1，2个
	geometry_msgs::PoseStamped _tar_pos1_when_wait;


	circle_pass_phase circle_pass_stage;

	Eigen::Vector3d current;
	mavros_msgs::State current_state;


	std::array<double, 100> threshold;

	/* -*- helper functions -*- */

	/**
	 * @brief Defines single position setpoint
	 */
	Eigen::Vector3d pos_setpoint(int tr_x, int tr_y, int tr_z){
		/** @todo Give possibility to user define amplitude of movement (square corners coordinates)*/
		return Eigen::Vector3d(tr_x * 2.0f, tr_y * 2.0f, tr_z * 1.0f);	// meters
	}

	/**
	 * @brief Defines circle path
	 */
	Eigen::Vector3d circle_shape(int angle){
		/** @todo Give possibility to user define amplitude of movement (circle radius)*/
		double r = 5.0f;	// 5 meters radius

		return Eigen::Vector3d(r * cos(angles::from_degrees(angle)),
				r * sin(angles::from_degrees(angle)),
				1.0f);
	}

	/**
	 * @brief Defines Gerono lemniscate path
	 */
	Eigen::Vector3d eight_shape(int angle){
		/** @todo Give possibility to user define amplitude of movement (vertical tangent size)*/
		double a = 5.0f;	// vertical tangent with 5 meters size

		return Eigen::Vector3d(a * cos(angles::from_degrees(angle)),
				a * sin(angles::from_degrees(angle)) * cos(angles::from_degrees(angle)),
				1.0f);
	}

	/**
	 * @brief Defines ellipse path
	 */
	Eigen::Vector3d ellipse_shape(int angle){
		/** @todo Give possibility to user define amplitude of movement (tangent sizes)*/
		double a = 5.0f;	// major axis
		double b = 2.0f;	// minor axis

		// rotation around y-axis
		return Eigen::Vector3d(a * cos(angles::from_degrees(angle)),
				0.0f,
				2.5f + b * sin(angles::from_degrees(angle)));
	}

	void ellipse_path_motion(ros::Rate loop_rate, control_mode mode);
	void eight_path_motion(ros::Rate loop_rate, control_mode mode);
	void circle_path_motion(ros::Rate loop_rate, control_mode mode);
	void square_path_motion(ros::Rate loop_rate, control_mode mode);


	/**
	 * @brief Gaussian noise generator for accepted position threshold
	 */
	std::array<double, 100> threshold_definition(){
		std::random_device rd;
		std::mt19937 gen(rd());
		std::array<double, 100> th_values;

		std::normal_distribution<double> th(0.1f,0.05f);

		for (auto &value : th_values) {
			value = th(gen);
		}
		return th_values;
	}


};

#endif
