#ifndef LOG_CLASSES_H_
#define LOG_CLASSES_H_

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Header.h>
#include <math.h>

#include <fstream>
#include <iostream>
#include <list>
#include <string>
#include <vector>

using namespace std;

class ConvertGoatPose {
// goatpose_ header same as initial clock header, BUT the seq !
// Attention. goatpose_.header.seq increments and is NOT the same as in clock header (where it is restarted 
// when changing from sit-stand mode to jumping mode).
// ALWAYS start playing your bag file before the first clock/pose message's been sent, otherwise you'll get
// a shift in the data!!!
public:  
	std_msgs::Header timeStamp_incoming_msg_;
	//ros::Duration time_offset_; 

	ros::NodeHandle n_; 
	ros::Publisher pub_GoatPose_; 

	vector<vector<double>> *clock_fields_;
	int nb_clock_fields_;
	bool first_msg_; 

	//----  Specific to custom pose message from goat ----
	bool reverse_order_; 
	// some messages got saved in reversed order in the array (custom messages) 
	int mode_;  
	// different modes available: 
	//0: carthesian + quaternions
	//1: spherical + quaternions [spherical as defined by goat team: r, theta, phi]
	//2: carthesian + euler [qx = roll, qy = pitch, qz = yaw, qw = 0]
	//3: spherical + euler

	void convertGoatPoseCb(const std_msgs::Float32MultiArray::ConstPtr& msg);

	ConvertGoatPose(ros::NodeHandle n, vector<vector<double>> *clock_fields, bool reverse_order, int mode);
	~ConvertGoatPose(void);
};

class ConvertJointState {
public: 
	std_msgs::Header timeStamp_incoming_msg_;
	//ros::Duration time_offset_; 

	ros::NodeHandle n_; 
	ros::Publisher pub_JointState_; 

	vector<vector<double>> *clock_fields_;
	int nb_clock_fields_; 
	bool first_msg_; 

	//----  Specific to custom pose message from goat ----
	bool reverse_order_; 
	// some messages got saved in reversed order in the array (custom messages) 
	
	void convertJointStateCb(const sensor_msgs::JointState::ConstPtr& msg);

	ConvertJointState(ros::NodeHandle n, vector<vector<double>> *clock_fields); 
	~ConvertJointState(void); 
};

class ConvertParameters {
public: 
	std_msgs::Header timeStamp_incoming_msg_;
	ros::Duration time_offset_; 

	ros::NodeHandle n_; 
	ros::Publisher pub_ControlState_; 

	vector<vector<double>> *clock_fields_;
	int nb_clock_fields_; 
	bool first_msg_; 

	void convertParametersCb(const std_msgs::Float32MultiArray::ConstPtr& msg);

	ConvertParameters(ros::NodeHandle n, vector<vector<double>> *clock_fields); 
	~ConvertParameters(void); 
};

#endif // LOG_CLASSES_H_