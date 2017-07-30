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

using namespace std;


class ConvertGoatPose {
public:  
	std_msgs::Header timeStamp_incoming_msg_;
	ros::Duration time_offset_; 

	bool reverse_order_; 

	ros::NodeHandle n_; 
	ros::Publisher pub_GoatPose_; 

	void convertGoatPoseCb(const std_msgs::Float32MultiArray::ConstPtr& msg);

	ConvertGoatPose(ros::NodeHandle n, bool reverse_order);
	~ConvertGoatPose(void);
};

class ConvertJointState {
public: 
	std_msgs::Header timeStamp_incoming_msg_;
	ros::Duration time_offset_; 

	ros::NodeHandle n_; 
	ros::Publisher pub_JointState_; 

	void convertJointStateCb(const sensor_msgs::JointState::ConstPtr& msg);

	ConvertJointState(ros::NodeHandle n); 
	~ConvertJointState(void); 
};

class ConvertParameters {
public: 
	std_msgs::Header timeStamp_incoming_msg_;
	ros::Duration time_offset_; 

	ros::NodeHandle n_; 
	ros::Publisher pub_Params_; 

	void convertParametersCb(const std_msgs::Float32MultiArray::ConstPtr& msg);

	ConvertParameters(ros::NodeHandle n); 
	~ConvertParameters(void); 
};

class ClockTime {
public:
	std_msgs::Header timeStamp_incoming_msg_;
	std_msgs::Header clockTime_current_msg_; 
	std_msgs::Header clockTime_first_msg_;
	ros::Duration time_offset_; 
	bool FIRSTMSG_; 

	ConvertGoatPose* 		goat_pose_ptr_; 
	ConvertJointState* 		goat_jointState_ptr_;
	ConvertParameters* 		goat_parameters_ptr_; 

	void clockTimeCb(const std_msgs::Header::ConstPtr& msg);

	ClockTime(ConvertGoatPose *goat_pose, ConvertJointState *goat_jointState, ConvertParameters *goat_parameters); 
	~ClockTime(); 
};





#endif // LOG_CLASSES_H_
