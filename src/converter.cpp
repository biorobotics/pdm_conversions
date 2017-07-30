#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <pdm_conversions/convert_classes.h>

#include <iostream>
#include <fstream>

using namespace std;


int main(int argc, char **argv)
{

	

	ros::init(argc, argv, "converter");
	ros::NodeHandle nh;

	ConvertGoatPose convert_goat_pose_0(nh, true); 
	ConvertJointState convert_goat_jointState_0(nh); 
	ConvertParameters convert_goat_parameters_0(nh);
	ClockTime my_clock_time_0(&convert_goat_pose_0, &convert_goat_jointState_0, &convert_goat_parameters_0); 

	 
	ros::Subscriber sub_pose = nh.subscribe("/pose", 1000, &ConvertGoatPose::convertGoatPoseCb, &convert_goat_pose_0);
	ros::Subscriber sub_jointState = nh.subscribe("/jointState", 1000, &ConvertJointState::convertJointStateCb, &convert_goat_jointState_0);
	ros::Subscriber sub_params = nh.subscribe("/params", 1000, &ConvertParameters::convertParametersCb, &convert_goat_parameters_0);
	ros::Subscriber sub_clockTime = nh.subscribe("/time", 1000, &ClockTime::clockTimeCb, &my_clock_time_0);
	
	while(ros::ok())
	{
		ros::spinOnce();
	}

	return 0; 
}
