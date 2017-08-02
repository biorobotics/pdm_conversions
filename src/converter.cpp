#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <pdm_conversions/convert_classes.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "converter");
	ros::NodeHandle nh;
	
	int a;
	ros::param::param("converter/file_id", a, 0); 
	cout<<"check the argument parsing:  "<< a <<endl;


	// ifstream in("/home/puneet/Desktop/dataset2/100B_mixFlatG_1/100B_mixFlatG_1_simTime.csv"); // Puneet
	ifstream in("/media/pea/Windows7_OS/Users/Astrid/Desktop/dataset2/100B_mixFlatG_1/100B_mixFlatG_1_simTime.csv"); // Astrid


	vector<vector<double>> clock_fields;
	if(in)
	{
		// Saving .csv clock data to an array
		string line; 
		while(getline(in, line))
		{
			stringstream sep(line); 
			string field; 

			clock_fields.push_back(vector<double>());

			while (getline(sep, field, ','))
			{
				clock_fields.back().push_back(stod(field));
			}
		}

		// Creating the objects containing the callback function and the whole structure		
		ConvertGoatPose convert_goat_pose_0(nh, &clock_fields, false, 0); 
		ConvertJointState convert_goat_jointState_0(nh, &clock_fields); 
		ConvertParameters convert_goat_parameters_0(nh, &clock_fields);
		
	 	// Subscribe to the different topics that need their timestamp, metric units and proper ros type:
		ros::Subscriber sub_pose = nh.subscribe("/pose", 1000, &ConvertGoatPose::convertGoatPoseCb, &convert_goat_pose_0);
		ros::Subscriber sub_jointState = nh.subscribe("/jointState", 1000, &ConvertJointState::convertJointStateCb, &convert_goat_jointState_0);
		ros::Subscriber sub_params = nh.subscribe("/params", 1000, &ConvertParameters::convertParametersCb, &convert_goat_parameters_0);
		
		while(ros::ok())
		{
			ros::spinOnce();
		}
	}
	return 0; 
}
