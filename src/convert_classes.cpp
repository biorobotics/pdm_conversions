#include <pdm_conversions/convert_classes.h>

using namespace std;








/// Class: ConvertGoatPose
///
/// Constructor
ConvertGoatPose::ConvertGoatPose(void)
{
	timeStamp_incoming_msg_.seq = 0; 
	//timeStamp_incoming_msg_.stamp = ros::Time::now();

	cout << "{Info} \tConvertGoatPose is being created." << endl; 
}

/// Destructor
ConvertGoatPose::~ConvertGoatPose(void)
{
	cout << "{Info} \tConvertGoatPose is being destructed." << endl; 
}

/// Member function(s)
/// callback function
void ConvertGoatPose::convertGoatPoseCb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	timeStamp_incoming_msg_.seq = timeStamp_incoming_msg_.seq + 1; 
	cout << "-GP " << timeStamp_incoming_msg_.seq <<  endl; 
	/*
	timeStamp_incoming_msg_.stamp = ros::Time::now();
	timeStamp_incoming_msg_.seq = timeStamp_incoming_msg_.seq + 1; 

	cout << "seq " << timeStamp_incoming_msg_.seq << endl; 
	cout << timeStamp_incoming_msg_.stamp << "incoming msg timestamp" << endl;
	if (timeStamp_incoming_msg_.seq == 1)
		cout << "offset not set? " << time_offset << endl; 
	else 
	{
		cout << time_offset_ << " time_offset_" << endl; 
		cout << "= " << timeStamp_incoming_msg_.stamp - time_offset_ << endl; 
	}*/
}




/// Class: ConvertJointState 
///
/// Constructor
ConvertJointState::ConvertJointState(void)
{
	timeStamp_incoming_msg_.seq = 0; 
	//timeStamp_incoming_msg_.stamp = ros::Time::now();

	cout << "{Info} \tConvertJointState is being created." << endl; 
}

/// Destructor
ConvertJointState::~ConvertJointState(void)
{
	cout << "{Info} \tConvertJointState is being destructed." << endl; 
}

/// Member function(s)
/// callback function
void ConvertJointState::convertJointStateCb(const sensor_msgs::JointState::ConstPtr& msg)
{
	timeStamp_incoming_msg_.seq = timeStamp_incoming_msg_.seq + 1;
	cout << "-JS " << timeStamp_incoming_msg_.seq << endl; 
	/*
	timeStamp_incoming_msg_.stamp = ros::Time::now();
	timeStamp_incoming_msg_.seq = timeStamp_incoming_msg_.seq + 1; 

	cout << "seq " << timeStamp_incoming_msg_.seq << endl; 
	cout << timeStamp_incoming_msg_.stamp << "incoming msg timestamp" << endl;
	if (timeStamp_incoming_msg_.seq == 1)
		cout << "offset not set? " << time_offset << endl; 
	else 
	{
		cout << time_offset_ << " time_offset_" << endl; 
		cout << "= " << timeStamp_incoming_msg_.stamp - time_offset_ << endl; 
	}*/
}



/// Class: ConvertParameters
///
/// Constructor
ConvertParameters::ConvertParameters(void)
{
	timeStamp_incoming_msg_.seq = 0; 
	//timeStamp_incoming_msg_.stamp = ros::Time::now();

	cout << "{Info} \tConvertParameters is being created." << endl; 
}

/// Destructor
ConvertParameters::~ConvertParameters(void)
{
	cout << "{Info} \tConvertParameters is being destructed." << endl; 
}

/// Member function(s)
/// callback function
void ConvertParameters::convertParametersCb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	timeStamp_incoming_msg_.seq = timeStamp_incoming_msg_.seq + 1; 
	cout << "-PARA " << timeStamp_incoming_msg_.seq << endl; 
	//timeStamp_incoming_msg_.stamp = ros::Time::now();
	

	//cout << "seq " << timeStamp_incoming_msg_.seq << endl; 
	//cout << timeStamp_incoming_msg_.stamp << "incoming msg timestamp" << endl;
	//if (timeStamp_incoming_msg_.seq == 1)
	//	cout << "Params: time_offset_" << time_offset_.toSec() << endl; 
	/*else 
	{
		
		cout << time_offset_ << " time_offset_" << endl; 
		cout << "= " << timeStamp_incoming_msg_.stamp - time_offset_ << endl; 
	
	}	*/
}



/// Class: ClockTime
/// Gets the first message of the simulated time and gives the offset to all other callback functions.
/// That way we can then publish the messages at their real time-stamp (and not ros::Time::now()).
///
/// Constructor
ClockTime::ClockTime(ConvertGoatPose *goat_pose, ConvertJointState *goat_jointState, ConvertParameters *goat_parameters) 
{
	goat_pose_ptr_ = goat_pose;
	goat_jointState_ptr_ = goat_jointState;
	goat_parameters_ptr_ = goat_parameters;

	timeStamp_incoming_msg_.seq = 0;
	clockTime_current_msg_.seq = 0; 
	clockTime_first_msg_.seq = 0;


	FIRSTMSG_ = true; 

	cout << "{Info} \t"<< timeStamp_incoming_msg_.stamp << " ClockTime is being created." << endl;  
}

/// Destructor
ClockTime::~ClockTime()
{
	cout << "{Info} \tClockTime is being destructed." << endl; 
}

/// Member function(s)
/// Callback-function
void ClockTime::clockTimeCb(const std_msgs::Header::ConstPtr& msg)
{
	timeStamp_incoming_msg_.stamp = ros::Time::now(); 
	timeStamp_incoming_msg_.seq = timeStamp_incoming_msg_.seq + 1; 
	clockTime_current_msg_.seq = msg->seq;
	clockTime_current_msg_.stamp = msg->stamp;

	if(timeStamp_incoming_msg_.seq == 1 && FIRSTMSG_)
	{
		clockTime_first_msg_.seq = 1; 
		clockTime_first_msg_.stamp = msg->stamp; 

		time_offset_ = timeStamp_incoming_msg_.stamp - clockTime_first_msg_.stamp; 

		cout << "*Clock: First offset\t" << time_offset_.toSec() << endl; 

		(*goat_pose_ptr_).time_offset_ = 		time_offset_; 
		(*goat_jointState_ptr_).time_offset_ = 	time_offset_; 
		(*goat_parameters_ptr_).time_offset_ = 	time_offset_; 

		FIRSTMSG_ = false; 
	}
	else
	{
		double diff; 
		diff = (timeStamp_incoming_msg_.stamp - clockTime_current_msg_.stamp).toSec() - time_offset_.toSec(); 
		cout << "*Clock: " << timeStamp_incoming_msg_.seq << "\t" <<  diff*1000 << " [ms]"<< endl; 
	}
}

