#include <pdm_conversions/convert_classes.h>

using namespace std;

const double EPSILON = 10^(-6); 
const double PI = 3.141592653589793; 
const double METER_PER_INCH = 0.0254;
const double RAD_PER_DEG = PI/180;   


/// Class: ConvertGoatPose
///
/// Constructor
ConvertGoatPose::ConvertGoatPose(ros::NodeHandle n, bool reverse_order)
{
	n_ = n; 
	reverse_order_ = reverse_order; 
	timeStamp_incoming_msg_.seq = 0; 
	pub_GoatPose_ = n_.advertise<geometry_msgs::TransformStamped>("goat/pose_cart", 100);
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
	timeStamp_incoming_msg_.stamp = ros::Time::now();
	timeStamp_incoming_msg_.seq = timeStamp_incoming_msg_.seq + 1; 
	int it_start = 3;  
	int nb_elements = 13;

	if (timeStamp_incoming_msg_.seq > 1)
	{
		geometry_msgs::TransformStamped goatpose_;

		goatpose_.header.seq 	= timeStamp_incoming_msg_.seq; 
		goatpose_.header.stamp 	= timeStamp_incoming_msg_.stamp - time_offset_; 
		goatpose_.header.frame_id 	= "robot_end_effector"; 
		goatpose_.child_frame_id 	= "robot_body";
	
		if (reverse_order_ == true)
		{
			goatpose_.transform.translation.x = msg->data[nb_elements -1 - it_start] * METER_PER_INCH; 
			goatpose_.transform.translation.y = msg->data[nb_elements -1 - it_start+1] * METER_PER_INCH;
			goatpose_.transform.translation.z = msg->data[nb_elements -1 - it_start+2] * METER_PER_INCH;

			// transform to unit quaternion
			double qx = msg->data[nb_elements -1 - it_start+3]; 
			double qy = msg->data[nb_elements -1 - it_start+4]; 
			double qz = msg->data[nb_elements -1 - it_start+5]; 
			double qw = msg->data[nb_elements -1 - it_start+6]; 
			double norm = sqrt(qx*qx + qy*qy + qz*qz + qw*qw); 

			if (norm > EPSILON)
			{
				goatpose_.transform.rotation.x = qx/norm; 
				goatpose_.transform.rotation.y = qy/norm; 
				goatpose_.transform.rotation.z = qz/norm;
				goatpose_.transform.rotation.w = qw/norm;	
			}
			else
			{
				cout << "{Warning}: Could not transform quaternion to unit quaternion. Quaternion not transformed." << endl; 
				goatpose_.transform.rotation.x = qx; 
				goatpose_.transform.rotation.y = qy; 
				goatpose_.transform.rotation.z = qz;
				goatpose_.transform.rotation.w = qw;
			}
		}
		else
		{
			goatpose_.transform.translation.x = msg->data[it_start] * METER_PER_INCH; 
			goatpose_.transform.translation.y = msg->data[it_start+1] * METER_PER_INCH;
			goatpose_.transform.translation.z = msg->data[it_start+2] * METER_PER_INCH;

			// transform to unit quaternion
			double qx = msg->data[it_start+3]; 
			double qy = msg->data[it_start+4]; 
			double qz = msg->data[it_start+5]; 
			double qw = msg->data[it_start+6]; 
			double norm = sqrt(qx*qx + qy*qy + qz*qz + qw*qw); 

			if (norm > EPSILON)
			{
				goatpose_.transform.rotation.x = qx/norm; 
				goatpose_.transform.rotation.y = qy/norm; 
				goatpose_.transform.rotation.z = qz/norm;
				goatpose_.transform.rotation.w = qw/norm;
			}
			else
			{
				goatpose_.transform.rotation.x = qx; 
				goatpose_.transform.rotation.y = qy; 
				goatpose_.transform.rotation.z = qz;
				goatpose_.transform.rotation.w = qw;
			}
		}
		pub_GoatPose_.publish(goatpose_);  
	} 
	//cout << "-GP " << timeStamp_incoming_msg_.seq <<  endl; 
}


/// Class: ConvertJointState 
///
/// Constructor
ConvertJointState::ConvertJointState(ros::NodeHandle n)
{
	n_ = n; 
	timeStamp_incoming_msg_.seq = 0; 
	pub_JointState_ = n_.advertise<sensor_msgs::JointState>("goat/jointState", 100); 
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
	timeStamp_incoming_msg_.stamp = ros::Time::now(); 
	timeStamp_incoming_msg_.seq = timeStamp_incoming_msg_.seq + 1;

	if (timeStamp_incoming_msg_.seq > 1)
	{
		sensor_msgs::JointState goatjointstate_; 
		goatjointstate_.header.stamp = timeStamp_incoming_msg_.stamp - time_offset_; 
		
		// keeping the same sequence, because there we can get the transition from sit-stand procedure to jumping (restarts at 0)
		goatjointstate_.header.seq = msg->header.seq; 
		goatjointstate_.name = msg->name;

		// Conversion to SI units if not already in SI units
		goatjointstate_.effort 		= msg->effort; 		// [N/m]
		goatjointstate_.velocity 	= msg->velocity; 	// [rad/s]
		goatjointstate_.position[1] = msg->position[1] * RAD_PER_DEG; // [rad]
		goatjointstate_.position[2] = msg->position[2] * RAD_PER_DEG; // [rad]
		goatjointstate_.position[3] = msg->position[3] * RAD_PER_DEG; // [rad]
		pub_JointState_.publish(goatjointstate_); 
	}
	//cout << "-JS " << timeStamp_incoming_msg_.seq << endl;
}



/// Class: ConvertParameters
///
/// Constructor
ConvertParameters::ConvertParameters(ros::NodeHandle n)
{
	n_ = n; 
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

