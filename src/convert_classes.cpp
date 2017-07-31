#include <pdm_conversions/convert_classes.h>

using namespace std;

const double EPSILON = 10^(-6); 
const double PI = 3.141592653589793; 
const double METER_PER_INCH = 0.0254;
const double RAD_PER_DEG = PI/180;   


/// Class: ConvertGoatPose
/// Constructor
ConvertGoatPose::ConvertGoatPose(ros::NodeHandle n, vector<vector<double>> *clock_fields, bool reverse_order, int mode)
{
	n_ 				 = n; 
	clock_fields_ 	 = clock_fields; 
	nb_clock_fields_ = (*clock_fields_).size(); 
	reverse_order_ 	 = reverse_order; 
	mode_ 			 = mode; // cf. header file for definitions
	if (mode_ == 0)
		pub_GoatPose_ 	 = n_.advertise<geometry_msgs::TransformStamped>("goat/pose_cq", 100); // carthesian + quaternion
	else if (mode_ == 1)
		pub_GoatPose_	 = n_.advertise<geometry_msgs::TransformStamped>("goat/pose_sq", 100); // spherical + quaternion
	else if (mode_ == 2)
		pub_GoatPose_	 = n_.advertise<geometry_msgs::TransformStamped>("goat/pose_ce", 100); // carthesian + euler
	else if (mode_ == 3)
		pub_GoatPose_	 = n_.advertise<geometry_msgs::TransformStamped>("goat/pose_se", 100); // spherical + euler
	else
		cout << "{Warning} \t No message will be published, chosen mode is invalid (choose 0,1,2 or 3 and NOT : " << mode_ << ")" << endl; 

	first_msg_ 		= true;  

	cout << "{Info} \tConvertGoatPose is being created." << endl; 
}

/// Destructor
ConvertGoatPose::~ConvertGoatPose(void)
{
	cout << "{Info} \tConvertGoatPose is being destructed." << endl; 
}

/// Callback function
void ConvertGoatPose::convertGoatPoseCb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	timeStamp_incoming_msg_.stamp = ros::Time::now();
	
	int nb_elements = 13;
	geometry_msgs::TransformStamped goatpose_;

	if (first_msg_)
	{
		timeStamp_incoming_msg_.seq = 0; 
		first_msg_ = false;
	}
	else
		timeStamp_incoming_msg_.seq = timeStamp_incoming_msg_.seq + 1; 
	
	if (timeStamp_incoming_msg_.seq < nb_clock_fields_ )
	{	 
		// Header: goatpose_ header same as initial clock header, BUT the seq !
		// Attention. goatpose_.header.seq increments and is NOT the same as in clock header (where it is restarted 
		// when changing from sit-stand mode to jumping mode).
		// ALWAYS start playing your bag file before the first clock/pose message's been sent, otherwise you'll get
		// a shift in the data!!!

		goatpose_.header.seq 		= timeStamp_incoming_msg_.seq; 
		// reading from array (extracted from .csv file):
		goatpose_.header.stamp.sec 	= (*clock_fields_)[timeStamp_incoming_msg_.seq][1];
		goatpose_.header.stamp.nsec	= (*clock_fields_)[timeStamp_incoming_msg_.seq][2];

		goatpose_.child_frame_id 		= "body";
		if(mode_ == 0)
			goatpose_.header.frame_id 	= "end_effector_[car_quat]";	
		else if(mode_ == 1)
			goatpose_.header.frame_id 	= "end_effector_[sph_quat]";		
		else if(mode_ == 2)
			goatpose_.header.frame_id 	= "end_effector_[car_eul]";		
		else if(mode_ == 3)
			goatpose_.header.frame_id 	= "end_effector_[sph_eul]";	
		else
			cout << "{Warning} \tMode selected not kown, could not write goatpose_. Select mode 0,1,2 or 3. Your mode =" << mode_ << "." << endl; 
			 

		if (mode_ == 0 || mode_ == 2)
		{
			// Translation: carthesian
			int it_start = 3;  
			if(reverse_order_ == true)
			{
				goatpose_.transform.translation.x = msg->data[nb_elements -1 - it_start]   * METER_PER_INCH; //x[m]
				goatpose_.transform.translation.y = msg->data[nb_elements -1 - it_start+1] * METER_PER_INCH; //y[m]
				goatpose_.transform.translation.z = msg->data[nb_elements -1 - it_start+2] * METER_PER_INCH; //z[m]					
			}
			else
			{
				goatpose_.transform.translation.x = msg->data[it_start]   * METER_PER_INCH; //x[m]
				goatpose_.transform.translation.y = msg->data[it_start+1] * METER_PER_INCH; //y[m]
				goatpose_.transform.translation.z = msg->data[it_start+2] * METER_PER_INCH; //z[m]		
			}
		}
		else if(mode_ == 1 || mode_ == 3)
		{
			// Translation: spherical
			int it_start = 0; 
			if(reverse_order_ == true)
			{
				goatpose_.transform.translation.x = msg->data[nb_elements -1 - it_start]   * METER_PER_INCH; 	//r[m]
				goatpose_.transform.translation.y = msg->data[nb_elements -1 - it_start+1]; 					//theta[rad]
				goatpose_.transform.translation.z = msg->data[nb_elements -1 - it_start+2]; 					//phi [rad]
			}
			else
			{
				goatpose_.transform.translation.x = msg->data[it_start] * METER_PER_INCH; 	//r[m]
				goatpose_.transform.translation.y = msg->data[it_start+1]; 					//theta[rad]
				goatpose_.transform.translation.z = msg->data[it_start+2]; 					//phi [rad]	
			}
		}
			
		if (mode_ == 0 || mode_ == 1)
		{
			// Rotation: unit quaternion
			int it_start = 6;
			double norm, qw, qx, qy, qz;

			if(reverse_order_ == true)
			{
				qw = msg->data[nb_elements -1 - it_start+3];
				qx = msg->data[nb_elements -1 - it_start]; 
				qy = msg->data[nb_elements -1 - it_start+1]; 
				qz = msg->data[nb_elements -1 - it_start+2];  
				norm = sqrt(qx*qx + qy*qy + qz*qz + qw*qw); 
			}
			else
			{
				qw = msg->data[it_start+3];
				qx = msg->data[it_start]; 
				qy = msg->data[it_start+1]; 
				qz = msg->data[it_start+2]; 
				norm = sqrt(qx*qx + qy*qy + qz*qz + qw*qw); 
			}
	
			
			if (norm > EPSILON)
			{
				goatpose_.transform.rotation.x = qx/norm; 
				goatpose_.transform.rotation.y = qy/norm; 
				goatpose_.transform.rotation.z = qz/norm;
				goatpose_.transform.rotation.w = qw/norm;	
			}
			else
			{
				cout << "{Warning} \tCould not transform quaternion to unit quaternion. Quaternion not transformed." << endl; 
				goatpose_.transform.rotation.x = qx; 
				goatpose_.transform.rotation.y = qy; 
				goatpose_.transform.rotation.z = qz;
				goatpose_.transform.rotation.w = qw;
			}	
		}
		else if(mode_ == 2 || mode_ == 3)
		{
			int it_start = 10; 
			if(reverse_order_ == true)
			{
				goatpose_.transform.rotation.x = msg->data[nb_elements -1 - it_start] * RAD_PER_DEG; 		// Roll [rad]
				goatpose_.transform.rotation.y = msg->data[nb_elements -1 - it_start + 1] * RAD_PER_DEG; 	// Pitch [rad]
				goatpose_.transform.rotation.z = msg->data[nb_elements -1 - it_start + 2] * RAD_PER_DEG; 	// Yaw [rad]
			}
			else
			{
				goatpose_.transform.rotation.x = msg->data[it_start] * RAD_PER_DEG; 		// Roll [rad]
				goatpose_.transform.rotation.y = msg->data[it_start + 1] * RAD_PER_DEG; 	// Pitch [rad]
				goatpose_.transform.rotation.z = msg->data[it_start + 2] * RAD_PER_DEG; 	// Yaw [rad]	
			}
		}			
	
		// Publishing message -------------------------------------------------------------
		pub_GoatPose_.publish(goatpose_); 
	}
	else
		cout << "{Warning} \t Cannot publish goatpose as there are no more clock messages." << endl; 
}


/// Class: ConvertJointState 
/// Constructor
ConvertJointState::ConvertJointState(ros::NodeHandle n, vector<vector<double>> *clock_fields)
{
	n_ 				 = n; 
	clock_fields_ 	 = clock_fields; 
	nb_clock_fields_ = (*clock_fields).size();  
	pub_JointState_ = n_.advertise<sensor_msgs::JointState>("goat/jointState", 100); 
	
	first_msg_ 		= true;  

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

	sensor_msgs::JointState goatjointstate_;
	
	if (first_msg_)
	{
		timeStamp_incoming_msg_.seq = 0; 
		first_msg_ = false;
	}
	else
		timeStamp_incoming_msg_.seq = timeStamp_incoming_msg_.seq + 1; 

	// Header: Rewriting the original one (because it did restart at 0 when transitioning from sit-stand mode to jumping).
	// The sit-stand mode to jumping mode transition is saved in a different topic among other state variables. 

	goatjointstate_.header.seq 			= timeStamp_incoming_msg_.seq; 
	// reading from array (extracted from .csv file):
	goatjointstate_.header.stamp.sec 	= (*clock_fields_)[timeStamp_incoming_msg_.seq][1]; 
	goatjointstate_.header.stamp.nsec	= (*clock_fields_)[timeStamp_incoming_msg_.seq][2];

	// Name is simply copied (see goat-team naming motors)
	goatjointstate_.name = msg->name;

	// Changing to SI-units if not already
	goatjointstate_.effort 		= msg->effort; 					  // [N/m]
	goatjointstate_.velocity 	= msg->velocity; 				  // [rad/s]
	goatjointstate_.position[1] = msg->position[1] * RAD_PER_DEG; // [rad]
	goatjointstate_.position[2] = msg->position[2] * RAD_PER_DEG; // [rad]
	goatjointstate_.position[3] = msg->position[3] * RAD_PER_DEG; // [rad]
	
	// Publishing message -------------------------------------------------------------
	pub_JointState_.publish(goatjointstate_); 
}


/// Class: ConvertParameters
///
/// Constructor
ConvertParameters::ConvertParameters(ros::NodeHandle n, vector<vector<double>> *clock_fields)
{
	n_ 				  = n; 
	clock_fields_ 	  = clock_fields; 
	nb_clock_fields_  = (*clock_fields).size();  
	pub_ControlState_ = n_.advertise<std_msgs::Float32MultiArray>("goat/controlState", 100); 
	
	first_msg_ 		  = true;  

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
	/*

	// State1: 0 = unknown, 1 = sit-stand, 2 = jumping
	// State2: 0 = unknown, 1 = in the air, 2 = on ground
	// l_landing, l_compressed, l_thrust
	// k_landing, k_thrust (can thrust be transformed in k_trust? if not what is behavior?)

	*/
}