#include <pdm_conversions/convert_classes.h>

using namespace std;

const double EPSILON = 10^(-6); 
const double PI = 3.141592653589793; 
const double METER_PER_INCH = 0.0254;
const double RAD_PER_DEG = PI/180;   


// ******************************************************************************************************************************
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
		pub_GoatPose_ 	 = n_.advertise<geometry_msgs::TransformStamped>("goat/pose_cq", 100); // cartesian + quaternion
	else if (mode_ == 1)
		pub_GoatPose_	 = n_.advertise<geometry_msgs::TransformStamped>("goat/pose_sq", 100); // spherical + quaternion
	else if (mode_ == 2)
		pub_GoatPose_	 = n_.advertise<geometry_msgs::TransformStamped>("goat/pose_ce", 100); // carthesian + euler
	else if (mode_ == 3)
		pub_GoatPose_	 = n_.advertise<geometry_msgs::TransformStamped>("goat/pose_se", 100); // spherical + euler
	else
		cout << "{Warning} \t No message will be published, chosen mode is invalid (choose 0, 1, 2 or 3 and NOT : " << mode_ << ")" << endl; 

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
	timeStamp_incoming_msg_.stamp = ros::Time::now(); // This is just a precaution, debugging and stats.
	
	if (first_msg_)
	{
		timeStamp_incoming_msg_.seq = 0; 
		first_msg_ = false;
	}
	else
		timeStamp_incoming_msg_.seq = timeStamp_incoming_msg_.seq + 1; 

	// Correcting the push_back / push_front from the different modes 
	// mode sit-stand 	--> reverse_order_ = false (initialized by user)
	// mode jumping 	--> reverse_order_ = true (when we read a 0 again in the .csv file stored in the array)	
	if (~reverse_order_ && (*clock_fields_)[timeStamp_incoming_msg_.seq][0]==0 && timeStamp_incoming_msg_.seq > 0)
	{
		reverse_order_ = true;
	}

	if (timeStamp_incoming_msg_.seq < nb_clock_fields_ )
	{	 
		geometry_msgs::TransformStamped goatpose_;

		goatpose_.header.seq 		= timeStamp_incoming_msg_.seq; 
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
		
		double norm, qw, qx, qy, qz;

		switch (mode_) // This code has many lines (it could be shorter), but this is also more readable than my origninal code
		{
			case 0: // Case 0 = carthesian position and quaternions rotation
				if(reverse_order_ == true)
				{
					goatpose_.transform.translation.x = msg->data[9] * METER_PER_INCH; //x[m]
					goatpose_.transform.translation.y = msg->data[8] * METER_PER_INCH; //y[m]
					goatpose_.transform.translation.z = msg->data[7] * METER_PER_INCH; //z[m]	
					qx = msg->data[6]; 
					qy = msg->data[5]; 
					qz = msg->data[4];  
					qw = msg->data[3];
					norm = sqrt(qx*qx + qy*qy + qz*qz + qw*qw); 				
				}
				else
				{
					goatpose_.transform.translation.x = msg->data[3] * METER_PER_INCH; //x[m]
					goatpose_.transform.translation.y = msg->data[4] * METER_PER_INCH; //y[m]
					goatpose_.transform.translation.z = msg->data[5] * METER_PER_INCH; //z[m]
					qx = msg->data[6]; 
					qy = msg->data[7]; 
					qz = msg->data[8]; 
					qw = msg->data[9];
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
				break; 

			case 1: // Case 1 = spherical position and quaternions rotation
				if(reverse_order_ == true)
				{
					goatpose_.transform.translation.x = msg->data[12] * METER_PER_INCH; //r[m]
					goatpose_.transform.translation.y = msg->data[11];				    //theta[rad]
					goatpose_.transform.translation.z = msg->data[10]; 				    //phi[rad]	
					qx = msg->data[6]; 
					qy = msg->data[5]; 
					qz = msg->data[4];  
					qw = msg->data[3];
					norm = sqrt(qx*qx + qy*qy + qz*qz + qw*qw); 				
				}
				else
				{
					goatpose_.transform.translation.x = msg->data[0] * METER_PER_INCH; //r[m]
					goatpose_.transform.translation.y = msg->data[1]; 				   //theta[rad]
					goatpose_.transform.translation.z = msg->data[2]; 				   //phi[rad]
					qx = msg->data[6]; 
					qy = msg->data[7]; 
					qz = msg->data[8]; 
					qw = msg->data[9];
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
				break; 

			case 2: // Case 2 = carthesian position and euler angles rotation
				if(reverse_order_ == true)
				{
					goatpose_.transform.translation.x = msg->data[9] * METER_PER_INCH; //x[m]
					goatpose_.transform.translation.y = msg->data[8] * METER_PER_INCH; //y[m]
					goatpose_.transform.translation.z = msg->data[7] * METER_PER_INCH; //z[m]
					goatpose_.transform.rotation.w = 0; 
					goatpose_.transform.rotation.x = msg->data[2] * RAD_PER_DEG; 	   // roll [rad] 	
					goatpose_.transform.rotation.y = msg->data[1] * RAD_PER_DEG; 	   // pitch [rad] 	
					goatpose_.transform.rotation.z = msg->data[0] * RAD_PER_DEG; 	   // yaw [rad] 		 				
				}
				else
				{
					goatpose_.transform.translation.x = msg->data[3] * METER_PER_INCH; //x[m]
					goatpose_.transform.translation.y = msg->data[4] * METER_PER_INCH; //y[m]
					goatpose_.transform.translation.z = msg->data[5] * METER_PER_INCH; //z[m]
					goatpose_.transform.rotation.w = 0; 
					goatpose_.transform.rotation.x = msg->data[10] * RAD_PER_DEG; 	   // roll [rad] 	
					goatpose_.transform.rotation.y = msg->data[11] * RAD_PER_DEG; 	   // pitch [rad] 	
					goatpose_.transform.rotation.z = msg->data[12] * RAD_PER_DEG; 	   // yaw [rad]  		
				}	
				break;

			case 3: // Case 2 = spherical position and euler angles rotation
				if(reverse_order_ == true)
				{
					goatpose_.transform.translation.x = msg->data[12] * METER_PER_INCH; //r[m]
					goatpose_.transform.translation.y = msg->data[11];				    //theta[rad]
					goatpose_.transform.translation.z = msg->data[10]; 				    //phi[rad]	
					goatpose_.transform.rotation.w = 0; 
					goatpose_.transform.rotation.x = msg->data[2] * RAD_PER_DEG; 	   // roll [rad] 	
					goatpose_.transform.rotation.y = msg->data[1] * RAD_PER_DEG; 	   // pitch [rad] 	
					goatpose_.transform.rotation.z = msg->data[0] * RAD_PER_DEG; 	   // yaw [rad] 		 				
				}
				else
				{
					goatpose_.transform.translation.x = msg->data[0] * METER_PER_INCH;  //r[m]
					goatpose_.transform.translation.y = msg->data[1];				    //theta[rad]
					goatpose_.transform.translation.z = msg->data[2]; 				    //phi[rad]	
					goatpose_.transform.rotation.w = 0; 
					goatpose_.transform.rotation.x = msg->data[10] * RAD_PER_DEG; 	   // roll [rad] 	
					goatpose_.transform.rotation.y = msg->data[11] * RAD_PER_DEG; 	   // pitch [rad] 	
					goatpose_.transform.rotation.z = msg->data[12] * RAD_PER_DEG; 	   // yaw [rad]  		
				}	
				break;

			default: 
				cout << "Invalid mode: "<< mode_ << endl; 
				break; 
		}

		// Publishing message -------------------------------------------------------------
		pub_GoatPose_.publish(goatpose_); 
	}
	else
		cout << "{Warning} \t Cannot publish goatpose as there are no more clock messages." << endl; 
}

// ******************************************************************************************************************************
/// Class: ConvertJointState 
/// Constructor
ConvertJointState::ConvertJointState(ros::NodeHandle n, vector<vector<double>> *clock_fields)
{
	n_ 				 = n; 
	clock_fields_ 	 = clock_fields; 
	nb_clock_fields_ = (*clock_fields).size();  
	pub_JointState_  = n_.advertise<sensor_msgs::JointState>("goat/jointState", 100); 
	reverse_order_ 	 = false;
	first_msg_ 		 = true;  

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

	if (~reverse_order_ && (*clock_fields_)[timeStamp_incoming_msg_.seq][0]==0 && timeStamp_incoming_msg_.seq > 0)
	{
		reverse_order_ = true;
	}
	// Header: Rewriting the original one (because it did restart at 0 when transitioning from sit-stand mode to jumping).
	// The sit-stand mode to jumping mode transition is saved in a different topic among other state variables. 
	goatjointstate_.header.seq 			= timeStamp_incoming_msg_.seq; 
	// reading from array (extracted from .csv file):
	goatjointstate_.header.stamp.sec 	= (*clock_fields_)[timeStamp_incoming_msg_.seq][1]; 
	goatjointstate_.header.stamp.nsec	= (*clock_fields_)[timeStamp_incoming_msg_.seq][2];

	// Name is simply copied (see goat-team naming motors)
	goatjointstate_.name = msg->name;

	// Changing to SI-units if not already

	goatjointstate_.position.push_back(msg->position[0]);
	goatjointstate_.position.push_back(msg->position[1]);
	goatjointstate_.position.push_back(msg->position[2]);
	// goatjointstate_.position[0] = msg->position[0] * RAD_PER_DEG; // [rad]
	// goatjointstate_.position[1] = msg->position[1] * RAD_PER_DEG; // [rad]
	// goatjointstate_.position[2] = msg->position[2] * RAD_PER_DEG; // [rad]
	if (reverse_order_)
	{
		goatjointstate_.effort.push_back(msg->effort[2]);
		goatjointstate_.effort.push_back(msg->effort[1]);
		goatjointstate_.effort.push_back(msg->effort[0]);
		goatjointstate_.velocity.push_back(msg->velocity[2]);
		goatjointstate_.velocity.push_back(msg->velocity[1]);
		goatjointstate_.position.push_back(msg->velocity[0]);

		// goatjointstate_.effort[0] 		= msg->effort[2]; 					  // [N/m]
		// goatjointstate_.effort[1] 		= msg->effort[1]; 					  // [N/m]
		// goatjointstate_.effort[2] 		= msg->effort[0]; 					  // [N/m]
		// goatjointstate_.velocity[0] 	= msg->velocity[2]; 				  // [rad/s]
		// goatjointstate_.velocity[1] 	= msg->velocity[1]; 				  // [rad/s]
		// goatjointstate_.velocity[2] 	= msg->velocity[0]; 				  // [rad/s]
	}
	else
	{
		goatjointstate_.effort.push_back(msg->effort[0]);
		goatjointstate_.effort.push_back(msg->effort[1]);
		goatjointstate_.effort.push_back(msg->effort[2]);
		goatjointstate_.velocity.push_back(msg->velocity[0]);
		goatjointstate_.velocity.push_back(msg->velocity[1]);
		goatjointstate_.position.push_back(msg->velocity[2]);
		// goatjointstate_.effort 		= msg->effort; 					  // [N/m]
		// goatjointstate_.velocity 	= msg->velocity; 				  // [rad/s]
	}
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
	// Timestamp!!!
	// State1: 0 = unknown, 1 = sit-stand, 2 = jumping 
	(jointState sequence number indicates that (restarts at 0), it was also manually copied in the simTime .csv files)
	// State2: 0 = unknown, 1 = in the air, 2 = on ground
	(already there)
	// l_landing, l_compressed, l_thrust
	// k_landing
	// Thrust 

	*/
}