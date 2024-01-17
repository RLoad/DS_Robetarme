/*
 * Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Authors: Mahdi Khoramshahi and Nadia Figueroa
 * email:   {mahdi.khoramshahi,nadia.figueroafernandez}@epfl.ch
 * website: lasa.epfl.ch
 *
 * This work was supported by the European Communitys Horizon 2020 Research and Innovation 
 * programme ICT-23-2014, grant agreement 644727-Cogimon and 643950-SecondHands.
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A_1 PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "test_mini_code.h"

test_mini_code::test_mini_code(
            //--- parem in node
              ros::NodeHandle &n,
              double frequency,
            //--- parem in launch
              std::string input_pose_name,
              std::string input_vel_name,
              std::string input_force_name,
			  std::string input_joint_state_name,
              std::string output_vel_name,
              std::string output_filtered_vel_name,
              std::string output_damping_eig_topic_name,
			  std::string output_joint_position_name,
              bool brecord_or_not,
              bool record_1_robotdemo_0_humandemo,
            //--- parem in yaml
              std::vector<double> target
							):
	 	//---- trans from node.cpp
			nh_(n),
			loop_rate_(frequency),
			input_pose_name_(input_pose_name),
			input_vel_name_(input_vel_name),
			input_force_name_(input_force_name),
			input_joint_state_name_(input_joint_state_name),
			output_vel_name_(output_vel_name),
			output_filtered_vel_name_(output_filtered_vel_name),
			output_damping_eig_topic_name_(output_damping_eig_topic_name),
			output_joint_position_name_(output_joint_position_name),
			dt_(1 / frequency),
			safe_time_count(1),
			brecord_or_not_(brecord_or_not),
			record_1_robotdemo_0_humandemo_(record_1_robotdemo_0_humandemo),
			target_(target),
		//---- init some parameter, which can use dycall to online change
			eig_velue{70.0,60.0},
			pose_command_velue{0.0, 0.667, 0.0017, -1.2, 0.018, -0.4, 0.008},
			// pose_command_velue{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
			pose_change_velocity(0.1),Velocity_limit_(0.1){

	ROS_INFO_STREAM("Motion generator node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}

test_mini_code::~test_mini_code(){
    ROS_INFO_STREAM("In destructor.. motion generator was killed! ");
		if	(brecord_or_not_)
		{
			file_pose_.close();
			file_vel_.close();
			file_force_.close();
		}
}

bool test_mini_code::Init() {
	//----- init input signal parameter
		end_force_.Resize(3);
    end_torque_.Resize(3);
    real_pose_.Resize(3);
    real_vel_.Resize(3);
    real_vel_ori_.Resize(3);
    real_pose_ori_.Resize(4);
    force_sum.Resize(3);
    end_force_zero_stable_.Resize(3);

	//----- init output singal
		desired_vel_.Resize(3);
		desired_vel_filtered_.Resize(3);
		
		//------ setup for variable impedance gains
		msg_desired_damping_eig_.layout.dim.push_back(std_msgs::MultiArrayDimension());
		msg_desired_damping_eig_.layout.dim[0].label = "length";
		msg_desired_damping_eig_.layout.dim[0].size = eig.size();
		msg_desired_damping_eig_.layout.dim[0].stride = 1;

	//------ setup for variable impedance gains
		msg_desired_pose_.layout.dim.push_back(std_msgs::MultiArrayDimension());
		msg_desired_pose_.layout.dim[0].label = "length";
		msg_desired_pose_.layout.dim[0].size = pose_command.size();
		msg_desired_pose_.layout.dim[0].stride = 1;

	//------ some condition for phase change and coupled
		_firstRealPoseReceived = false;

		first_get_joint_state_received = false;

		//----- walid compensation
			if (record_1_robotdemo_0_humandemo_)
			{
				// _toolOffsetFromEE = 0.315f; //---- pen tool with f/t sensor
				_toolOffsetFromEE = 0.23f;	//---- knife tool with f/t sensor
			}else
			{
				_toolOffsetFromEE = 0.0f; //--- change this for tool offset
			}

			_toolMass=0.308f;
			_gravity << 0.0f, 0.0f, -9.80665f;
			_toolComPositionFromSensor << 0.0f,0.0f,0.06f;
			_wrenchCount=0;

			float _filteredForceGain=0.5;

	
		

  //-------------- write data file-------------------------------------------------------------------
		if	(brecord_or_not_)
		{
			std::string recPath;

			// initialize the files for saving data
			if (!nh_.getParam("recording_path", recPath))
			{
				ROS_ERROR("[demo_recorder_node] Couldn't retrieve the recording path.");
				return false;
			}
			ROS_INFO_STREAM("recording ESN path: " << recPath);

			///----- change each time 
			mkdir(recPath.c_str(), 0777);

			//----- Creating a subdirectory for a specific interaction based on time
			time_t rawtime;
			tm* timeinfo;
			char buffer[80];
			time(&rawtime);
			timeinfo = localtime(&rawtime);
			strftime(buffer, 80, "%Y-%m-%d-%H-%M-%S", timeinfo);
			recPath += std::string(buffer);
			mkdir(recPath.c_str(), 0777);
			ROS_INFO_STREAM("[demo_recorder_node] Recording to :" << recPath.c_str());

			//------- open file to record
			std::string recPath_esn_pose = recPath + "/Pose.txt";
			file_pose_.open(recPath_esn_pose);
			std::string recPath_esn_vel = recPath + "/Vel.txt";
			file_vel_.open(recPath_esn_vel);
			std::string recPath_esn_force = recPath + "/Force.txt";
			file_force_.open(recPath_esn_force);

			//----- judge if data file is open
			if(!file_pose_){
				ROS_INFO_STREAM("open failed\n" );
			}else
			{
				ROS_INFO_STREAM("open sucess\n" );
			}
		}
  //----------------------------------------------------------------------------

	if (!InitializeROS()) {
		ROS_ERROR_STREAM("ERROR intializing the ROS");
		return false;
	}

	if (!InitDSAndFilter()) {
		ROS_ERROR_STREAM("ERROR intializing the DS and filter");
		return false;
	}

	return true;
}


bool test_mini_code::InitializeROS() {
  sub_real_pose_              = nh_.subscribe( input_pose_name_ , 1000, &test_mini_code::UpdateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_real_vel_              = nh_.subscribe( input_vel_name_ , 1000, &test_mini_code::UpdateRealVel, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_real_force_             = nh_.subscribe( input_force_name_ , 1000, &test_mini_code::UpdateRealForce, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_real_joint_states_              = nh_.subscribe( input_joint_state_name_ , 1000, &test_mini_code::UpdateRealJointStates, this, ros::TransportHints().reliable().tcpNoDelay());
  
  pub_desired_vel_          	= nh_.advertise<geometry_msgs::Twist>(output_vel_name_, 1);    
	pub_desired_vel_filtered_ 	= nh_.advertise<geometry_msgs::Pose>(output_filtered_vel_name_, 1);
	pub_desired_damping_eig_         = nh_.advertise<std_msgs::Float64MultiArray>(output_damping_eig_topic_name_, 1);

	//--- position control
	pub_desired_position_          	= nh_.advertise<std_msgs::Float64MultiArray>(output_joint_position_name_, 1);    
	
	if (nh_.ok()) { // Wait for poses being published
		ros::spinOnce();
		ROS_INFO("The Motion generator is ready.");
		return true;
	}
	else {
		ROS_ERROR("The ros node has a problem.");
		return false;
	}
}

bool test_mini_code::InitDSAndFilter(){

	//--- trans target in ymal into target_pose_
		target_pose_.Resize(3);

		for (int i = 0; i < 3; i++)
		target_pose_(i) = target_[i];

	return true;
}

void test_mini_code::Run() {

  //------ run while loop
    while (nh_.ok()) {

      ComputeCommand();
      PublishCommand();

      //-----  ros spinonce deal with publish topic
      ros::spinOnce();
      loop_rate_.sleep();
    }

    nh_.shutdown();
}

void test_mini_code::UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) {

	msg_real_pose_ = *msg;

	real_pose_(0) = msg_real_pose_.position.x;
	real_pose_(1) = msg_real_pose_.position.y;
	real_pose_(2) = msg_real_pose_.position.z;

	real_pose_ori_(0) = msg_real_pose_.orientation.x;
	real_pose_ori_(1) = msg_real_pose_.orientation.y;
	real_pose_ori_(2) = msg_real_pose_.orientation.z;
	real_pose_ori_(3) = msg_real_pose_.orientation.w;

	//---- Update end effecotr pose (position+orientation)
	_x << msg_real_pose_.position.x, msg_real_pose_.position.y, msg_real_pose_.position.z;
	_q << msg_real_pose_.orientation.w, msg_real_pose_.orientation.x, msg_real_pose_.orientation.y, msg_real_pose_.orientation.z;
	_wRb = quaternionToRotationMatrix(_q);

	// std::cerr<<"_x befor: "<<_x<<"\n";
	_x = _x+_toolOffsetFromEE*_wRb.col(2);
	
	for (size_t i = 0; i < 3; i++)
	{
		real_pose_(i)=_x(i);
	}
	// std::cerr<<"_x after: "<<real_pose_<<"\n";

	if(!_firstRealPoseReceived)
	{
		_firstRealPoseReceived = true;
	// 	_xd = _x;
	// 	_qd = _q;
	// 	_vd.setConstant(0.0f);
	}
}

void test_mini_code::UpdateRealVel(const geometry_msgs::Twist::ConstPtr& msg_vel) {

	msg_real_vel_ = *msg_vel;

	real_vel_(0) = msg_real_vel_.linear.x;
	real_vel_(1) = msg_real_vel_.linear.y;
	real_vel_(2) = msg_real_vel_.linear.z;

	real_vel_ori_(0) = msg_real_vel_.angular.x;
	real_vel_ori_(1) = msg_real_vel_.angular.y;
	real_vel_ori_(2) = msg_real_vel_.angular.z;
	
}

void test_mini_code::UpdateRealForce(const geometry_msgs::WrenchStamped& msg_real_force_) {  //geometry_msgs::PoseStamped

	//------ remember change this when change the ATI 
	end_force_(0)=msg_real_force_.wrench.force.x;
	end_force_(1)=-msg_real_force_.wrench.force.y;
	end_force_(2)=-msg_real_force_.wrench.force.z;

	end_torque_(0)=msg_real_force_.wrench.torque.x;
	end_torque_(1)=msg_real_force_.wrench.torque.y;
	end_torque_(2)=msg_real_force_.wrench.torque.z;

	end_force_=end_force_-end_force_zero_stable_;
}

void test_mini_code::UpdateRealJointStates(const sensor_msgs::JointState& msg_real_joint_state_) {

	if (!first_get_joint_state_received)
	{
		for (size_t i = 0; i < 7; i++)
		{
			pose_command_velue[i]=msg_real_joint_state_.position[i];
		}

		first_get_joint_state_received = true;
	}
	
}

void test_mini_code::ComputeCommand() {

	double warn_freq=0.2;

  //---- lock mutex
	  mutex_.lock();

	//---- init some local parameter 
		MathLib::Vector Trans_pose ; Trans_pose.Resize(3);
		MathLib::Vector command_pose_task ; command_pose_task.Resize(3);
		MathLib::Vector command_pose_joint ; command_pose_joint.Resize(7);


	//------ command calculate part ------------------%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%--------------------------
		//---- aware robot state
			Trans_pose = real_pose_ -target_pose_;
			//Trans_pose = 0;

		//---- calcuate deisried velocity
			desired_vel_(0) = (-1.9) * Trans_pose[0];
			desired_vel_(1) = (-1.9) * Trans_pose[1];
			desired_vel_(2) = (-1.9) * Trans_pose[2];
			ROS_WARN_STREAM_THROTTLE(warn_freq, "real_pose_  aa: " << real_pose_(0)<<", "<< real_pose_(1)<<", "<< real_pose_(2));
			ROS_WARN_STREAM_THROTTLE(warn_freq, "Trans_pose  aa: " << Trans_pose(0)<<", "<< Trans_pose(1)<<", "<< Trans_pose(2));
			ROS_WARN_STREAM_THROTTLE(warn_freq, "desired_vel_ aaa : " << desired_vel_(0)<<", "<< desired_vel_(1)<<", "<< desired_vel_(2));

		//---- use IK get joint position 
			command_pose_task=real_pose_+desired_vel_*dt_;
			// here code IK which will be a function like: command_pose_joint=IK(command_pose_task);


		//---- calculate pose_command

			for (size_t i = 0; i < 7; i++)
			{
				// pose_command_velue[i]=command_pose_joint(i);

				//---- TEST position control
					if (safe_time_count<=1000)
					{
						pose_command_velue[i]=pose_command_velue[i]+pose_change_velocity*dt_;
						safe_time_count+=1;
					}
					else
					{
						safe_time_count=1;
						pose_change_velocity=(-1)*pose_change_velocity;
					}

			}

			pose_command.insert(pose_command.begin(),pose_command_velue,pose_command_velue+7);
			
			ROS_WARN_STREAM_THROTTLE(warn_freq, "pose_command_velue  : " << pose_command_velue[0]<<", "<< pose_command_velue[1]<<", "<< pose_command_velue[2]<<", "<< pose_command_velue[3]<<", "<< pose_command_velue[4]<<", "<< pose_command_velue[5]<<", "<< pose_command_velue[6]);
			// ROS_WARN_STREAM_THROTTLE(warn_freq, "pose_command  : " << pose_command(0)<<", "<< pose_command(1)<<", "<< pose_command(2));
		

		//---- calculate eig
			eig_velue[0]=20;
			eig_velue[1]=20;
			eig.insert(eig.begin(),eig_velue,eig_velue+2);

	//------ command calculate part end ------------------%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%--------------------------
	

	//---- velocity limitation and filter
		if (desired_vel_.Norm() > Velocity_limit_) {
			desired_vel_ = desired_vel_ / desired_vel_.Norm() * Velocity_limit_;
			}

		desired_vel_filtered_=desired_vel_;

		if (desired_vel_filtered_.Norm() > Velocity_limit_) {
			desired_vel_filtered_ = desired_vel_filtered_ / desired_vel_filtered_.Norm() * Velocity_limit_;
			}

	//------ pub output --------------------------------------------
		//------ set velocity ( can see simulatior distrub in it )
			msg_desired_vel_.linear.x  = desired_vel_(0);
			msg_desired_vel_.linear.y  = desired_vel_(1);
			msg_desired_vel_.linear.z  = desired_vel_(2);
			msg_desired_vel_.angular.x = 0;
			msg_desired_vel_.angular.y = 0;
			msg_desired_vel_.angular.z = 0.001;

			msg_desired_vel_filtered_.position.x  = desired_vel_filtered_(0);
			msg_desired_vel_filtered_.position.y  = desired_vel_filtered_(1);
			msg_desired_vel_filtered_.position.z  = desired_vel_filtered_(2);
			msg_desired_vel_filtered_.orientation.x = 0;  
			msg_desired_vel_filtered_.orientation.y = 0;
			msg_desired_vel_filtered_.orientation.z = 0;
			msg_desired_vel_filtered_.orientation.w = 0;
			
		//---------- set imp eig
			msg_desired_damping_eig_.data.insert(msg_desired_damping_eig_.data.end(), eig.begin(), eig.end());
		
		//---------- set position
			// msg_desired_pose_.data.insert(msg_desired_pose_.data.end(), pose_command.begin(), pose_command.end());
			// for (size_t i = 0; i < 7; i++)
			// {
			// 	msg_desired_pose_.data.push_back(pose_command_velue[2]);
			// }
			msg_desired_pose_.data=pose_command;


	//----- print new information
		ROS_WARN_STREAM_THROTTLE(warn_freq, "real_pose_  : " << real_pose_(0)<<", "<< real_pose_(1)<<", "<< real_pose_(2));
		ROS_WARN_STREAM_THROTTLE(warn_freq, "Trans_pose  : " << Trans_pose(0)<<", "<< Trans_pose(1)<<", "<< Trans_pose(2));
		ROS_WARN_STREAM_THROTTLE(warn_freq, "desired_vel_  : " << desired_vel_(0)<<", "<< desired_vel_(1)<<", "<< desired_vel_(2));

	//---- pub data
		if	(brecord_or_not_)
		{
			// ROS_WARN_STREAM_THROTTLE(warn_freq, "Record");
			file_pose_ << ros::Time::now() << "\t" << real_pose_(0) << "\t" << real_pose_(1)<< "\t" << real_pose_(2) 
								 << "\t" << real_pose_ori_(0) << "\t" << real_pose_ori_(1) << "\t" << real_pose_ori_(2) << "\t" << real_pose_ori_(3)
								 << "\t" << _toolOffsetFromEE
								 << "\t" <<msg_real_pose_.position.x<< "\t" <<msg_real_pose_.position.y<< "\t" <<msg_real_pose_.position.z
								 <<"\n";

			file_vel_ << ros::Time::now() << "\t" << real_vel_(0) << "\t" << real_vel_(1)<< "\t" << real_vel_(2) 
								 << "\t" << real_vel_ori_(0) << "\t" << real_vel_ori_(1) << "\t" << real_vel_ori_(2) <<  "\n";

			file_force_ << ros::Time::now() << "\t"<<  end_force_(0) << "\t" << end_force_(1)<< "\t" << end_force_(2) 
								 << "\t" << end_torque_(0) << "\t" << end_torque_(1) << "\t" << end_torque_(2)<< "\n";
		}else
		{
			// ROS_WARN_STREAM_THROTTLE(warn_freq, "Not Record");
		}

  //---- unlock mutex
	  mutex_.unlock();

}


void test_mini_code::PublishCommand() {
	//---- for damping control
		pub_desired_vel_.publish(msg_desired_vel_);
		pub_desired_vel_filtered_.publish(msg_desired_vel_filtered_);
		pub_desired_damping_eig_.publish(msg_desired_damping_eig_);
		eig.clear();
	//--- for position control
		// pub_desired_position_.publish(msg_desired_pose_);
		// pose_command.clear();
}


Eigen::Matrix3f test_mini_code::quaternionToRotationMatrix(Eigen::Vector4f q)
{
  Eigen::Matrix3f R;

	//----  unit
	double q0 = q(0);
  double q1 = q(1);
  double q2 = q(2);
  double q3 = q(3);

  R(0,0) = q0*q0+q1*q1-q2*q2-q3*q3;
  R(1,0) = 2.0f*(q1*q2+q0*q3);
  R(2,0) = 2.0f*(q1*q3-q0*q2);

  R(0,1) = 2.0f*(q1*q2-q0*q3);
  R(1,1) = q0*q0-q1*q1+q2*q2-q3*q3;
  R(2,1) = 2.0f*(q2*q3+q0*q1);

  R(0,2) = 2.0f*(q1*q3+q0*q2);
  R(1,2) = 2.0f*(q2*q3-q0*q1);
  R(2,2) = q0*q0-q1*q1-q2*q2+q3*q3;  
	//------///

  return R;
}
