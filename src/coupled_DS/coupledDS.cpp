/*
 * Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Authors: Rui Wu
 * email:   {rui.wu}@epfl.ch
 * website: lasa.epfl.ch
 *
 * This work was supported by ...
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

#include "coupledDS.h"

coupledDS::coupledDS(
            //--- parem in node
              ros::NodeHandle &n,
              double frequency,
            //--- parem in launch
              std::string input_pose_name,
              std::string input_velocity_name,
              std::string input_force_name,
              std::string output_velocity_name,
              std::string output_filtered_velocity_name,
              std::string output_damping_eig_topic_name,
              bool brecord_or_not,
              bool record_1_robotdemo_0_humandemo,
            //--- parem in yaml
              std::vector<double> target,
							double Convergence_Rate,
							double Cycle_radius,
							double Cycle_speed
							):
	 	//---- trans from node.cpp
			nh_(n),
			loop_rate_(frequency),
			input_pose_name_(input_pose_name),
			input_velocity_name_(input_velocity_name),
			input_force_name_(input_force_name),
			output_velocity_name_(output_velocity_name),
			output_filtered_velocity_name_(output_filtered_velocity_name),
			output_damping_eig_topic_name_(output_damping_eig_topic_name),
			dt_(1 / frequency),
			brecord_or_not_(brecord_or_not),
			record_1_robotdemo_0_humandemo_(record_1_robotdemo_0_humandemo),
			target_(target),
			Convergence_Rate_(Convergence_Rate),
			Cycle_radius_(Cycle_radius),
			Cycle_speed_(Cycle_speed),
		//---- init some parameter, which can use dycall to online change
			eig_velue{70.0,60.0},Velocity_limit_(0.15),
		//--- for keyboard control	
			phase(1),
		//--- for circleDS control
			T_init(0){
	this->vd = new float[3];
	ROS_INFO_STREAM("Motion generator node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}

coupledDS::~coupledDS(){
    ROS_INFO_STREAM("In destructor.. motion generator was killed! ");
		if	(brecord_or_not_)
		{
			file_pose_.close();
			file_velocity_.close();
			file_force_.close();
		}
}

bool coupledDS::Init() {
	//----- init input signal parameter
		end_force_.Resize(3);
    end_torque_.Resize(3);
    real_pose_.Resize(3);
    real_velocity_.Resize(3);
    real_velocity_ori_.Resize(3);
    real_pose_ori_.Resize(4);
    force_sum.Resize(3);
    end_force_zero_stable_.Resize(3);

	//----- init output singal
		desired_velocity_.Resize(3);
		desired_velocity_filtered_.Resize(3);
		
		//------ setup for variable impedance gains
		msg_desired_damping_eig_.layout.dim.push_back(std_msgs::MultiArrayDimension());
		msg_desired_damping_eig_.layout.dim[0].label = "length";
		msg_desired_damping_eig_.layout.dim[0].size = eig.size();
		msg_desired_damping_eig_.layout.dim[0].stride = 1;

	//------ init circle DS parameter
		desired_pose_.Resize(3);
		// desired_velocity_des.Resize(3);

	//--- init for ori control
		vel_at_begin_.Resize(3);
		ori_at_begin_.Resize(4);
		eigenvector_1.Resize(3);eigenvector_2.Resize(3);eigenvector_3.Resize(3); 
		desired_ori_velocity_filtered_.Resize(4);

		ori_at_end_.Resize(4);

	//--- init for keyboard control
		stop_target.Resize(3);

	//--- for 3D limit cycle
		normal_vector.Resize(3);

	//------ some condition for phase change and coupled
		_firstRealPoseReceived = false;
		

		//----- walid compensation
			if (record_1_robotdemo_0_humandemo_)
			{
				// _toolOffsetFromEE = 0.315f; //---- pen tool with f/t sensor
				_toolOffsetFromEE = 0.23f;	//---- knife tool with f/t sensor
			}else
			{
				_toolOffsetFromEE = 0.095f; //--- change this for tool offset
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
			file_velocity_.open(recPath_esn_vel);
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


bool coupledDS::InitializeROS() {
  sub_real_pose_              = nh_.subscribe( input_pose_name_ , 1000, &coupledDS::UpdateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_real_velocity_              = nh_.subscribe( input_velocity_name_ , 1000, &coupledDS::UpdateRealVel, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_real_force_             = nh_.subscribe( input_force_name_ , 1000, &coupledDS::UpdateRealForce, this, ros::TransportHints().reliable().tcpNoDelay());
  
  pub_desired_velocity_          	= nh_.advertise<geometry_msgs::Twist>(output_velocity_name_, 1);    
	pub_desired_velocity_filtered_ 	= nh_.advertise<geometry_msgs::Pose>(output_filtered_velocity_name_, 1);
	pub_desired_damping_eig_         = nh_.advertise<std_msgs::Float64MultiArray>(output_damping_eig_topic_name_, 1);
	
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

bool coupledDS::InitDSAndFilter(){
	//--- init param for DS 

	//--- init ori control parameter
		_firstChangeOri = false;
		_endChangeOri = false;
	

	//--- trans target in ymal into target_pose_
		target_pose_.Resize(3);
		target_pose_ori_.Resize(4);
		target_pose_prev_.Resize(3);

		for (int i = 0; i < 3; i++)
		target_pose_(i) = target_[i];
		for (int i = 3; i < 7; i++)
		target_pose_ori_(i-3) = target_[i];
		for (int i = 0; i < 3; i++)
		target_pose_prev_(i) = target_[i];

	//---- for x rotate, offset traget
		target_pose_(1)=target_pose_(1)-0.4;
		target_pose_(2)=target_pose_(2)-0.10718;

		

	//--- target for linear DS
		linearDS_offset.Resize(3);
		linearDS_offset(0)=0.0;
		linearDS_offset(1)=0.0;
		linearDS_offset(2)=0.3;
		linear_motion_target_pose_=target_pose_-linearDS_offset;

	return true;
}

void coupledDS::check_kb_input() {
	Keyboard::nonblock_2(1);
	char c = Keyboard::get_char();
  if ((&c != NULL) && (c != '\0')) {
    switch(c) {
      case 'q' :
        phase=5;
				for (int i = 0; i < 3; i++)
							{stop_target(i)=real_pose_(i);} 
							stop_target(2)=stop_target(2)-0.1;
							stop_target(1)=stop_target(1)-0.1;
				// if(!_endChangeOri)
				// {
				// 	_endChangeOri = true;
				// 	ori_at_end_ = desired_ori_velocity_filtered_;//--- inside, ori_at_begin_ is fixed
				// }
				ROS_WARN_THROTTLE(0.2, "stop code: q");
        break;
			case 'w' :
        phase=5;
				for (int i = 0; i < 3; i++)
							{stop_target(i)=real_pose_(i);} 
							stop_target(1)=stop_target(1)+0.1;
				ROS_WARN_THROTTLE(0.2, "stop code: w");
        break;
			case 'x' :
        phase=5;
				for (int i = 0; i < 3; i++)
							{stop_target(i)=real_pose_(i);} 
							stop_target(1)=stop_target(1)-0.1;
				ROS_WARN_THROTTLE(0.2, "stop code: x");
        break;
			case 'a' :
        phase=5;
				for (int i = 0; i < 3; i++)
							{stop_target(i)=real_pose_(i);} 
							stop_target(0)=stop_target(0)-0.1;
				ROS_WARN_THROTTLE(0.2, "stop code: a");
        break;
			case 'd' :
        phase=5;
				for (int i = 0; i < 3; i++)
							{stop_target(i)=real_pose_(i);} 
							stop_target(0)=stop_target(0)+0.1;
				ROS_WARN_THROTTLE(0.2, "stop code: d");
        break;
			case 's' :
        phase=5;
				for (int i = 0; i < 3; i++)
							{stop_target(i)=real_pose_(i);} 
							stop_target(2)=stop_target(2)-0.1;
				ROS_WARN_THROTTLE(0.2, "stop code: d");
        break;
			case 'r' :
        phase=0;
				ROS_WARN_THROTTLE(0.2, "restart code: r");
        break;
    }
  }
	Keyboard::nonblock_2(0);
}

void coupledDS::Run() {

  //------ run while loop
    while (nh_.ok()) {

			check_kb_input();
      ComputeCommand();
      PublishCommand();

      //-----  ros spinonce deal with publish topic
      ros::spinOnce();
      loop_rate_.sleep();
    }

		for(int i=0;i<3;i++)
    vd[i] = 0.0;

    nh_.shutdown();
}

void coupledDS::UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) {

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
	_wRb = Utils<float>::quaternionToRotationMatrix(_q);

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

		//--- trans pose to ori control
			desired_pose_ = real_pose_;
			std:cerr << desired_pose_(2) << " " <<  real_pose_(2) << std::endl;

	}
}

void coupledDS::UpdateRealVel(const geometry_msgs::Twist::ConstPtr& msg_vel) {

	msg_real_velocity_ = *msg_vel;

	real_velocity_(0) = msg_real_velocity_.linear.x;
	real_velocity_(1) = msg_real_velocity_.linear.y;
	real_velocity_(2) = msg_real_velocity_.linear.z;

	real_velocity_ori_(0) = msg_real_velocity_.angular.x;
	real_velocity_ori_(1) = msg_real_velocity_.angular.y;
	real_velocity_ori_(2) = msg_real_velocity_.angular.z;
	
}

void coupledDS::UpdateRealForce(const geometry_msgs::WrenchStamped& msg_real_force_) {  //geometry_msgs::PoseStamped

	//------ remember change this when change the ATI 
		end_force_(0)=msg_real_force_.wrench.force.x;
		end_force_(1)=-msg_real_force_.wrench.force.y;
		end_force_(2)=-msg_real_force_.wrench.force.z;

	end_torque_(0)=msg_real_force_.wrench.torque.x;
	end_torque_(1)=msg_real_force_.wrench.torque.y;
	end_torque_(2)=msg_real_force_.wrench.torque.z;

	end_force_=end_force_-end_force_zero_stable_;
}

void coupledDS::ComputeCommand() {

	double warn_freq=0.2;

  //---- lock mutex
	  mutex_.lock();

	//---- init some local parameter for DS calculation
		//--- param for circle DS
			MathLib::Vector Trans_pose ; Trans_pose.Resize(3);
			MathLib::Vector desired_velocity_des; desired_velocity_des.Resize(3);
			MathLib::Vector desired_velocity_cir; desired_velocity_cir.Resize(3);
		//--- param for linear DS
			MathLib::Vector Trans_pose_linear ; Trans_pose_linear.Resize(3);
			MathLib::Vector desired_velocity_linear_; desired_velocity_linear_.Resize(3);

	//------ command calculate part ------------------%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%--------------------------

		if (phase==1)//---- go to begin poing
		{
			//---- aware robot state
				Trans_pose = real_pose_ -target_pose_;

			//---- calcuate deisried velocity
				desired_velocity_(0) = (-1.9) * Trans_pose[0];
				desired_velocity_(1) = (-1.9) * Trans_pose[1];
				desired_velocity_(2) = (-1.9) * Trans_pose[2];

			//--- set init ori
				desired_ori_velocity_filtered_(0) = target_pose_ori_(0);
				desired_ori_velocity_filtered_(1) = target_pose_ori_(1);
				desired_ori_velocity_filtered_(2) = target_pose_ori_(2);
				desired_ori_velocity_filtered_(3) = target_pose_ori_(3);

			//--- arrive and change phase
				pos_error_ = std::sqrt((real_pose_ - target_pose_).Norm2());
				ROS_WARN_STREAM_THROTTLE(0.2, "pos_error_approach_:" << pos_error_);
				if (pos_error_ <0.005)
				{
					phase = 2;
					ROS_WARN_STREAM_THROTTLE(0.2, "go phase 2!!");
					std:cerr << "go phase 2!!" << phase << std::endl;
				}

		}else if (phase==2)//---- do a circle
		{
			int use_circle_DS_with_linearDS=2;//--- 1: linearDS+circleDS, 2, 1/4 cricleDS+circleDS
			if (use_circle_DS_with_linearDS==1)//--- use circleDS + linearDS
			{//--- find out that the circle not accurate is because damping controller is not good, use passiveDS get good result...

				//--- linear DS
					Trans_pose_linear = real_pose_ -linear_motion_target_pose_;
					desired_velocity_linear_(0) = -0.2*Trans_pose_linear(0);
					desired_velocity_linear_(1) = -0.2*Trans_pose_linear(1);
					desired_velocity_linear_(2) = -0.2*Trans_pose_linear(2);

				//--- for combine DS, the target pose need change based on linearDS
					target_pose_+=desired_velocity_linear_*dt_;

				//---- get distance with target
					Trans_pose = real_pose_ -target_pose_;
					

				//---- calcuate deisried velocity
					//--- circle DS
						int stop_when_get_some_angle=0;
						int rotate_with_x_y_z=1;//--  1 for X, 2 Y 3Z
						desired_velocity_cir = getCircleVelocity(real_pose_,target_pose_,stop_when_get_some_angle,rotate_with_x_y_z,0,1);

					//--- combine DS
						desired_velocity_=desired_velocity_linear_+desired_velocity_cir;

			}else{//--- test circleDS with another 1/4 circleDS

				
				//--- calcuate deisried velocity of main motion
						MathLib::Vector target_pose_A_ ; target_pose_A_.Resize(3);
						double c_radius_offset_A=0;;
						int stop_when_get_some_angle_A=0;
						int rotate_with_x_y_z_A=1;//--  1 for X, 2 Y 3Z
						int XYlags1_XZlags2_YZlags3=3;
						double radius_offset_from_target=1;
						if (rotate_with_x_y_z_A==1)
						{	
							//--- set circle center is orign
								target_pose_A_(0)=target_[0];
								target_pose_A_(1)=target_[1];
								target_pose_A_(2)=target_[2]*(1-radius_offset_from_target);
							//--- set circle radius
								 c_radius_offset_A=target_[2]*radius_offset_from_target;
						}else	if (rotate_with_x_y_z_A==2)
						{
							//--- set circle center is orign
								target_pose_A_(0)=target_[0];
								target_pose_A_(1)=target_[1];
								target_pose_A_(2)=target_[2]*(1-radius_offset_from_target);
							//--- set circle radius
								 c_radius_offset_A=target_[2]*radius_offset_from_target;
						}else	if (rotate_with_x_y_z_A==3)
						{
							//--- set circle center is orign
								target_pose_A_(0)=target_[0]*(1-radius_offset_from_target);
								target_pose_A_(1)=target_[1];
								target_pose_A_(2)=target_[2];
							//--- set circle radius
								 c_radius_offset_A=target_[0]*radius_offset_from_target;
						}
						double velocity_scale_A=0.05;

					//------- main motion
						desired_velocity_linear_ = getCircleVelocity(target_pose_prev_,target_pose_A_,stop_when_get_some_angle_A,rotate_with_x_y_z_A,c_radius_offset_A,velocity_scale_A);
						// desired_velocity_linear_ = desired_velocity_linear_.Zero();

					//--- for combine DS, the target pose need change based on linearDS
						target_pose_+=desired_velocity_linear_*dt_;///---- maybe this calculate also is wrong, not accurate

					//--- use target pose as init position for linear motion begin point
						target_pose_prev_=target_pose_;

					//---- get distance with target
						Trans_pose_linear = real_pose_ -target_pose_;


				//--- ori control 
				//--- ori for 1/4circleDS+circleDS
					if (time_count_for_ori_control==0)
					{
						time_count_for_ori_control+=1;
					}else
					{
						desired_ori_velocity_filtered_ = OriController(desired_velocity_linear_,desired_ori_velocity_filtered_, XYlags1_XZlags2_YZlags3);
					}
					
					
				//---- calcuate deisried velocity of periodic motion
					//--- circle DS
						//---- 2D limit cycle
							// int stop_when_get_some_angle=0;
							// int rotate_with_x_y_z=3;//--  1 for X, 2 Y 3Z
							// double velocity_scale_B=1.5;
							// desired_velocity_cir = getCircleVelocity(real_pose_,target_pose_,stop_when_get_some_angle,rotate_with_x_y_z,0,velocity_scale_B);

							// Eigen::Vector4f new_quat;
							// new_quat(0)=desired_ori_velocity_filtered_(3);
							// new_quat(1)=desired_ori_velocity_filtered_(0);
							// new_quat(2)=desired_ori_velocity_filtered_(1);
							// new_quat(3)=desired_ori_velocity_filtered_(2);
							// Eigen::Matrix3f rotMat = Utils<float>::quaternionToRotationMatrix(new_quat);

							// Eigen::Vector3f desired_velocity_cir_eigen;
							// for (size_t i = 0; i < 3; i++)
							// {
							// 	desired_velocity_cir_eigen[i]=desired_velocity_cir(i);
							// }
							// desired_velocity_cir_eigen=rotMat*desired_velocity_cir_eigen;
							// for (size_t i = 0; i < 3; i++)
							// {
							// 	desired_velocity_cir(i)=desired_velocity_cir_eigen[i];
							// }

							// desired_velocity_=desired_velocity_cir;//--- only linear velocity
						
						//---- 3D limit cycle
							updateLimitCycle3DPosVel(real_pose_,target_pose_,desired_ori_velocity_filtered_);
							
							for (size_t i = 0; i < 3; i++)
							{
								desired_velocity_(i)=vd[i];//--- because the target_pose_ is changed by desired_velocity_linear_, so no need add it here
							}
							// desired_velocity_=desired_velocity_linear_;//--- only linear velocity

			}

			//--- change direction
				if (use_circle_DS_with_linearDS==1)//--- use circleDS + linearDS
				{
					pos_error_ = std::sqrt((Trans_pose_linear).Norm2());
					ROS_WARN_STREAM_THROTTLE(0.2, "pos_error_linear_:" << pos_error_);
					if (pos_error_ <0.05)
					{
						ROS_WARN_STREAM_THROTTLE(0.2, "go back!!");
						linear_motion_target_pose_+=linearDS_offset;
						linearDS_offset=linearDS_offset*(-1);
						// ROS_WARN_STREAM_THROTTLE(0.2, "go back!!");
					}
				}else//--- test circleDS with another 1/4 circleDS
				{
					/* code */
				}
				
		}
		
		//---- calculate eig
			eig_velue[0]=500;
			eig_velue[1]=500;
			eig.insert(eig.begin(),eig_velue,eig_velue+2);

	//------ command calculate part end ------------------%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%--------------------------


	//----- stop phase
		if (phase!=5)
		{

		}
		else if (phase==5) 
		{
			MathLib::Vector Trans_pose; Trans_pose.Resize(3);

			ROS_WARN_STREAM_THROTTLE(0.2, "retract");
			ROS_WARN_STREAM_THROTTLE(0.2, "stop_target:"<< stop_target);
			Trans_pose = real_pose_ - stop_target;
			desired_velocity_(0) = -1.0*Trans_pose(0);
			desired_velocity_(1) = -1.0*Trans_pose(1);
			desired_velocity_(2) = -1.0*Trans_pose(2);

			

			// desired_ori_velocity_filtered_(0) = ori_at_end_(0);
			// desired_ori_velocity_filtered_(1) = ori_at_end_(1);
			// desired_ori_velocity_filtered_(2) = ori_at_end_(2);
			// desired_ori_velocity_filtered_(3) = ori_at_end_(3);

			desired_ori_velocity_filtered_(0) = target_pose_ori_(0);
			desired_ori_velocity_filtered_(1) = target_pose_ori_(1);
			desired_ori_velocity_filtered_(2) = target_pose_ori_(2);
			desired_ori_velocity_filtered_(3) = target_pose_ori_(3);

			if (desired_velocity_.Norm() < 0.0039)
			{
				desired_velocity_ = desired_velocity_ * 0;
				ROS_WARN_STREAM_THROTTLE(0.2, "robot stop");
			}

		}
	

	//---- velocity limitation and filter
		if (desired_velocity_.Norm() > Velocity_limit_) {
			desired_velocity_ = desired_velocity_ / desired_velocity_.Norm() * Velocity_limit_;
			}

		desired_velocity_filtered_=desired_velocity_;

		if (desired_velocity_filtered_.Norm() > Velocity_limit_) {
			desired_velocity_filtered_ = desired_velocity_filtered_ / desired_velocity_filtered_.Norm() * Velocity_limit_;
			}

	//------ pub output --------------------------------------------
		//------ set velocity ( can see simulatior distrub in it )
			msg_desired_velocity_.linear.x  = desired_velocity_(0);
			msg_desired_velocity_.linear.y  = desired_velocity_(1);
			msg_desired_velocity_.linear.z  = desired_velocity_(2);
			msg_desired_velocity_.angular.x = 0;
			msg_desired_velocity_.angular.y = 0;
			msg_desired_velocity_.angular.z = 0.001;

			msg_desired_velocity_filtered_.position.x  = desired_velocity_filtered_(0);
			msg_desired_velocity_filtered_.position.y  = desired_velocity_filtered_(1);
			msg_desired_velocity_filtered_.position.z  = desired_velocity_filtered_(2);
			//--- if use target pose
				// msg_desired_velocity_filtered_.orientation.x = target_pose_ori_(0);
				// msg_desired_velocity_filtered_.orientation.y = target_pose_ori_(1);
				// msg_desired_velocity_filtered_.orientation.z = target_pose_ori_(2);
				// msg_desired_velocity_filtered_.orientation.w = target_pose_ori_(3);
			//--- if use traject calculated pose
				msg_desired_velocity_filtered_.orientation.x = desired_ori_velocity_filtered_(0);  // in iiwa_toolkit, send this can change ori in real time
				msg_desired_velocity_filtered_.orientation.y = desired_ori_velocity_filtered_(1);
				msg_desired_velocity_filtered_.orientation.z = desired_ori_velocity_filtered_(2);
				msg_desired_velocity_filtered_.orientation.w = desired_ori_velocity_filtered_(3);
			
		//---------- set imp eig
			msg_desired_damping_eig_.data.insert(msg_desired_damping_eig_.data.end(), eig.begin(), eig.end());


	//----- print new information
		// ROS_WARN_STREAM_THROTTLE(warn_freq, "real_pose_  : " << real_pose_(0)<<", "<< real_pose_(1)<<", "<< real_pose_(2));
		// ROS_WARN_STREAM_THROTTLE(warn_freq, "Trans_pose  : " << Trans_pose(0)<<", "<< Trans_pose(1)<<", "<< Trans_pose(2));
		// ROS_WARN_STREAM_THROTTLE(warn_freq, "desired_velocity_  : " << desired_velocity_(0)<<", "<< desired_velocity_(1)<<", "<< desired_velocity_(2));

	//---- pub data
		if	(brecord_or_not_)
		{
			// ROS_WARN_STREAM_THROTTLE(warn_freq, "Record");
			file_pose_ << ros::Time::now() << "\t" << real_pose_(0) << "\t" << real_pose_(1)<< "\t" << real_pose_(2) 
								 << "\t" << real_pose_ori_(0) << "\t" << real_pose_ori_(1) << "\t" << real_pose_ori_(2) << "\t" << real_pose_ori_(3)
								 << "\t" << xd(0) << "\t" << xd(1) << "\t" << xd(2)
								 <<"\n";

			file_velocity_ << ros::Time::now() << "\t" << real_velocity_(0) << "\t" << real_velocity_(1)<< "\t" << real_velocity_(2) 
								 << "\t" << real_velocity_ori_(0) << "\t" << real_velocity_ori_(1) << "\t" << real_velocity_ori_(2) <<  "\n";

			file_force_ << ros::Time::now() << "\t"<<  end_force_(0) << "\t" << end_force_(1)<< "\t" << end_force_(2) 
								 << "\t" << end_torque_(0) << "\t" << end_torque_(1) << "\t" << end_torque_(2)<< "\n";
		}else
		{
			// ROS_WARN_STREAM_THROTTLE(warn_freq, "Not Record");
		}

  //---- unlock mutex
	  mutex_.unlock();

}


void coupledDS::PublishCommand() {
	pub_desired_velocity_.publish(msg_desired_velocity_);
	pub_desired_velocity_filtered_.publish(msg_desired_velocity_filtered_);
	pub_desired_damping_eig_.publish(msg_desired_damping_eig_);
	eig.clear();
}

MathLib::Vector coupledDS::getCircleVelocity(MathLib::Vector pose, MathLib::Vector target_pose_cricleDS, int stop_when_get_some_angle,int rotate_with_x_y_z, double Cycle_radius_offset_, double speed_scale_)
{
	MathLib::Vector velocity;
	velocity.Resize(3);

	pose = pose-target_pose_cricleDS;
	// ROS_WARN_STREAM_THROTTLE(0.2, "target_pose_2:"<< target_pose_);

	double x_vel,y_vel,z_vel,R,T,cricle_plane_error;

	// int rotate_with_x_y_z=2;//--  1 for X, 2 Y 3Z

	if (rotate_with_x_y_z==3)
	{
		x_vel = 0;
		y_vel = 0;
		z_vel = - Convergence_Rate_ * pose(2);

		R = sqrt(pose(0) * pose(0) + pose(1) * pose(1));
		T = atan2(pose(1), pose(0));

	}else if (rotate_with_x_y_z==2)
	{
		x_vel = 0;
		y_vel = - Convergence_Rate_ * pose(1);
		z_vel = 0;

		R = sqrt(pose(0) * pose(0) + pose(2) * pose(2));
		T = atan2(pose(2), pose(0));
	}else
	{
		x_vel = - Convergence_Rate_ * pose(0);
		y_vel = 0;
		z_vel = 0;

		R = sqrt(pose(1) * pose(1) + pose(2) * pose(2));
		T = atan2(pose(2), pose(1));
	}
	// ROS_WARN_STREAM_THROTTLE(0.2, "T"<<T);
	

	double Rdot = - Convergence_Rate_ * (R - Cycle_radius_ - Cycle_radius_offset_);
	double Tdot = Cycle_speed_*speed_scale_;

	if (rotate_with_x_y_z==3)
	{
		x_vel = Rdot * cos(T) - R * Tdot * sin(T);
		y_vel = Rdot * sin(T) + R * Tdot * cos(T);
		cricle_plane_error=pose(2);
	}else if (rotate_with_x_y_z==2)
	{
		x_vel = Rdot * cos(T) - R * Tdot * sin(T);
		z_vel = Rdot * sin(T) + R * Tdot * cos(T);
		cricle_plane_error=pose(1);
	}else
	{
		y_vel = Rdot * cos(T) - R * Tdot * sin(T);
		z_vel = Rdot * sin(T) + R * Tdot * cos(T);
		cricle_plane_error=pose(0);
	}

	// int stop_when_get_some_angle=1;
	double T_diff=0;
	if (cricle_plane_error<0.0001)
	{
		ROS_WARN_STREAM_THROTTLE(0.2, "get circle plane, Cycle_radius_:" <<Cycle_radius_);
		if(!_firstReachCirclePlane)
		{
			_firstReachCirclePlane = true;
			T_init=T;
			std:cerr << "T_init: " << T_init << std::endl;

		}
		
		T_diff=abs(T-T_init);
		if (stop_when_get_some_angle & phase!=5)
		{
			ROS_WARN_STREAM_THROTTLE(0.2, "T_diff"<<T_diff);
			if (T_diff>=0.8)
			{
				ROS_WARN_STREAM_THROTTLE(0.02, "finished 1/4 circle,T_diff"<<T_diff);
				x_vel = 0;
				y_vel = 0;
				z_vel = 0;
				phase=5;
				for (int i = 0; i < 3; i++)
				{stop_target(i)=real_pose_(i);} 
				// stop_target(2)=stop_target(2)-0.1;
			}
			
		}
	}
	

	velocity(0) = x_vel;
	velocity(1) = y_vel;
	velocity(2) = z_vel;

	// ROS_WARN_STREAM_THROTTLE(0.2, "velocity in circle  : " << velocity(0)<<", "<< velocity(1)<<", "<< velocity(2));
		
	return velocity;
}

void coupledDS::updateLimitCycle3DPosVel(MathLib::Vector real_pose, MathLib::Vector target_pose_cricleDS, MathLib::Vector desired_ori_velocity_filtered_) 
{
	int N = 3;
  float rho0 = 0.05;//---Current radius 
  float M = 6;//---Current  mass
  float R = 3; //R[2] = {1,1};//---Current rotational speed
  float a[3] = {1,1,1};//---Current scaling:
  float x0[3] = {0.5,0.1,-0.3};//---Current origin
  float theta0[3] = {0.0000, 0.5236, 0.0000}; //--- Current rotation around origin
	Eigen::Vector3f x;
	// Eigen::Vector3f xd;

	//--- trans real ori to rotation matrix
		Eigen::Vector4f new_quat;
		new_quat(0)=desired_ori_velocity_filtered_(3);
		new_quat(1)=desired_ori_velocity_filtered_(0);
		new_quat(2)=desired_ori_velocity_filtered_(1);
		new_quat(3)=desired_ori_velocity_filtered_(2);
		Eigen::Matrix3f rotMat = Utils<float>::quaternionToRotationMatrix(new_quat);

		// std::cerr << "Quaternion: " << desired_ori_velocity_filtered_(0) << ", " << desired_ori_velocity_filtered_(1) << ", " << desired_ori_velocity_filtered_(2) << ", " << desired_ori_velocity_filtered_(3) << std::endl;


	//--- calculate the rotation matrix by vector
		// Eigen::Matrix3f rotMat = eul2rotmat(theta0[0],theta0[1],theta0[2]);

	  	// std::cerr << "Rotation matrix: " << rotMat << std::endl; 

	//--- get real pose
		for (size_t i = 0; i < 3; i++)
		{
			x0[i]=-target_pose_cricleDS(i);
		}
		for (size_t i = 0; i < 3; i++)
		{
			x(i)=real_pose(i);
		}


  float max_v = 0.3;
  float epsilon = 0.0001;      // tolerance to avoid azimuth perturbations when "crossing" the origin

  //--- store previous desired velocity
  float vprev[N];
  for (int i=0; i<N; i++)
    vprev[i] = this->vd[i];

//   //std::cerr << "Current radius and mass: " << rho0 << ", " << M << std::endl;
//   //std::cerr << "Current rotational speed: " << R << std::endl; //R[0] << ", " << R[1] << std::endl;
//   //std::cerr << "Current scaling: " << a[0] << ", " << a[1] << ", " << a[2] << std::endl;
//   //std::cerr << "Current origin: " << x0[0] << ", " << x0[1] << ", " << x0[2] << std::endl; 
//   //std::cerr << "Current rotation around origin: " << theta0[0] << ", " << theta0[1] << ", " << theta0[2] << std::endl; 


	
  float next_r[N], sphvel[N];
  Eigen::VectorXf x_shape(N);
  float x_shape2[N];
  for (int i=0; i<N; i++)
    x_shape(i) = a[i] * (x(i) + x0[i]) + epsilon;
  x_shape = rotMat.transpose() * x_shape;

// std::cerr << "Rotation matrix transpose: " << rotMat.transpose() << std::endl; 

  for (int i=0; i<N; i++)
    x_shape2[i] = x_shape(i);
  float* r = cart2hyper(x_shape2, 1, N);

  sphvel[0] = (-sqrt(2 * M) * (r[0] - rho0));
  next_r[0] = r[0] + sphvel[0] * this->dt_;
  for(int i=1; i<N; i++) {
    // new one-dimensional R
    if(i==N-1)
      sphvel[i] = (R * exp(-pow(2 * M * (r[0] - rho0),2)));
    else
      sphvel[i] = (-sqrt(2 * M) * r[i]);
    next_r[i] = r[i] + sphvel[i] * this->dt_;
  }

  this->xd = (Eigen::Vector3f) hyper2cart(next_r, 1, N);
  this->xd = rotMat * this->xd;
  for(int i=0; i<N; i++) {
    this->xd(i) -= x0[i];
    this->xd(i) /= a[i];
    //this->vd[i] = (xd[i] - x[i]) / this->dt_;
  }

  this->vd = sph2cartvel(r,sphvel,1,N);

  if (smooth == true){
    if (smoothCount++ < MaxCount){
      for(int i=0; i<N; i++){
        this->vd[i] *= (float)(smoothCount)/MaxCount;
        this->vd[i] += (float)(MaxCount-smoothCount)/MaxCount * vprev[i];
      }
    }
    else {
      smooth = false;
      smoothCount = 0;
    }
  }

  float norm_v = sqrt(pow(this->vd[0],2) + pow(this->vd[1],2) + pow(this->vd[2],2));
  if(norm_v > max_v) {
    for(int i=0; i<N; i++){
      this->vd[i] = this->vd[i] * max_v / norm_v;
    }
  }

  // std::cerr << "Previous spherical position: " << r[0] << " " << r[1] << " " << r[2] << std::endl;
  // std::cerr << "Sent velocities: " << vd[0] << " " << vd[1] << " " << vd[2] << std::endl;
  // std::cerr << "Desired spherical position: " << next_r[0] << " " << next_r[1] << " " << next_r[2] << std::endl;

	// computeDesiredOrientation(rotMat);
	// desired_ori_velocity_filtered_(0)=_qd[1];
	// desired_ori_velocity_filtered_(1)=_qd[2];
	// desired_ori_velocity_filtered_(2)=_qd[3];
	// desired_ori_velocity_filtered_(3)=_qd[0];

}

MathLib::Vector coupledDS::circle_ds(MathLib::Vector x)
{
    MathLib::Vector theta_r_v;  // this velue in 0, is theta, 1 is r, 23 is v
		theta_r_v.Resize(5);
    theta_r_v(0) = atan2(x(1),x(0)); //theta
	
    theta_r_v(1) = sqrt(x(0)*x(0)+x(1)*x(1)); //r

    double theta_dot = 1;
    double r_dot = -1*(theta_r_v(1) - 0.035);

     theta_r_v(2) = r_dot * cos(theta_r_v(0)) - theta_r_v(1) * theta_dot * sin(theta_r_v(0));
	 
     theta_r_v(3) = r_dot * sin(theta_r_v(0)) + theta_r_v(1) * theta_dot * cos(theta_r_v(0));
		theta_r_v(4) = 0;
    
    return theta_r_v;
}

MathLib::Vector coupledDS::OriController(MathLib::Vector desired_velocity_, MathLib::Vector desired_ori_velocity_filtered_,int XYlags1_XZlags2_YZlags3)
{
	//---- init
		// int XYlags1_XZlags2_YZlags3=3; // 1 is XY lagsDS, 2 is XZ lags, 3 for YZ
		int using_angle_control=1; // 1 use, 0 not use
		MathLib::Vector new_quat;new_quat.Resize(4);

		if (using_angle_control==1)
		{
			//----- using begin cutting point ori as a base to control ori
				if(!_firstChangeOri)
				{
					_firstChangeOri = true;
					vel_at_begin_ = desired_velocity_;
					ori_at_begin_ = desired_ori_velocity_filtered_;//--- inside, ori_at_begin_ is fixed
				}
				// ori_at_begin_ = desired_ori_velocity_filtered_; //--- outside might cause unstable

			//----- calculate eigen of desired and real cut direction
				Eigen::Vector3d eigenvector_1_new_eigen;
				MathLib::Vector eigenvector_1_real;eigenvector_1_real.Resize(3);
				
				eigenvector_1=desired_velocity_ / desired_velocity_.Norm();
				eigenvector_1_real=vel_at_begin_/ vel_at_begin_.Norm();
				
				for	(int i = 0; i < 3; i++)
				{eigenvector_1_new_eigen(i)=eigenvector_1(i);}

				Eigen::Vector3d eigenvector_1_real_Eigen(3);
				for	(int i = 0; i < 3; i++)
				{eigenvector_1_real_Eigen(i)=eigenvector_1_real(i);}
				
			//----- here, we only control the angle on two plant, XY and XZ or YZ
				Eigen::Vector3d Z(0,0,1); Eigen::Vector3d X(1,0,0); Eigen::Vector3d Y(0,1,0);
				Eigen::VectorXd real_pose_ori_Eigen(4);
				Eigen::VectorXd new_quat_vec_eigen(4);

				if	(XYlags1_XZlags2_YZlags3==1)
				{
					//----- limit the real velocity and desired velocity only on X-Y plant
						eigenvector_1_real_Eigen=eigenvector_1_real_Eigen-eigenvector_1_real_Eigen.dot(Z)*Z;
						eigenvector_1_new_eigen=eigenvector_1_new_eigen-eigenvector_1_new_eigen.dot(Z)*Z;

						eigenvector_1_real_Eigen=eigenvector_1_real_Eigen/eigenvector_1_real_Eigen.norm();
						eigenvector_1_new_eigen=eigenvector_1_new_eigen/eigenvector_1_new_eigen.norm();
					
					//----- calculate the quaternion between this two vector
						double dot_of_cut_dir = eigenvector_1_real_Eigen.dot(eigenvector_1_new_eigen);
						
						if (dot_of_cut_dir<-0.99999999)
						{
							new_quat_vec_eigen=X.cross(eigenvector_1_real_Eigen);
							if (new_quat_vec_eigen.norm()<0.0000001)
								new_quat_vec_eigen=Y.cross(eigenvector_1_real_Eigen);

							new_quat_vec_eigen=new_quat_vec_eigen.normalized();
							double rad = PI;
							rad = rad * 0.5;
							double s = sin(rad);
							new_quat_vec_eigen(0) = s * new_quat_vec_eigen(0);
							new_quat_vec_eigen(1) = s * new_quat_vec_eigen(1);
							new_quat_vec_eigen(2) = s * new_quat_vec_eigen(2);
							new_quat_vec_eigen(3) = cos(rad);
							std::cerr<<"NEED action !!!!! dot<-0.999  \n";
						}
						else if (dot_of_cut_dir>0.99999999)
						{
							new_quat_vec_eigen(0) = 1;
							new_quat_vec_eigen(1) = 0;
							new_quat_vec_eigen(2) = 0;
							new_quat_vec_eigen(3) = 0;
							std::cerr<<"NEED action !!!!! dot > 0.999 \n";
						}
						else
						{
							Eigen::Vector3d cross_of_cut_dir = eigenvector_1_real_Eigen.cross(eigenvector_1_new_eigen);
							for	(int i = 0; i < 3; i++)
							{new_quat_vec_eigen(i+1)=cross_of_cut_dir(i);}
								
							new_quat_vec_eigen(0)=sqrt((eigenvector_1_real_Eigen.norm()*eigenvector_1_real_Eigen.norm()) * (eigenvector_1_new_eigen.norm() *eigenvector_1_new_eigen.norm())) + eigenvector_1_real_Eigen.dot(eigenvector_1_new_eigen);
							new_quat_vec_eigen=new_quat_vec_eigen/new_quat_vec_eigen.norm();
						} 
				}else if (XYlags1_XZlags2_YZlags3==2)
				{
					//----- limit the real velocity and desired velocity only on X-Z plant
						eigenvector_1_real_Eigen=eigenvector_1_real_Eigen-eigenvector_1_real_Eigen.dot(Y)*Y;
						eigenvector_1_new_eigen=eigenvector_1_new_eigen-eigenvector_1_new_eigen.dot(Y)*Y;

						eigenvector_1_real_Eigen=eigenvector_1_real_Eigen/eigenvector_1_real_Eigen.norm();
						eigenvector_1_new_eigen=eigenvector_1_new_eigen/eigenvector_1_new_eigen.norm();
					
					//----- calculate the quaternion between this two vector
						double dot_of_cut_dir = eigenvector_1_real_Eigen.dot(eigenvector_1_new_eigen);
						
						if (dot_of_cut_dir<-0.99999999)
						{
							new_quat_vec_eigen=X.cross(eigenvector_1_real_Eigen);
							if (new_quat_vec_eigen.norm()<0.0000001)
								new_quat_vec_eigen=Z.cross(eigenvector_1_real_Eigen);

							new_quat_vec_eigen=new_quat_vec_eigen.normalized();
							double rad = PI;
							rad = rad * 0.5;
							double s = sin(rad);
							new_quat_vec_eigen(0) = s * new_quat_vec_eigen(0);
							new_quat_vec_eigen(1) = s * new_quat_vec_eigen(1);
							new_quat_vec_eigen(2) = s * new_quat_vec_eigen(2);
							new_quat_vec_eigen(3) = cos(rad);
							std::cerr<<"NEED action !!!!! dot<-0.999  \n";
						}
						else if (dot_of_cut_dir>0.99999999)
						{
							new_quat_vec_eigen(0) = 1;
							new_quat_vec_eigen(1) = 0;
							new_quat_vec_eigen(2) = 0;
							new_quat_vec_eigen(3) = 0;
							std::cerr<<"NEED action !!!!! dot > 0.999 \n";
						}
						else
						{
							Eigen::Vector3d cross_of_cut_dir = eigenvector_1_real_Eigen.cross(eigenvector_1_new_eigen);
							for	(int i = 0; i < 3; i++)
							{new_quat_vec_eigen(i+1)=cross_of_cut_dir(i);}
								
							new_quat_vec_eigen(0)=sqrt((eigenvector_1_real_Eigen.norm()*eigenvector_1_real_Eigen.norm()) * (eigenvector_1_new_eigen.norm() *eigenvector_1_new_eigen.norm())) + eigenvector_1_real_Eigen.dot(eigenvector_1_new_eigen);
							new_quat_vec_eigen=new_quat_vec_eigen/new_quat_vec_eigen.norm();
						} 
				}else
				{
					//----- limit the real velocity and desired velocity only on X-Z plant
						eigenvector_1_real_Eigen=eigenvector_1_real_Eigen-eigenvector_1_real_Eigen.dot(X)*X;
						eigenvector_1_new_eigen=eigenvector_1_new_eigen-eigenvector_1_new_eigen.dot(X)*X;

						eigenvector_1_real_Eigen=eigenvector_1_real_Eigen/eigenvector_1_real_Eigen.norm();
						eigenvector_1_new_eigen=eigenvector_1_new_eigen/eigenvector_1_new_eigen.norm();
					
					//----- calculate the quaternion between this two vector
						double dot_of_cut_dir = eigenvector_1_real_Eigen.dot(eigenvector_1_new_eigen);
						
						if (dot_of_cut_dir<-0.99999999)
						{
							new_quat_vec_eigen=Z.cross(eigenvector_1_real_Eigen);
							if (new_quat_vec_eigen.norm()<0.0000001)
								new_quat_vec_eigen=Y.cross(eigenvector_1_real_Eigen);

							new_quat_vec_eigen=new_quat_vec_eigen.normalized();
							double rad = PI;
							rad = rad * 0.5;
							double s = sin(rad);
							new_quat_vec_eigen(0) = s * new_quat_vec_eigen(0);
							new_quat_vec_eigen(1) = s * new_quat_vec_eigen(1);
							new_quat_vec_eigen(2) = s * new_quat_vec_eigen(2);
							new_quat_vec_eigen(3) = cos(rad);
							std::cerr<<"NEED action !!!!! dot<-0.999  \n";
						}
						else if (dot_of_cut_dir>0.99999999)
						{
							new_quat_vec_eigen(0) = 1;
							new_quat_vec_eigen(1) = 0;
							new_quat_vec_eigen(2) = 0;
							new_quat_vec_eigen(3) = 0;
							std::cerr<<"NEED action !!!!! dot > 0.999 \n";
						}
						else
						{
							Eigen::Vector3d cross_of_cut_dir = eigenvector_1_real_Eigen.cross(eigenvector_1_new_eigen);
							for	(int i = 0; i < 3; i++)
							{new_quat_vec_eigen(i+1)=cross_of_cut_dir(i);}
								
							new_quat_vec_eigen(0)=sqrt((eigenvector_1_real_Eigen.norm()*eigenvector_1_real_Eigen.norm()) * (eigenvector_1_new_eigen.norm() *eigenvector_1_new_eigen.norm())) + eigenvector_1_real_Eigen.dot(eigenvector_1_new_eigen);
							new_quat_vec_eigen=new_quat_vec_eigen/new_quat_vec_eigen.norm();
						} 
				}

			//----- continue::  calculate the quaternion between this two vector
				for	(int i = 0; i < 4; i++)
				{new_quat(i)=new_quat_vec_eigen(i);}

				Eigen::MatrixXd new_quat_matrix_eigen(4,4);
				Eigen::VectorXd desired_ori_velocity_filtered_Eigen(4);

				new_quat_matrix_eigen(0,0)=new_quat(0);new_quat_matrix_eigen(0,1)=-new_quat(1);new_quat_matrix_eigen(0,2)=-new_quat(2);new_quat_matrix_eigen(0,3)=-new_quat(3);
				new_quat_matrix_eigen(1,0)=new_quat(1);new_quat_matrix_eigen(1,1)=new_quat(0);new_quat_matrix_eigen(1,2)=-new_quat(3);new_quat_matrix_eigen(1,3)=new_quat(2);
				new_quat_matrix_eigen(2,0)=new_quat(2);new_quat_matrix_eigen(2,1)=new_quat(3);new_quat_matrix_eigen(2,2)=new_quat(0);new_quat_matrix_eigen(2,3)=-new_quat(1);
				new_quat_matrix_eigen(3,0)=new_quat(3);new_quat_matrix_eigen(3,1)=-new_quat(2);new_quat_matrix_eigen(3,2)=new_quat(1);new_quat_matrix_eigen(3,3)=new_quat(0);
				
				real_pose_ori_Eigen(1) = ori_at_begin_(0);
				real_pose_ori_Eigen(2) = ori_at_begin_(1);
				real_pose_ori_Eigen(3) = ori_at_begin_(2);
				real_pose_ori_Eigen(0) = ori_at_begin_(3);
					
				desired_ori_velocity_filtered_Eigen=new_quat_matrix_eigen*real_pose_ori_Eigen;

			//----- finial velcity and angle vel for motion 
				if (desired_ori_velocity_filtered_Eigen.norm()>1)
				{
					desired_ori_velocity_filtered_Eigen=desired_ori_velocity_filtered_Eigen/desired_ori_velocity_filtered_Eigen.norm();
				}

				desired_ori_velocity_filtered_(3) = desired_ori_velocity_filtered_Eigen(0);
				desired_ori_velocity_filtered_(0) = desired_ori_velocity_filtered_Eigen(1);
				desired_ori_velocity_filtered_(1) = desired_ori_velocity_filtered_Eigen(2);
				desired_ori_velocity_filtered_(2) = desired_ori_velocity_filtered_Eigen(3);

		}else
		{}

		return desired_ori_velocity_filtered_;

	}	

void coupledDS::computeDesiredOrientation(Eigen::Matrix3f rotMat)
{
  // Compute rotation error between current orientation and plane orientation using Rodrigues' law
  Eigen::Vector3f k,temp;
  temp = -rotMat.col(2);
  k = _wRb.col(2).cross(temp);
  float c = _wRb.col(2).transpose()*temp;  
  float s = k.norm();
  k /= s;
  
  Eigen::Matrix3f K;
  K << Utils<float>::getSkewSymmetricMatrix(k);

  Eigen::Matrix3f Re;
  if(fabs(s)< FLT_EPSILON)
  {
    Re = Eigen::Matrix3f::Identity();
  }
  else
  {
    Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
  }
  
  // Convert rotation error into axis angle representation
  Eigen::Vector3f omega;
  float angle;
  Eigen::Vector4f qtemp = Utils<float>::rotationMatrixToQuaternion(Re);
  Utils<float>::quaternionToAxisAngle(qtemp,omega,angle);

  // Compute final quaternion on plane
  _qd = Utils<float>::quaternionProduct(qtemp,_q);

}



