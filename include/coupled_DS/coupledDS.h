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

#ifndef __COUPLEDDS_H__
#define __COUPLEDDS_H__

//---- ros and msd
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include "std_msgs/MultiArrayDimension.h"
#include <std_msgs/Float64.h>

//---- eigen and vector
#include "Eigen/Eigen"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <vector>
#include <mutex>
#include <array>

//--- lasa vector and filter
#include "MathLib.h"
#include "GMRDynamics.h"
#include "CDDynamics.h"

//----for record data
#include <sstream>		
#include <iostream>		
#include <fstream>		
#include <string>		
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

//--- some lib maybe use later
#include <signal.h>
#include <pthread.h>
#include "nav_msgs/Path.h"
#include <stdlib.h> 
#include <dynamic_reconfigure/server.h>
#include <boost/bind.hpp>
#include <math.h>

//---- func in cFunc
#include "robot_control/keyboard.h"
// #include <cFunc/keyboard.h>

//---- func for 3D limit cycle
// #include "transformations.h"
// #include <bifurcation_sample/transformations.h>
// #include "bifurcation_sample/transformations.h"

//---- func for utils
#include "Utils.h"

class coupledDS {

private:

  double dt_;

  //-----ROS--------------------------------------------------
    //----- ROS system variables
    ros::NodeHandle           nh_;
    ros::Rate                 loop_rate_;

    //------ Publishers/Subscriber
    ros::Subscriber           sub_real_pose_;
    ros::Subscriber           sub_real_velocity_;
    ros::Subscriber           sub_real_force_;
    
    ros::Publisher            pub_desired_velocity_;
    ros::Publisher            pub_desired_velocity_filtered_;
    ros::Publisher            pub_desired_damping_eig_;

    //----- Topic Names
    std::string               input_pose_name_;
    std::string               input_velocity_name_;
    std::string               input_force_name_;

    std::string               output_velocity_name_;
    std::string               output_filtered_velocity_name_;
    std::string               output_damping_eig_topic_name_;
    
    
    //----- Messages
    geometry_msgs::Pose       msg_real_pose_;
    geometry_msgs::Twist       msg_real_velocity_;
    geometry_msgs::Twist      msg_real_force_;

    geometry_msgs::Twist            msg_desired_velocity_;
    geometry_msgs::Pose            msg_desired_velocity_filtered_;
    std_msgs::Float64MultiArray     msg_desired_damping_eig_;
  //------------------------------------------------------------

  //----   Filter variables
    std::unique_ptr<CDDynamics> CCDyn_filter_;

    double Wn_;
    MathLib::Vector accLimits_;
    MathLib::Vector velLimits_;


	//----- Class variables
	  std::mutex                mutex_;

  //----- motion genetor
    int M_;

  //---- input signal 
    MathLib::Vector     end_force_;
    MathLib::Vector     end_torque_;
    MathLib::Vector     real_pose_;
    MathLib::Vector     real_velocity_;
    MathLib::Vector     real_velocity_ori_;
    MathLib::Vector     real_pose_ori_;
    MathLib::Vector     force_sum;
    MathLib::Vector     end_force_zero_stable_;
    MathLib::Vector     target_pose_;
    MathLib::Vector     target_pose_ori_;
    MathLib::Vector     target_pose_prev_;

  //---- output 
    MathLib::Vector desired_velocity_;
    MathLib::Vector desired_velocity_filtered_;
    std::vector<double> eig;
    float eig_velue[2];
  
  //---- limitation for output
    double Velocity_limit_;

  //------ record data
    bool brecord_or_not;
    bool brecord_or_not_;
    std::ofstream file_pose_;
    std::ofstream file_velocity_;
    std::ofstream file_force_;

  //------ change tool offset
    bool record_1_robotdemo_0_humandemo_;

  //------ parameter in yaml
    std::vector<double> target_;
    double Convergence_Rate_;
    double Cycle_radius_;
    double Cycle_speed_;

  //--- about tool offset
    bool _firstRealPoseReceived= false;
    double _toolOffsetFromEE;
    double _toolMass;
    Eigen::Vector3f _toolComPositionFromSensor;	

    Eigen::Matrix<float,6,1> _wrenchBias;
    Eigen::Matrix<float,6,1> _wrench;
    Eigen::Matrix<float,6,1> _filteredWrench;
    int _wrenchCount;
    float _filteredForceGain;

    Eigen::Matrix3f _wRb;				// Current rotation matrix [m] (3x1)
    Eigen::Vector3f _x;				// Current position [m] (3x1)
    Eigen::Vector4f _q;				// Current end effector quaternion (4x1)

    Eigen::Vector3f _gravity;

  //--- circle DS
    MathLib::Vector     desired_pose_;
    // MathLib::Vector     desired_velocity_des;
    double T_init;
    bool            _firstReachCirclePlane= false;
    double pos_error_;
    bool            phase_2_begin= false;
    

  //--- linear DS
    MathLib::Vector     linear_motion_target_pose_;
    MathLib::Vector     linearDS_offset;

  //--- ori control
    bool            _firstChangeOri= false;
    MathLib::Vector vel_at_begin_;
		MathLib::Vector ori_at_begin_;
		MathLib::Vector eigenvector_1,eigenvector_2,eigenvector_3; 
    MathLib::Vector desired_ori_velocity_filtered_;
    
    bool            _endChangeOri= false;
    MathLib::Vector ori_at_end_;

    int time_count_for_ori_control=0;

  //--- check_kb_input
		MathLib::Vector stop_target;
		int phase;

  //--- for 3D limit cycle
    float* vd;					// Desired velocity
    // Eigen::Vector3f x;			// Current position
    Eigen::Vector3f xd;			// Desired position
    //--- variables to smooth switching
		bool smooth = false;
		float MaxCount = 20;
		int smoothCount = 0;

    MathLib::Vector normal_vector;
    Eigen::Vector4f _qd;				// desired effector quaternion (4x1)

public:
	coupledDS(
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
              double Cycle_speed);

  ~coupledDS(void);

	bool Init();

	void Run();

private:

	bool InitializeROS();

  bool InitDSAndFilter();

  void UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg);

  void UpdateRealVel(const geometry_msgs::Twist::ConstPtr& msg_vel);

	void UpdateRealForce(const geometry_msgs::WrenchStamped& msg_real_force_);

  void check_kb_input();

	void ComputeCommand();

	void PublishCommand();

  MathLib::Vector getCircleVelocity(MathLib::Vector pose, MathLib::Vector target_pose_cricleDS, int stop_when_get_some_angle,int rotate_with_x_y_z, double Cycle_radius_offset_, double speed_scale_);

  void updateLimitCycle3DPosVel(MathLib::Vector pose, MathLib::Vector target_pose_cricleDS, MathLib::Vector desired_ori_velocity_filtered_);

  MathLib::Vector circle_ds(MathLib::Vector x);

  MathLib::Vector OriController(MathLib::Vector desired_velocity_, MathLib::Vector desired_ori_velocity_filtered_,int XYlags1_XZlags2_YZlags3);

  void computeDesiredOrientation(Eigen::Matrix3f rotMat);

};


#endif
