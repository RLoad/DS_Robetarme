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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __TEST_MINI_CODE_H__
#define __TEST_MINI_CODE_H__

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
#include <sensor_msgs/JointState.h>

//---- eigen and vector
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>

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

class test_mini_code {

private:

  double dt_;

  int safe_time_count;

  //-----ROS--------------------------------------------------
    //----- ROS system variables
    ros::NodeHandle           nh_;
    ros::Rate                 loop_rate_;

    //------ Publishers/Subscriber
    ros::Subscriber           sub_real_pose_;
    ros::Subscriber           sub_real_vel_;
    ros::Subscriber           sub_real_force_;
    ros::Subscriber           sub_real_joint_states_;
    
    ros::Publisher            pub_desired_vel_;
    ros::Publisher            pub_desired_vel_filtered_;
    ros::Publisher            pub_desired_damping_eig_;
    ros::Publisher            pub_desired_position_;

    //----- Topic Names
    std::string               input_pose_name_;
    std::string               input_vel_name_;
    std::string               input_force_name_;
    std::string               input_joint_state_name_;

    std::string               output_vel_name_;
    std::string               output_filtered_vel_name_;
    std::string               output_damping_eig_topic_name_;
    std::string               output_joint_position_name_;
    
    //----- Messages
    geometry_msgs::Pose       msg_real_pose_;
    geometry_msgs::Twist       msg_real_vel_;
    geometry_msgs::Twist      msg_real_force_;
    sensor_msgs::JointState      msg_real_joint_state_;

    geometry_msgs::Twist            msg_desired_vel_;
    geometry_msgs::Pose            msg_desired_vel_filtered_;
    std_msgs::Float64MultiArray     msg_desired_damping_eig_;
    std_msgs::Float64MultiArray     msg_desired_pose_;
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
    MathLib::Vector     real_vel_;
    MathLib::Vector     real_vel_ori_;
    MathLib::Vector     real_pose_ori_;
    MathLib::Vector     force_sum;
    MathLib::Vector     end_force_zero_stable_;
    MathLib::Vector     target_pose_;

  //---- output 
    MathLib::Vector desired_vel_;
    MathLib::Vector desired_vel_filtered_;
    std::vector<double> eig;
    float eig_velue[2];
    std::vector<double> pose_command;
    float pose_command_velue[7];
    float pose_change_velocity;
  
  //---- limitation for output
    double Velocity_limit_;

  //------ record data
    bool brecord_or_not;
    bool brecord_or_not_;
    std::ofstream file_pose_;
    std::ofstream file_vel_;
    std::ofstream file_force_;

  //------ change tool offset
    bool record_1_robotdemo_0_humandemo_;

  //------ target in yaml
    std::vector<double> target_;

  //--- about tool offset
    bool _firstRealPoseReceived;
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

    bool first_get_joint_state_received;

public:
	test_mini_code(
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
              std::vector<double> target);

  ~test_mini_code(void);

	bool Init();

	void Run();

private:

	bool InitializeROS();

  bool InitDSAndFilter();

  void UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg);

  void UpdateRealVel(const geometry_msgs::Twist::ConstPtr& msg_vel);

	void UpdateRealForce(const geometry_msgs::WrenchStamped& msg_real_force_);

  void UpdateRealJointStates(const sensor_msgs::JointState& msg_real_joint_state_);


	void ComputeCommand();

	void PublishCommand();

  Eigen::Matrix3f quaternionToRotationMatrix(Eigen::Vector4f q);

};


#endif
