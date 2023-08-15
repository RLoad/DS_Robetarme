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

#include "ros/ros.h"
#include "coupledDS.h"


#include <vector>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "coupledDS_node");

  ros::NodeHandle nh;
  double frequency = 300.0;

  //--- Parameters in launch
    std::string input_pose_name;
    std::string input_velocity_name;
    std::string input_force_name;
    std::string output_velocity_name;
    std::string output_filtered_velocity_name;
    std::string output_damping_eig_topic_name;
    bool brecord_or_not(false);
    bool record_1_robotdemo_0_humandemo(false);

  //---- parameters in yaml
    std::vector<double> target;
    double Convergence_Rate;
    double Cycle_radius;  
    double Cycle_speed;  

  //---- input output and paremeter in launch
    if (!nh.getParam("input_pose_name", input_pose_name))   {
      ROS_ERROR("Couldn't retrieve the topic name for the input. ");
      // return -1;
    }

    if (!nh.getParam("input_velocity_name", input_velocity_name))   {
      ROS_ERROR("Couldn't retrieve the topic name for the input. ");
      // return -1;
    }

    if (!nh.getParam("input_force_name", input_force_name))   {
      ROS_ERROR("Couldn't retrieve the topic name for the input force. ");
      // return -1;
    }

    if (!nh.getParam("output_velocity_name", output_velocity_name))   {
      ROS_ERROR("Couldn't retrieve the vel name for the orign output. ");
      // return -1;
    }

    if (!nh.getParam("output_filtered_velocity_name", output_filtered_velocity_name))   {
      ROS_ERROR("Couldn't retrieve the vel name for the filtered output. ");
      // return -1;
    }

    if (!nh.getParam("output_damping_eig_topic_name", output_damping_eig_topic_name))   {
      ROS_ERROR("Couldn't retrieve the damping eig name for the output. ");
      // return -1;
    }

    if (!nh.getParam("record_or_not", brecord_or_not))   {
      ROS_ERROR("Couldn't retrieve the record or not command. ");
      // return -1;
    }

    if (!nh.getParam("record_1_robotdemo_0_humandemo", record_1_robotdemo_0_humandemo))   {
      ROS_ERROR("Couldn't retrieve the record_1_robotdemo_0_humandemo. ");
      // return -1;
    }

  //---- parameter in yaml
    if (!nh.getParam("target", target))   {
      ROS_ERROR("Couldn't retrieve the target. ");
      // return -1;
    }
    if (!nh.getParam("Convergence_Rate", Convergence_Rate))   {
      ROS_ERROR("Couldn't retrieve the Convergence_Rate. ");
      // return -1;
    }
    if (!nh.getParam("Cycle_radius", Cycle_radius))   {
      ROS_ERROR("Couldn't retrieve the Cycle_radius. ");
      // return -1;
    }
    if (!nh.getParam("Cycle_speed", Cycle_speed))   {
      ROS_ERROR("Couldn't retrieve the Cycle_speed. ");
      // return -1;
    }
 

  coupledDS coupledDS(
                      //--- parem in node
                        nh, frequency,
                      //--- param in lanuch
                        input_pose_name,
                        input_velocity_name,
                        input_force_name,
                        output_velocity_name,
                        output_filtered_velocity_name,
                        output_damping_eig_topic_name,
                        brecord_or_not,
                        record_1_robotdemo_0_humandemo,
                      //--- param in yaml
                        target,
                        Convergence_Rate,
                        Cycle_radius,
                        Cycle_speed);
  if (!coupledDS.Init()) {
    return -1;
  }
  else {
    coupledDS.Run();
  }

  return 0;
}
