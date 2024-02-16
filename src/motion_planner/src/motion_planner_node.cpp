#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boustrophedon_msgs/PlanMowingPathAction.h>
#include <boustrophedon_msgs/ConvertPlanToPath.h>

#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
//---- eigen and vector
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <cmath>
#include "library_planner.h"




//----------------------- main loop ------------------------------------------------
int main(int argc, char** argv) 
{
  ros::init(argc, argv, "motion_planner");
  ros::NodeHandle n;

  actionlib::SimpleActionClient<boustrophedon_msgs::PlanMowingPathAction> client("plan_path",true);  // server name and spin thread
  ros::Publisher polygon_pub = n.advertise<geometry_msgs::PolygonStamped>("/input_polygon", 1, true);
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/result_path", 1, true);
  ros::Publisher start_pub = n.advertise<geometry_msgs::PoseStamped>("/start_pose", 1, true);

  // Create an instance of PoseSubscriber

  TargetExtraction targetextraction(n);
  std::vector<Eigen::Vector3d> polygons_positions = targetextraction.get_polygons();
  Eigen::Quaterniond quatTarget = targetextraction.get_quat_target();
  Eigen::Vector3d posTarget = targetextraction.get_pos_target();
  
  PathPlanner pathplanner(n, quatTarget, posTarget,polygons_positions);

  //frequency
  DynamicalSystem limitcycle(n);
  // Define the parameter name
  std::string param_name = "robot";
  // Declare a variable to store the parameter value
  std::string robot_name;
  // Try to get the parameter value
  n.getParam(param_name, robot_name);

  ros::Publisher pub_desired_vel_filtered_ = 
      n.advertise<geometry_msgs::Pose>("/passive_control/vel_quat", 1);

  ros::Rate loop_rate(limitcycle.fs);



  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to startnew_rad
  client.waitForServer();  // will wait for infinite time

  ROS_INFO("Action server started");

  boustrophedon_msgs::PlanMowingPathGoal goal;
  // goal = pathplanning.ComputeGoal();
  goal = pathplanner.ComputeGoal();

  polygon_pub.publish(goal.property);

  ROS_INFO_STREAM("Waiting for goal");
  // pathplanning.publishInitialPose();
  pathplanner.publishInitialPose();

  nav_msgs::Path path;
  nav_msgs::Path path_transformed;
  n.setParam("/startDS", false);
  n.setParam("/finishDS", false);
  // extract initial_pose from optitrack center of marker
  while (ros::ok())
  {
    ros::Time start_time = ros::Time::now();

    goal.robot_position = limitcycle.initial_pose;

    start_pub.publish(goal.robot_position);

    client.sendGoal(goal);
    ROS_INFO_STREAM("Sending goal");

    // wait for the action to return
    bool finished_before_timeout = client.waitForResult(ros::Duration(30.0));
    actionlib::SimpleClientGoalState state = client.getState();
    boustrophedon_msgs::PlanMowingPathResultConstPtr result = client.getResult();
    if (result->plan.points.size() < 1)
    {
      ROS_INFO("Action did not finish before the time out.");
    }
    else{
      ROS_INFO("Action finished: %s", state.toString().c_str());

      std::cout << "Result with : " << result->plan.points.size() << std::endl;

      if (result->plan.points.size() >=5){
        pathplanner.convertStripingPlanToPath(result->plan, path);
        path_transformed = pathplanner.get_transformed_path(path);
        break;
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  n.setParam("/startDS", true);


  bool startController;
  n.getParam("/startController", startController);

  while (ros::ok())
  {

    ros::Time start_time = ros::Time::now();


    path_pub.publish(path_transformed);

    ros::Time end_time = ros::Time::now();
    ros::Duration elapsed_time = end_time - start_time;

    if (startController == false)
    {
      pathplanner.set_strategique_position(n);
      //taking the first point of the path
      limitcycle.centerLimitCycle(0)=path_transformed.poses[0].pose.position.x;
      limitcycle.centerLimitCycle(1)=path_transformed.poses[0].pose.position.y;
      limitcycle.centerLimitCycle(2)=path_transformed.poses[0].pose.position.z;
      
            //--- here waiting for orinetation control
      limitcycle.desired_ori_velocity_filtered_(0)=pathplanner.targetQuat.x();
      limitcycle.desired_ori_velocity_filtered_(1)=pathplanner.targetQuat.y();
      limitcycle.desired_ori_velocity_filtered_(2)=pathplanner.targetQuat.z();
      limitcycle.desired_ori_velocity_filtered_(3)=pathplanner.targetQuat.w();

      n.getParam("/startController", startController);

    }
    else{
      limitcycle.desired_vel_filtered_=limitcycle.calculateVelocityCommand(path_transforme, pathplanner.sum_rad);
      //ROS_INFO_STREAM("desired_vel_filtered_: " << desired_vel_filtered_ );
      limitcycle.msg_desired_vel_filtered_.position.x  = limitcycle.desired_vel_filtered_(0);
      limitcycle.msg_desired_vel_filtered_.position.y  = limitcycle.desired_vel_filtered_(1);
      limitcycle.msg_desired_vel_filtered_.position.z  = limitcycle.desired_vel_filtered_(2);
      limitcycle.msg_desired_vel_filtered_.orientation.x = limitcycle.desired_ori_velocity_filtered_(0);
      limitcycle.msg_desired_vel_filtered_.orientation.y = limitcycle.desired_ori_velocity_filtered_(1);  
      limitcycle.msg_desired_vel_filtered_.orientation.z = limitcycle.desired_ori_velocity_filtered_(2);  
      limitcycle.msg_desired_vel_filtered_.orientation.w = limitcycle.desired_ori_velocity_filtered_(3);  
      pub_desired_vel_filtered_.publish(limitcycle.msg_desired_vel_filtered_);
    }
    if (limitcycle.finish == true){
      n.setParam("/finishDS", true);
      break;
    }

    ros::spinOnce();
    loop_rate.sleep();

  } 
  n.setParam("/startDS", false);

  return 0;
}





