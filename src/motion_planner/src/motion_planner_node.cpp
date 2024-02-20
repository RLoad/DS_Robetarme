#include <ros/ros.h>
#include <cstdlib>
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
  double ros_freq = 300;
  ros::init(argc, argv, "motion_planner");
  ros::NodeHandle n;

  ros::Rate loop_rate(ros_freq);

  TargetExtraction targetextraction(n);
  std::vector<Eigen::Vector3d> polygons_positions = targetextraction.get_polygons();
  Eigen::Quaterniond quatTarget = targetextraction.get_quat_target();
  Eigen::Vector3d posTarget = targetextraction.get_pos_target();
  targetextraction.see_target();

  // get_polygons_optimzed()
  // get corner_polygons()
  
  PathPlanner pathplanner(n, quatTarget, posTarget,polygons_positions);
  DynamicalSystem dynamicalsystem(n, ros_freq);
  BoustrophedonServer boustrophedonserver(n,pathplanner.optimum_radius);
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to startnew_rad
  boustrophedonserver.client.waitForServer();  // will wait for infinite time

  ROS_INFO("Action server started");

  boustrophedon_msgs::PlanMowingPathGoal goal;
  goal = pathplanner.ComputeGoal();
  boustrophedonserver.polygon_pub.publish(goal.property);

  ROS_INFO_STREAM("Waiting for goal");
  pathplanner.publishInitialPose();

  nav_msgs::Path path;
  nav_msgs::Path path_transformed;
  n.setParam("/startDS", false);
  n.setParam("/finishDS", false);

  // set info for DS (shoul be inside the constructor)
  dynamicalsystem.set_linear_speed(0.04);
  dynamicalsystem.set_limitCycle_speed_conv(2*3.14, 10);
  dynamicalsystem.set_limitCycle_radius(pathplanner.optimum_radius);

  while (ros::ok())
  {
    ros::Time start_time = ros::Time::now();

    goal.robot_position = pathplanner.initialPose;
    boustrophedonserver.start_pub.publish(goal.robot_position);
    boustrophedonserver.client.sendGoal(goal);
    ROS_INFO_STREAM("Sending goal");

    // wait for the action to return
    bool finished_before_timeout = boustrophedonserver.client.waitForResult(ros::Duration(30.0));
    actionlib::SimpleClientGoalState state = boustrophedonserver.client.getState();
    boustrophedon_msgs::PlanMowingPathResultConstPtr result = boustrophedonserver.client.getResult();
    if (result->plan.points.size() < 1)
    {
      ROS_INFO("Action did not finish before the time out.");
    }
    else{
      ROS_INFO("Action finished: %s", state.toString().c_str());

      std::cout << "Result with : " << result->plan.points.size() << std::endl;

      if (result->plan.points.size() >2){
        pathplanner.convertStripingPlanToPath(result->plan, path);
        path_transformed = pathplanner.get_transformed_path(path);
        dynamicalsystem.set_path(path_transformed);
        boustrophedonserver.path_pub.publish(path_transformed);
        boustrophedonserver.closeRosLaunch();
        break;
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  // set Ros param to different node start/Stop
  n.setParam("/startDS", true);
  bool startController;
  n.getParam("/startController", startController);
  pathplanner.set_strategique_position(n);


  // waiting for controller
  while(!startController && ros::ok()){
    n.getParam("/startController", startController);
  }

  while (ros::ok())
  {
    ros::Time start_time = ros::Time::now();
    ros::Time end_time = ros::Time::now();
    ros::Duration elapsed_time = end_time - start_time;

    dynamicalsystem.get_DS_vel();      
    dynamicalsystem.publish_ros_msg_vel();    

    if (dynamicalsystem.finish == true){
      n.setParam("/finishDS", true);
      break;
    }

    ros::spinOnce();
    loop_rate.sleep();

  } 
  n.setParam("/startDS", false);

  return 0;
}