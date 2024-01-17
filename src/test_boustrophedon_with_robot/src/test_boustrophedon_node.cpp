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

// #include "test_mini_code.h"

//------------- define all parameter----------------------------------------------------------------------
geometry_msgs::Quaternion headingToQuaternion(double x, double y, double z);
Eigen::Matrix3f quaternionToRotationMatrix(Eigen::Vector4f q);

bool got_initial_pose = false;
geometry_msgs::PoseStamped initial_pose;

geometry_msgs::Pose       msg_real_pose_;

Eigen::Vector3f real_pose_;
Eigen::Vector4f real_pose_ori_;

Eigen::Matrix3f _wRb;				// Current rotation matrix [m] (3x1)
Eigen::Vector3f _x;				// Current position [m] (3x1)
Eigen::Vector4f _q;				// Current end effector quaternion (4x1)  
double _toolOffsetFromEE= 0.23f;//---- knife tool with f/t sensor
bool _firstRealPoseReceived;

Eigen::Vector3f desired_vel_;
Eigen::Vector3f desired_vel_filtered_;


geometry_msgs::Twist msg_desired_vel_;
geometry_msgs::Pose  msg_desired_vel_filtered_;

std::size_t i_follow = 0;


//----------------define all function-------------------------------------
// server has a service to convert StripingPlan to Path, but all it does it call this method
bool convertStripingPlanToPath(const boustrophedon_msgs::StripingPlan& striping_plan, nav_msgs::Path& path)
{
  path.header.frame_id = striping_plan.header.frame_id;
  path.header.stamp = striping_plan.header.stamp;

  path.poses.clear();

  // path.poses.resize(striping_plan.points.size());
  // std::transform(striping_plan.points.begin(), striping_plan.points.end(), path.poses.begin(),
  //                [&](const boustrophedon_msgs::StripingPoint& point) {
  //                  geometry_msgs::PoseStamped pose;
  //                  pose.header.frame_id = striping_plan.header.frame_id;
  //                  pose.header.stamp = striping_plan.header.stamp;
  //                  pose.pose.position = point.point;
  //                  pose.pose.orientation.x = 0.0;
  //                  pose.pose.orientation.y = 0.0;
  //                  pose.pose.orientation.z = 0.0;
  //                  pose.pose.orientation.w = 1.0;
  //                  return pose;
  //                });

  for (std::size_t i = 0; i < striping_plan.points.size(); i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = striping_plan.header.frame_id;
    pose.header.stamp = striping_plan.header.stamp;
    pose.pose.position = striping_plan.points[i].point;

    if (i < striping_plan.points.size() - 1)
    {
      double dx = striping_plan.points[i + 1].point.x - striping_plan.points[i].point.x;
      double dy = striping_plan.points[i + 1].point.y - striping_plan.points[i].point.y;
      double dz = striping_plan.points[i + 1].point.z - striping_plan.points[i].point.z;

      pose.pose.orientation = headingToQuaternion(dx, dy, dz);
    }
    else
    {
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
    }

    path.poses.push_back(pose);
  }

  return true;
}

Eigen::Vector3f calaulteVelocityCommand(const boustrophedon_msgs::StripingPlan& striping_plan, Eigen::Vector3f real_pose_)
{
  double dx,dy,dz;
  double scale_vel=1.2;
  Eigen::Vector3f d_vel_;
  Eigen::Vector3f path_point;

  

  

  if (i_follow < striping_plan.points.size() - 1)
  {
    path_point(0)=striping_plan.points[i_follow + 1].point.x;
    path_point(1)=striping_plan.points[i_follow + 1].point.y;
    path_point(2)=0.9;

    // Eigen::Vector4f target_ori=1;
    // Eigen::Matrix3f pathRotMat=ori2rotmat;
    // path_point=pathRotMat*path_point;

    dx = path_point(2) - real_pose_(0);
    dy = path_point(1) - real_pose_(1);
    dz = path_point(0) - real_pose_(2);

    d_vel_(0)=dx*scale_vel;
    d_vel_(1)=dy*scale_vel;
    d_vel_(2)=dz*scale_vel;

    if (d_vel_.norm()<=0.04)
    {
      i_follow+=1;
    }

  }else
  {
    path_point(0)=striping_plan.points[i_follow].point.x;
    path_point(1)=striping_plan.points[i_follow].point.y;
    path_point(2)=0.9;

    dx = path_point(2) - real_pose_(0);
    dy = path_point(1) - real_pose_(1);
    dz = path_point(0) - real_pose_(2);

    d_vel_(0)=dx;
    d_vel_(1)=dy;
    d_vel_(2)=dz;
  }

  std::cerr<<"i_follow: "<<i_follow << std::endl;
  std::cerr<<"real_pose_: "<< real_pose_(0) <<","<< real_pose_(1) <<","<< real_pose_(2) <<"," << std::endl;
  std::cerr<<"striping_plan: "<< striping_plan.points[i_follow + 1].point.x <<","
                              << striping_plan.points[i_follow + 1].point.y <<","
                              << striping_plan.points[i_follow + 1].point.z+0.5 <<"," << std::endl;
  std::cerr<<"vel dx: "<< dx <<","<< dy <<","<< dz <<"," << std::endl;
  std::cerr<<"d_vel_: "<< d_vel_(0) <<","<< d_vel_(1) <<","<< d_vel_(2) <<"," << std::endl;
  std::cerr<<"d_vel_.norm(): "<<d_vel_.norm() << std::endl;

  return d_vel_;
}


void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr init_pose)
{
  initial_pose.header = init_pose->header;
  initial_pose.pose = init_pose->pose.pose;
  got_initial_pose = true;
}

void UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) {

  got_initial_pose = true;

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

//----------------------- main loop ------------------------------------------------
int main(int argc, char** argv) 
{
    ros::init(argc, argv, "test_boustrophedon_node");
    ros::NodeHandle n;

    actionlib::SimpleActionClient<boustrophedon_msgs::PlanMowingPathAction> client("plan_path",
                                                                                 true);  // server name and spin thread

    ros::Publisher polygon_pub = n.advertise<geometry_msgs::PolygonStamped>("/input_polygon", 1, true);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/result_path", 1, true);
    ros::Publisher start_pub = n.advertise<geometry_msgs::PoseStamped>("/start_pose", 1, true);
    ros::Subscriber init_pose =
        n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, initialPoseCallback);
    
    ros::Subscriber sub_real_pose_= 
        n.subscribe( "/iiwa/ee_info/Pose" , 1000, &UpdateRealPosition, ros::TransportHints().reliable().tcpNoDelay());
    // ros::Publisher pub_desired_vel_ = 
    //     n.advertise<geometry_msgs::Twist>("/passive_control/vel_quat", 1);   
    ros::Publisher pub_desired_vel_filtered_ = 
        n.advertise<geometry_msgs::Pose>("/passive_control/vel_quat", 1);

    ros::Rate loop_rate(10);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    client.waitForServer();  // will wait for infinite time

    ROS_INFO("Action server started");

    boustrophedon_msgs::PlanMowingPathGoal goal;

    goal.property.header.stamp = ros::Time::now();
    goal.property.header.frame_id = "map";

    double target_length=0.2;

    goal.property.polygon.points.resize(4);
    goal.property.polygon.points[0].x = 0+0.5;
    goal.property.polygon.points[0].y = 0+0.0;
    goal.property.polygon.points[1].x = 0+0.5;
    goal.property.polygon.points[1].y = target_length+0.0;
    goal.property.polygon.points[2].x = target_length+0.5;
    goal.property.polygon.points[2].y = target_length+0.0;
    goal.property.polygon.points[3].x = target_length+0.5;
    goal.property.polygon.points[3].y = 0+0.0;

    goal.property.polygon.points[0].z = 0.1;
    goal.property.polygon.points[1].z = 0.1;
    goal.property.polygon.points[2].z = 0.1;
    goal.property.polygon.points[3].z = 0.1;

    // goal.property.polygon.points[0].x = 0.5;
    // goal.property.polygon.points[0].y = 0;
    // goal.property.polygon.points[0].z = 0;
    // goal.property.polygon.points[1].x = 0.5;
    // goal.property.polygon.points[1].y = 0;
    // goal.property.polygon.points[1].z = 0.5;
    // goal.property.polygon.points[2].x = 0.5;
    // goal.property.polygon.points[2].y = 0.5;
    // goal.property.polygon.points[2].z = 0.5;
    // goal.property.polygon.points[3].x = 0.5;
    // goal.property.polygon.points[3].y = 0.5;
    // goal.property.polygon.points[3].z = 0;

    goal.robot_position.pose.orientation.w = 1.0;

    polygon_pub.publish(goal.property);

    ROS_INFO_STREAM("Waiting for goal");

    while (ros::ok())
    {
        if (got_initial_pose)
        {
            ros::Time start_time = ros::Time::now();

            goal.robot_position = initial_pose;
            // goal.robot_position.header = pose.header;
            // goal.robot_position.pose.position.x = real_pose_(0);
            // goal.robot_position.pose.position.y = real_pose_(1);
            // goal.robot_position.pose.position.z = real_pose_(2);
            // goal.robot_position.pose.orientation.x = real_pose_ori_(0);
            // goal.robot_position.pose.orientation.y = real_pose_ori_(1);
            // goal.robot_position.pose.orientation.z = real_pose_ori_(2);
            // goal.robot_position.pose.orientation.w = real_pose_ori_(3);

            start_pub.publish(goal.robot_position);

            client.sendGoal(goal);
            ROS_INFO_STREAM("Sending goal");

            // wait for the action to return
            bool finished_before_timeout = client.waitForResult(ros::Duration(30.0));

            if (!finished_before_timeout)
            {
                ROS_INFO("Action did not finish before the time out.");
                continue;
            }

            actionlib::SimpleClientGoalState state = client.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
            boustrophedon_msgs::PlanMowingPathResultConstPtr result = client.getResult();
            std::cout << "Result with : " << result->plan.points.size() << std::endl;

            nav_msgs::Path path;
            convertStripingPlanToPath(result->plan, path);

            path_pub.publish(path);

            got_initial_pose = false;

            ros::Time end_time = ros::Time::now();
            ros::Duration elapsed_time = end_time - start_time;
            ROS_INFO_STREAM("Time elapsed: " << elapsed_time.toSec() << " seconds");

            // ROS_INFO_STREAM("real_pose_: " << real_pose_ );
            
            desired_vel_filtered_=calaulteVelocityCommand(result->plan, real_pose_);

            ROS_INFO_STREAM("desired_vel_filtered_: " << desired_vel_filtered_ );

        }

      

        // msg_desired_vel_.linear.x  = desired_vel_(0);
        // msg_desired_vel_.linear.y  = desired_vel_(1);
        // msg_desired_vel_.linear.z  = desired_vel_(2);
        // msg_desired_vel_.angular.x = 0;
        // msg_desired_vel_.angular.y = 0;
        // msg_desired_vel_.angular.z = 0.001;
        msg_desired_vel_filtered_.position.x  = desired_vel_filtered_(0);
        msg_desired_vel_filtered_.position.y  = desired_vel_filtered_(1);
        msg_desired_vel_filtered_.position.z  = desired_vel_filtered_(2);
        msg_desired_vel_filtered_.orientation.x = 0;  
        msg_desired_vel_filtered_.orientation.y = 0;
        msg_desired_vel_filtered_.orientation.z = 0;
        msg_desired_vel_filtered_.orientation.w = 0;


        pub_desired_vel_filtered_.publish(msg_desired_vel_filtered_);
        

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

geometry_msgs::Quaternion headingToQuaternion(double x, double y, double z)
{
  // get orientation from heading vector
  const tf2::Vector3 heading_vector(x, y, z);
  const tf2::Vector3 origin(1, 0, 0);

  const auto w = (origin.length() * heading_vector.length()) + tf2::tf2Dot(origin, heading_vector);
  const tf2::Vector3 a = tf2::tf2Cross(origin, heading_vector);
  tf2::Quaternion q(a.x(), a.y(), a.z(), w);
  q.normalize();

  if (!std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z()) || !std::isfinite(q.w()))
  {
    q.setX(0);
    q.setY(0);
    q.setZ(0);
    q.setW(1);
  }

  return tf2::toMsg(q);
}


Eigen::Matrix3f quaternionToRotationMatrix(Eigen::Vector4f q)
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

