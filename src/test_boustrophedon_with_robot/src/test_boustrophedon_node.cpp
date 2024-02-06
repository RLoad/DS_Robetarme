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

class PathPlanning {       // The class
    public:             // Access specifier
    std::vector<double> FirstPos;
    Eigen::Vector3d pos_obj;
    Eigen::Vector4d quat_obj;
    double height_target, width_target, flow_radius, limit_cycle_radius ,sum_rad, optimum_radius;
    std::vector<double> p1,p2,p3,p4;
    geometry_msgs::PoseStamped msgP;
    std::string name_base;
    PathPlanning(ros::NodeHandle& nh) {
        // Subscribe to the PoseStamped topic
        pose_subscriber_ = nh.subscribe("/vrpn_client_node/TargetRobetarme/pose_transform", 10, &PathPlanning::CC_vrpn_obj, this);
        initialPosePub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
        nh.getParam("/optitrack_publisher/name_base_optitrack", name_base);
        transformedPolygon_ = nh.advertise<geometry_msgs::PolygonStamped>("/transformed_polygon", 1, true);
        visualizeCut(nh);
    }

    void CC_vrpn_obj(const geometry_msgs::PoseStamped::ConstPtr msg) {  // Method/function defined inside the class
        pos_obj    = {msg->pose.position.x,msg->pose.position.y,msg->pose.position.z};
        quat_obj   = {msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w};
    } 

    void visualizeCut(ros::NodeHandle& n) {
        // Retrieve parameters from ROS Parameter Server
        n.getParam("height_target", height_target);
        n.getParam("width_target", width_target);
        n.getParam("limit_cycle_radius", limit_cycle_radius);
        n.getParam("flow_radius", flow_radius);
        sum_rad = flow_radius + limit_cycle_radius;
        // Rest of the code remains the same
        int sections_height = std::round(height_target / sum_rad);
        int sections_width = std::round(width_target / sum_rad);
        optimum_radius = (height_target / sections_height + width_target / sections_width) / 2.0;

        std::cout << "Nombre de sections en hauteur : " << sections_height << std::endl;
        std::cout << "Nombre de sections en largeur : " << sections_width << std::endl;
        std::cout << "Valeur réelle de z en  : " << optimum_radius << std::endl;
        // Publish z_actual_height and z_actual_width as ROS parameters
        n.setParam("/boustrophedon_server/stripe_separation", 2*optimum_radius);

    }

    boustrophedon_msgs::PlanMowingPathGoal  ComputeGoal() {


        // for reduced target
      double x = (width_target-2*optimum_radius)/2;
      double y = (height_target-2*optimum_radius)/2;
      
      p1 = {pos_obj(0)+x, pos_obj(1)+y,pos_obj(2)};
      p2 = {pos_obj(0)-x, pos_obj(1)+y,pos_obj(2)};
      p3 = {pos_obj(0)-x, pos_obj(1)-y,pos_obj(2)};
      p4 = {pos_obj(0)+x, pos_obj(1)-y,pos_obj(2)};
      name_base = "base";


      //polygon for the server
      boustrophedon_msgs::PlanMowingPathGoal goal;

      goal.property.header.stamp = ros::Time::now();
      goal.property.header.frame_id = name_base;
      goal.property.polygon.points.resize(4);
      goal.property.polygon.points[0].x = p1[0];
      goal.property.polygon.points[0].y = p1[1];
      goal.property.polygon.points[0].z = p1[2];
      goal.property.polygon.points[1].x = p2[0];
      goal.property.polygon.points[1].y = p2[1];
      goal.property.polygon.points[1].z = p2[2];           
      goal.property.polygon.points[2].x = p3[0];
      goal.property.polygon.points[2].y = p3[1];
      goal.property.polygon.points[2].z = p3[2];
      goal.property.polygon.points[3].x = p4[0];
      goal.property.polygon.points[3].y = p4[1];
      goal.property.polygon.points[3].z = p4[2];

      goal.robot_position.pose.orientation.x = 0.0;
      goal.robot_position.pose.orientation.y = 0.0;
      goal.robot_position.pose.orientation.z = 0.0;
      goal.robot_position.pose.orientation.w = 1.0;


      //for real target
      x = (width_target)/2;
      y = (height_target)/2;
      
      p1 = {pos_obj(0)+x, pos_obj(1)+y,pos_obj(2)};
      p2 = {pos_obj(0)-x, pos_obj(1)+y,pos_obj(2)};
      p3 = {pos_obj(0)-x, pos_obj(1)-y,pos_obj(2)};
      p4 = {pos_obj(0)+x, pos_obj(1)-y,pos_obj(2)};

    return goal;
}
  void visualTarget(){

      //polygon for the visualiton on the target
      Eigen::Affine3d transformation = Eigen::Translation3d(pos_obj(0),pos_obj(1),pos_obj(2)) * Eigen::Quaterniond(quat_obj(3),quat_obj(0),quat_obj(1),quat_obj(2));
      Eigen::Vector3d point1(p1[0],p1[1],p1[2]);
      Eigen::Vector3d point2(p2[0],p2[1],p2[2]);
      Eigen::Vector3d point3(p3[0],p3[1],p3[2]);
      Eigen::Vector3d point4(p4[0],p4[1],p4[2]);
      // Transform each point
      Eigen::Vector3d transformedPoint1 = transformation * point1;
      Eigen::Vector3d transformedPoint2 = transformation * point2;
      Eigen::Vector3d transformedPoint3 = transformation * point3;
      Eigen::Vector3d transformedPoint4 = transformation * point4;

    
      geometry_msgs::PolygonStamped visualpolygonTarget;
      std::vector<Eigen::Vector3d> polygonPoints = {transformedPoint1, transformedPoint2, transformedPoint3, transformedPoint4};
      visualpolygonTarget.header.frame_id = "base";  
      visualpolygonTarget.header.stamp = ros::Time::now();

      for (const auto& point : polygonPoints) {
          geometry_msgs::Point32 msg_point;
          msg_point.x = point.x();
          msg_point.y = point.y();
          msg_point.z = point.z();
          visualpolygonTarget.polygon.points.push_back(msg_point);
      }
      transformedPolygon_.publish(visualpolygonTarget);
  }

  void publishInitialPose() {

      double delta = 0.3;
      // Create a publisher for the /initialpose topic

      // Create and fill the message
      geometry_msgs::PoseWithCovarianceStamped initialPoseMsg;
      initialPoseMsg.header.seq = 0;
      initialPoseMsg.header.stamp = ros::Time(0);
      initialPoseMsg.header.frame_id = name_base;

      initialPoseMsg.pose.pose.position.x = p2[0]- delta;
      initialPoseMsg.pose.pose.position.y = p2[1]- delta;
      initialPoseMsg.pose.pose.position.z = p2[2];

      initialPoseMsg.pose.pose.orientation.x = 0.0;
      initialPoseMsg.pose.pose.orientation.y = 0.0;
      initialPoseMsg.pose.pose.orientation.z = 0.0;
      initialPoseMsg.pose.pose.orientation.w = 1.0;

      initialPoseMsg.pose.covariance.fill(0.0);  // Fill the covariance with zeros

      initialPosePub_.publish(initialPoseMsg);
      
  }

  
  nav_msgs::Path transformPath(const nav_msgs::Path& originalPath) {
    nav_msgs::Path transformedPath;
    Eigen::Affine3d transformation = Eigen::Translation3d(pos_obj(0),pos_obj(1),pos_obj(2)) * Eigen::Quaterniond(quat_obj(3),quat_obj(0),quat_obj(1),quat_obj(2));

    transformedPath.header = originalPath.header;

    for (const auto& pose : originalPath.poses) {
        // Convert pose to Eigen types for transformation
        Eigen::Vector3d originalPosition(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        Eigen::Quaterniond originalOrientation(pose.pose.orientation.w, pose.pose.orientation.x,
                                               pose.pose.orientation.y, pose.pose.orientation.z);

        // Apply transformation
        Eigen::Vector3d transformedPosition = transformation * originalPosition;
        
        // Convert the rotation matrix to a quaternion before applying the rotation
        Eigen::Quaterniond transformedOrientation(transformation.rotation());
        transformedOrientation = transformedOrientation * originalOrientation;

        // Convert back to geometry_msgs types
        geometry_msgs::PoseStamped transformedPose;
        transformedPose.header = originalPath.header;
        transformedPose.pose.position.x = transformedPosition.x();
        transformedPose.pose.position.y = transformedPosition.y();
        transformedPose.pose.position.z = transformedPosition.z();
        transformedPose.pose.orientation.w = transformedOrientation.w();
        transformedPose.pose.orientation.x = transformedOrientation.x();
        transformedPose.pose.orientation.y = transformedOrientation.y();
        transformedPose.pose.orientation.z = transformedOrientation.z();

        // Add the transformed pose to the new path
        transformedPath.poses.push_back(transformedPose);
    }

    return transformedPath;
}


    private:
    ros::Subscriber pose_subscriber_;
    ros::Publisher initialPosePub_;
    ros::Publisher transformedPolygon_;

};



//------------- define all parameter----------------------------------------------------------------------
geometry_msgs::Quaternion headingToQuaternion(double x, double y, double z);
Eigen::Matrix3f quaternionToRotationMatrix(Eigen::Vector4f q);
void updateLimitCycle3DPosVel_with2DLC(Eigen::Vector3f pose, Eigen::Vector3f target_pose_cricleDS,Eigen::Vector4f desired_ori_, double radius);

bool got_initial_pose = false;
geometry_msgs::PoseStamped initial_pose;

geometry_msgs::Pose       msg_real_pose_;

Eigen::Vector3f real_pose_;
Eigen::Vector4f real_pose_ori_;

Eigen::Matrix3f _wRb;				// Current rotation matrix [m] (3x1)
Eigen::Vector3f _x;				// Current position [m] (3x1)
Eigen::Vector4f _q;				// Current end effector quaternion (4x1)  
double _toolOffsetFromEE= 0.25f;//---- knife tool with f/t sensor
bool _firstRealPoseReceived;

Eigen::Vector3f desired_vel_;
Eigen::Vector3f desired_vel_filtered_;


geometry_msgs::Twist msg_desired_vel_;
geometry_msgs::Pose  msg_desired_vel_filtered_;

std::size_t i_follow = 0;

int fs=300;
double dt_=1.0/fs;
Eigen::Vector3f target_pose_;
Eigen::Vector4f desired_ori_velocity_filtered_;
Eigen::Vector3f vd;

double Velocity_limit_=0.5;


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
// this function take the path comoute from server and create a linear DS
//when the eef is close the the next point it change the goal until the last point of the path
Eigen::Vector3f calculateVelocityCommand(nav_msgs::Path& path_transf, Eigen::Vector3f real_pose_,Eigen::Vector4f desired_ori_,double radius)
{
  double tol = 0.05;
  double dx,dy,dz;
  double desired_vel=0.02;
  double norm;
  double scale_vel;
  Eigen::Vector3f d_vel_;
  Eigen::Vector3f path_point;
  

  if (i_follow < path_transf.poses.size() - 1)
  {
    path_point(0)=path_transf.poses[i_follow + 1].pose.position.x;
    path_point(1)=path_transf.poses[i_follow + 1].pose.position.y;
    path_point(2)=path_transf.poses[i_follow + 1].pose.position.z;
  
    dx = path_point(0) - real_pose_(0);
    dy = path_point(1) - real_pose_(1);
    dz = path_point(2) - real_pose_(2);

    norm = sqrt(dx*dx+dy*dy+dz*dz);
    scale_vel = desired_vel/norm;

    d_vel_(0)=dx*scale_vel;
    d_vel_(1)=dy*scale_vel;
    d_vel_(2)=dz*scale_vel;

    if (i_follow!=0)
    {
      target_pose_+=d_vel_*dt_;
    }

    std::cerr<<"std::sqrt((path_point - real_pose_).norm()): "<<std::sqrt((path_point - real_pose_).norm())<< std::endl;
    if (std::sqrt((path_point - target_pose_).norm())<=tol)
    {
      i_follow+=1;
    }
    updateLimitCycle3DPosVel_with2DLC(real_pose_,target_pose_,desired_ori_, radius );

  }else
  {
    path_point(0)=path_transf.poses[i_follow].pose.position.x;
    path_point(1)=path_transf.poses[i_follow].pose.position.y;
    path_point(2)=path_transf.poses[i_follow].pose.position.z;

    dx = path_point(2) - real_pose_(0);
    dy = path_point(1) - real_pose_(1);
    dz = path_point(0) - real_pose_(2);

    norm = sqrt(dx*dx+dy*dy+dz*dz);
    scale_vel = desired_vel/norm;

    d_vel_(0)=dx*scale_vel;
    d_vel_(1)=dy*scale_vel;
    d_vel_(2)=dz*scale_vel;
    if (i_follow!=0)
    {
      target_pose_+=d_vel_*dt_;
    }
  }



  if (vd.norm() > Velocity_limit_) {
    vd = vd / vd.norm() * Velocity_limit_;
      ROS_WARN_STREAM_THROTTLE(1.5, "TOO FAST");
    }

  // std::cerr<<"i_follow: "<<i_follow << std::endl;
  // // std::cerr<<"real_pose_: "<< real_pose_(0) <<","<< real_pose_(1) <<","<< real_pose_(2) <<"," << std::endl;
  // // std::cerr<<"striping_plan: "<< path_transf.poses[i_follow + 1].pose.position.x<<","
  // //                             << path_transf.poses[i_follow + 1].pose.position.y <<","
  // //                             << path_transf.poses[i_follow + 1].pose.position.z<<"," << std::endl;
  // // std::cerr<<"vel dx: "<< dx <<","<< dy <<","<< dz <<"," << std::endl;
  // std::cerr<<"d_vel_: "<< d_vel_(0) <<","<< d_vel_(1) <<","<< d_vel_(2) <<"," << std::endl;
  // // std::cerr<<"d_vel_.norm(): "<<d_vel_.norm() << std::endl;
  // std::cerr<<"vd: "<< vd(0) <<","<< vd(1) <<","<< vd(2) <<"," << std::endl;
  // std::cerr<<"desired_ori_: "<< desired_ori_(0) <<","<< desired_ori_(1) <<","<< desired_ori_(2) <<","<< desired_ori_(3) <<"," << std::endl;
  // std::cerr<<"target_pose_: "<< target_pose_(0) <<","<< target_pose_(1) <<","<< target_pose_(2) <<"," << std::endl;

  return vd;
  //return d_vel_;
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

	//---- Update end effector pose (position+orientation)
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

    actionlib::SimpleActionClient<boustrophedon_msgs::PlanMowingPathAction> client("plan_path",true);  // server name and spin thread
    ros::Publisher polygon_pub = n.advertise<geometry_msgs::PolygonStamped>("/input_polygon", 1, true);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/result_path", 1, true);
    ros::Publisher start_pub = n.advertise<geometry_msgs::PoseStamped>("/start_pose", 1, true);
    ros::Subscriber init_pose = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, initialPoseCallback);

   // Create an instance of PoseSubscriber
    PathPlanning pathplanning(n);
    // Define the parameter name
    std::string param_name = "robot";
    // Declare a variable to store the parameter value
    std::string robot_name;
    // Try to get the parameter value
    n.getParam(param_name, robot_name);

    ros::Subscriber sub_real_pose_= 
        n.subscribe( robot_name + "/ee_info/Pose" , 1000, &UpdateRealPosition, ros::TransportHints().reliable().tcpNoDelay());
    // ros::Subscriber sub_real_pose_= 
    //     n.subscribe( "/iiwa/ee_info/Pose" , 1000, &UpdateRealPosition, ros::TransportHints().reliable().tcpNoDelay());
    // ros::Publisher pub_desired_vel_ = 
    //     n.advertise<geometry_msgs::Twist>("/passive_control/vel_quat", 1);   
    ros::Publisher pub_desired_vel_filtered_ = 
        n.advertise<geometry_msgs::Pose>("/passive_control/vel_quat", 1);

    
    ros::Rate loop_rate(fs);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    client.waitForServer();  // will wait for infinite time

    ROS_INFO("Action server started");

    boustrophedon_msgs::PlanMowingPathGoal goal;
    goal = pathplanning.ComputeGoal();

    polygon_pub.publish(goal.property);

    ROS_INFO_STREAM("Waiting for goal");
    pathplanning.publishInitialPose();
    nav_msgs::Path path;

    // extract initial_pose from optitrack center of marker
    while (ros::ok())
    {
      ros::Time start_time = ros::Time::now();

      goal.robot_position = initial_pose;

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

        std::cout << "two small Result with : " << result->plan.points.size() << std::endl;

        if (result->plan.points.size() >=5){
          convertStripingPlanToPath(result->plan, path);
          break;
        }
      }

      ros::spinOnce();
      loop_rate.sleep();
    }


    while (ros::ok())
    {
      pathplanning.visualTarget();

      ros::Time start_time = ros::Time::now();

      nav_msgs::Path path_transformed = pathplanning.transformPath(path);

      path_pub.publish(path_transformed);

      ros::Time end_time = ros::Time::now();
      ros::Duration elapsed_time = end_time - start_time;
      //ROS_INFO_STREAM("Time elapsed: " << elapsed_time.toSec() << " seconds");

      // ROS_INFO_STREAM("real_pose_: " << real_pose_ );

      if (i_follow==0)
      {
        target_pose_(0)=path_transformed.poses[0].pose.position.x;
        target_pose_(1)=path_transformed.poses[0].pose.position.y;
        target_pose_(2)=path_transformed.poses[0].pose.position.z;
        // Set values for a single ROS parameter
        std::vector<double> parameter_quat = {pathplanning.quat_obj(0),pathplanning.quat_obj(1),pathplanning.quat_obj(2),pathplanning.quat_obj(3)};
        n.setParam("/initialQuat", parameter_quat);

        Eigen::Vector4f Quat4f = pathplanning.quat_obj.cast<float>();
        
        _wRb = quaternionToRotationMatrix(Quat4f);
        Eigen::Quaterniond quaternion(pathplanning.quat_obj(3),pathplanning.quat_obj(0),pathplanning.quat_obj(1),pathplanning.quat_obj(2));
        Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();
        Eigen::Vector3d target_pose_3d(target_pose_(0), target_pose_(1), target_pose_(2));
        Eigen::Vector3d parameter_pos3f = target_pose_3d- _toolOffsetFromEE*rotationMatrix.col(2);
        // Eigen::Vector3f parameter_pos3f = target_pose_ - _toolOffsetFromEE*_wRb.col(2);
        
        std::vector<double> parameter_pos;
        parameter_pos.reserve(parameter_pos3f.size());  // Reserve space for efficiency

        for (int i = 0; i < parameter_pos3f.size(); ++i) {
            parameter_pos.push_back(static_cast<double>(parameter_pos3f[i]));
        }
        n.setParam("/initialPos", parameter_pos);
      }
      

      //--- here waiting for orinetation control
      desired_ori_velocity_filtered_(0)=pathplanning.quat_obj(0);
      desired_ori_velocity_filtered_(1)=pathplanning.quat_obj(1);
      desired_ori_velocity_filtered_(2)=pathplanning.quat_obj(2);
      desired_ori_velocity_filtered_(3)=pathplanning.quat_obj(3);
      double new_rad;
      n.getParam("new_rad", new_rad);
      double flow_radius;
      n.getParam("flow_radius", flow_radius);

      double rad_up =new_rad -flow_radius;

      desired_vel_filtered_=calculateVelocityCommand(path_transformed, real_pose_,desired_ori_velocity_filtered_, rad_up);
      //ROS_INFO_STREAM("desired_vel_filtered_: " << desired_vel_filtered_ );
      msg_desired_vel_filtered_.position.x  = desired_vel_filtered_(0);
      msg_desired_vel_filtered_.position.y  = desired_vel_filtered_(1);
      msg_desired_vel_filtered_.position.z  = desired_vel_filtered_(2);
      msg_desired_vel_filtered_.orientation.x = desired_ori_velocity_filtered_(0);
      msg_desired_vel_filtered_.orientation.y = desired_ori_velocity_filtered_(1);  
      msg_desired_vel_filtered_.orientation.z = desired_ori_velocity_filtered_(2);  
      msg_desired_vel_filtered_.orientation.w = desired_ori_velocity_filtered_(3);  
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

void updateLimitCycle3DPosVel_with2DLC(Eigen::Vector3f pose, Eigen::Vector3f target_pose_cricleDS,Eigen::Vector4f desired_ori_, double radius) 
{
	double Convergence_Rate_LC_=10;
	double Cycle_radius_LC_=radius;//0.015;
	double Cycle_speed_LC_=3.14;
	float a[2] = {1., 1.};
	float norm_a=std::sqrt(a[0]*a[0]+a[1]*a[1]);
	for (int i=0; i<2; i++)
			a[i]=a[i]/norm_a;

	Eigen::Vector3f velocity;
	Eigen::Vector3f pose_eig;

	//--- trans real ori to rotation matrix
		Eigen::Vector4f new_quat;
		new_quat(0)=desired_ori_velocity_filtered_(3);
		new_quat(1)=desired_ori_velocity_filtered_(0);
		new_quat(2)=desired_ori_velocity_filtered_(1);
		new_quat(3)=desired_ori_velocity_filtered_(2);
		Eigen::Matrix3f rotMat = quaternionToRotationMatrix(new_quat);

  // std::cerr<<"pose: "<< pose(0) <<","<< pose(1) <<","<< pose(2) <<"," << std::endl;
  // std::cerr<<"target_pose_cricleDS: "<< target_pose_cricleDS(0) <<","<< target_pose_cricleDS(1) <<","<< target_pose_cricleDS(2) <<"," << std::endl;

	pose = pose-target_pose_cricleDS;
	for (size_t i = 0; i < 3; i++)
	{
		pose_eig(i)=pose(i);
	}
	pose_eig = rotMat.transpose() * pose_eig;
	for (int i=0; i<2; i++)
			pose_eig(i) = a[i] * pose_eig(i);

	double x_vel,y_vel,z_vel,R,T,cricle_plane_error;

	x_vel = 0;
	y_vel = 0;
	z_vel = - Convergence_Rate_LC_ * pose_eig(2);

	R = sqrt(pose_eig(0) * pose_eig(0) + pose_eig(1) * pose_eig(1));
	T = atan2(pose_eig(1), pose_eig(0));

	double Rdot = - Convergence_Rate_LC_ * (R - Cycle_radius_LC_);
	double Tdot = Cycle_speed_LC_;

	x_vel = Rdot * cos(T) - R * Tdot * sin(T);
	y_vel = Rdot * sin(T) + R * Tdot * cos(T);
	cricle_plane_error=pose_eig(2);

	velocity(0) = x_vel;
	velocity(1) = y_vel;
	velocity(2) = z_vel;

	velocity=rotMat*velocity;

	for(int i=0; i<3; i++){
		vd[i] = velocity(i);
	}


}




