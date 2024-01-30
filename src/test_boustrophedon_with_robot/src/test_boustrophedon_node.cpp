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
class Optitrack {       // The class
    public:             // Access specifier
    Eigen::Vector3d pos_obj;
    Eigen::Vector4d quat_obj;
    std::vector<double> p1,p2,p3,p4;
    geometry_msgs::PoseStamped msgP;
    std::string name_base;
    Optitrack(ros::NodeHandle& nh) {
        // Subscribe to the PoseStamped topic
        pose_subscriber_ = nh.subscribe("/vrpn_client_node/TargetRobetarme/pose_transform", 10, &Optitrack::CC_vrpn_obj, this);
        initialPosePub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
        nh.getParam("/optitrack_publisher/name_base_optitrack", name_base);
    }

    void CC_vrpn_obj(const geometry_msgs::PoseStamped::ConstPtr msg) {  // Method/function defined inside the class
        pos_obj    = {msg->pose.position.x,msg->pose.position.y,msg->pose.position.z};
        quat_obj   = {msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w};
    } 

    boustrophedon_msgs::PlanMowingPathGoal  ComputeGoal(double width = 0.64, double height = 0.49) {

      double x = width/2;
      double y = height/2;
      
      p1 = {pos_obj(0)+x, pos_obj(1)+y,pos_obj(2)};
      p2 = {pos_obj(0)-x, pos_obj(1)+y,pos_obj(2)};
      p3 = {pos_obj(0)-x, pos_obj(1)-y,pos_obj(2)};
      p4 = {pos_obj(0)+x, pos_obj(1)-y,pos_obj(2)};
      name_base = "base";


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

    return goal;
}
  void publishInitialPose() {

      double delta = 0.1;
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

Eigen::Quaterniond orientation() {
    // Initial quaternion (replace with your values)
    Eigen::Quaterniond initialQuaternion(quat_obj(3),quat_obj(0),quat_obj(1),quat_obj(2));

    // Rotation axis (x-axis in this case)
    Eigen::Vector3d rotationAxis(0.0, 1.0, 0.0);

    // Rotation angle (180 degrees)
    double rotationAngle = M_PI;  // M_PI is the constant for pi

    // Create a quaternion representing the rotation
    Eigen::Quaterniond rotationQuaternion(Eigen::AngleAxisd(rotationAngle, rotationAxis));
    

    // Perform the rotation
    Eigen::Quaterniond finalQuaternion = rotationQuaternion * initialQuaternion;

    // // Display the results
    // std::cout << "Initial Quaternion: " << initialQuaternion.coeffs().transpose() << std::endl;
    // std::cout << "Rotation Quaternion: " << rotationQuaternion.coeffs().transpose() << std::endl;
    // std::cout << "Final Quaternion: " << finalQuaternion.coeffs().transpose() << std::endl;

    return finalQuaternion;
}



    private:
    ros::Subscriber pose_subscriber_;
    ros::Publisher initialPosePub_;
};

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
// this function take the path comoute from server and create a linear DS
//when the eef is close the the next point it change the goal until the last point of the path
Eigen::Vector3f calaulteVelocityCommand(nav_msgs::Path& path_transf, Eigen::Vector3f real_pose_)
{
  double dx,dy,dz;
  double scale_vel=1.2;
  Eigen::Vector3f d_vel_;
  Eigen::Vector3f path_point;

  if (i_follow < path_transf.poses.size() - 1)
  {
    path_point(0)=path_transf.poses[i_follow + 1].pose.position.x;
    path_point(1)=path_transf.poses[i_follow + 1].pose.position.y;
    path_point(2)=path_transf.poses[i_follow + 1].pose.position.z;
  

    // Eigen::Vector4f target_ori=1;
    // Eigen::Matrix3f pathRotMat=ori2rotmat;
    // path_point=pathRotMat*path_point;

    dx = path_point(0) - real_pose_(0);
    dy = path_point(1) - real_pose_(1);
    dz = path_point(2) - real_pose_(2);

    d_vel_(0)=dx*scale_vel;
    d_vel_(1)=dy*scale_vel;
    d_vel_(2)=dz*scale_vel;

    if (d_vel_.norm()<=0.04)
    {
      i_follow+=1;
    }

  }else
  {
    path_point(0)=path_transf.poses[i_follow].pose.position.x;
    path_point(1)=path_transf.poses[i_follow].pose.position.y;
    path_point(2)=path_transf.poses[i_follow].pose.position.z;

    dx = path_point(2) - real_pose_(0);
    dy = path_point(1) - real_pose_(1);
    dz = path_point(0) - real_pose_(2);

    d_vel_(0)=dx;
    d_vel_(1)=dy;
    d_vel_(2)=dz;
  }

  // std::cerr<<"i_follow: "<<i_follow << std::endl;
  // std::cerr<<"real_pose_: "<< real_pose_(0) <<","<< real_pose_(1) <<","<< real_pose_(2) <<"," << std::endl;
  // std::cerr<<"striping_plan: "<< path_transf.poses[i_follow + 1].pose.position.x<<","
  //                             << path_transf.poses[i_follow + 1].pose.position.y <<","
  //                             << path_transf.poses[i_follow + 1].pose.position.z<<"," << std::endl;
  // std::cerr<<"vel dx: "<< dx <<","<< dy <<","<< dz <<"," << std::endl;
  // std::cerr<<"d_vel_: "<< d_vel_(0) <<","<< d_vel_(1) <<","<< d_vel_(2) <<"," << std::endl;
  // std::cerr<<"d_vel_.norm(): "<<d_vel_.norm() << std::endl;

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
    Optitrack optitrack(n);
    // Define the parameter name
    std::string param_name = "robot";
    // Declare a variable to store the parameter value
    std::string robot_name;
    // Try to get the parameter value
    n.getParam(param_name, robot_name);

    ros::Subscriber sub_real_pose_= 
        n.subscribe( robot_name + "/ee_info/Pose" , 1000, &UpdateRealPosition, ros::TransportHints().reliable().tcpNoDelay());
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
    goal = optitrack.ComputeGoal();

    polygon_pub.publish(goal.property);

    ROS_INFO_STREAM("Waiting for goal");
    optitrack.publishInitialPose();
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

        std::cout << "Result with : " << result->plan.points.size() << std::endl;

        convertStripingPlanToPath(result->plan, path);
        break;
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
    while (ros::ok())
    {
      ros::Time start_time = ros::Time::now();

      nav_msgs::Path path_transformed = optitrack.transformPath(path);

      path_pub.publish(path_transformed);

      ros::Time end_time = ros::Time::now();
      ros::Duration elapsed_time = end_time - start_time;
      //ROS_INFO_STREAM("Time elapsed: " << elapsed_time.toSec() << " seconds");

      // ROS_INFO_STREAM("real_pose_: " << real_pose_ );
      
      desired_vel_filtered_=calaulteVelocityCommand(path_transformed, real_pose_);
      //ROS_INFO_STREAM("desired_vel_filtered_: " << desired_vel_filtered_ );
      msg_desired_vel_filtered_.position.x  = desired_vel_filtered_(0);
      msg_desired_vel_filtered_.position.y  = desired_vel_filtered_(1);
      msg_desired_vel_filtered_.position.z  = desired_vel_filtered_(2);
      msg_desired_vel_filtered_.orientation.x = optitrack.quat_obj(0);
      msg_desired_vel_filtered_.orientation.y = optitrack.quat_obj(1);  
      msg_desired_vel_filtered_.orientation.z = optitrack.quat_obj(2);  
      msg_desired_vel_filtered_.orientation.w = optitrack.quat_obj(3);  
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

