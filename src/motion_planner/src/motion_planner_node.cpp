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



class LimitCycle {  
  public:
    bool got_initial_pose = false;
    geometry_msgs::PoseStamped initial_pose;
    geometry_msgs::Pose       msg_real_pose_;


    Eigen::Vector3f real_pose_;
    Eigen::Vector4f real_pose_ori_;

    Eigen::Matrix3f _wRb;				// Current rotation matrix [m] (3x1)
    Eigen::Vector3f _x;				// Current position [m] (3x1)
    Eigen::Vector4f _q;				// Current end effector quaternion (4x1)  
    double _toolOffsetFromEE= 0.0f;//---- knife tool with f/t sensor
    bool _firstRealPoseReceived;

    Eigen::Vector3f desired_vel_;
    Eigen::Vector3f desired_vel_filtered_;


    geometry_msgs::Twist msg_desired_vel_;
    geometry_msgs::Pose  msg_desired_vel_filtered_;

    std::size_t i_follow = 0;

    double dt_ = 0;
    int fs=0;
    Eigen::Vector3f target_pose_;
    Eigen::Vector4f desired_ori_velocity_filtered_;
    Eigen::Vector3f vd;

    double Velocity_limit_=0;
    bool finish =false;
    std::string robot_name = "ur5";

    LimitCycle(ros::NodeHandle& n) {
      // Subscribe to the Pose
      init_pose = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, &LimitCycle::initialPoseCallback, this);
      sub_real_pose_= n.subscribe<geometry_msgs::Pose>( robot_name + "/ee_info/Pose" , 1000, &LimitCycle::UpdateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());
      paramter_initialization();
    }
    void paramter_initialization(){
      fs = 300;
      dt_=1.0/fs;
      Velocity_limit_=1.5;
      _toolOffsetFromEE= 0.75f;//---- knife tool with f/t sensor
    }

    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& init_pose)
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
    //----------------define all function-------------------------------------
    // server has a service to convert StripingPlan to Path, but all it does it call this method
    bool convertStripingPlanToPath(const boustrophedon_msgs::StripingPlan& striping_plan, nav_msgs::Path& path)
    {
      path.header.frame_id = striping_plan.header.frame_id;
      path.header.stamp = striping_plan.header.stamp;

      path.poses.clear();


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
      double tol = 0.15;
      double dx,dy,dz;
      double desired_vel=0.04;
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

        std::cerr<<"target number: "<<i_follow<< std::endl;
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

        d_vel_(0)=0;
        d_vel_(1)=0;
        d_vel_(2)=0;
        vd(0)=0;
        vd(1)=0;
        vd(2)=0;
        finish =true;
    
      }

      if (vd.norm() > Velocity_limit_) {
        vd = vd / vd.norm() * Velocity_limit_;
          ROS_WARN_STREAM_THROTTLE(1.5, "TOO FAST");
      }
      return vd;
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
      double Cycle_speed_LC_=2.5* 3.14;
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
    private:
      ros::Subscriber init_pose;
      ros::Subscriber sub_real_pose_;
};


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
    LimitCycle limitcycle(n);
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
          limitcycle.convertStripingPlanToPath(result->plan, path);
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
      targetextraction.see_target();

      path_pub.publish(path_transformed);
    }
    while (ros::ok())
    {

      ros::Time start_time = ros::Time::now();


      path_pub.publish(path_transformed);

      ros::Time end_time = ros::Time::now();
      ros::Duration elapsed_time = end_time - start_time;
 
      double rad_up;

      if (startController == false)
      {
        pathplanner.set_strategique_position(n);
        //taking the first point of the path
        limitcycle.target_pose_(0)=path_transformed.poses[0].pose.position.x;
        limitcycle.target_pose_(1)=path_transformed.poses[0].pose.position.y;
        limitcycle.target_pose_(2)=path_transformed.poses[0].pose.position.z;
        
              //--- here waiting for orinetation control
        limitcycle.desired_ori_velocity_filtered_(0)=pathplanning.quat_obj(0);
        limitcycle.desired_ori_velocity_filtered_(1)=pathplanning.quat_obj(1);
        limitcycle.desired_ori_velocity_filtered_(2)=pathplanning.quat_obj(2);
        limitcycle.desired_ori_velocity_filtered_(3)=pathplanning.quat_obj(3);
        double optimum_radius;
        n.getParam("optimum_radius", optimum_radius);
        double flow_radius;
        n.getParam("flow_radius", flow_radius);

        rad_up = optimum_radius -flow_radius;
        n.getParam("/startController", startController);

      }
      else{
        limitcycle.desired_vel_filtered_=limitcycle.calculateVelocityCommand(path_transformed, limitcycle.real_pose_,limitcycle.desired_ori_velocity_filtered_, rad_up);
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





