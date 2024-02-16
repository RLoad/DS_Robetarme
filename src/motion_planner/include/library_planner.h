#ifndef LIBRARY_PLANNER_H
#define LIBRARY_PLANNER_H

#include <ros/ros.h>
#include <ros/package.h>
#include "geometry_msgs/Pose.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include "geometry_msgs/Twist.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <yaml-cpp/yaml.h>  
#include <Eigen/Dense>
#include <boustrophedon_msgs/PlanMowingPathAction.h>
#include <boustrophedon_msgs/ConvertPlanToPath.h>
#include <vector>
#include <string>

// this class extrac the polygon and the Pose center of the Target
// it should be replace by the same extraction but from a CAD
class TargetExtraction {
public:
    TargetExtraction(ros::NodeHandle& nh);
    std::vector<Eigen::Vector3d> get_polygons();
    Eigen::Quaterniond get_quat_target();
    Eigen::Vector3d get_pos_target();
    void CC_vrpn_target(const geometry_msgs::PoseStamped::ConstPtr msg);
    void see_target();
private:
    bool targetReceived = false;
    double height_target, width_target;
    Eigen::Quaterniond targetQuat;
    Eigen::Vector3d targetPos;
    std::vector<Eigen::Vector3d> polygons_positions;
    ros::Subscriber poseTargetSub;
    ros::Publisher originalPolygonPub;

};

// This class create a path that navigate throug the target depending the polygons
class PathPlanner {
public:
    std::vector<double> firstPos;
    double sum_rad;
    Eigen::Quaterniond targetQuat;
    Eigen::Vector3d targetPos;
    geometry_msgs::PoseStamped initialPose;
    geometry_msgs::PoseWithCovarianceStamped initialPoseMsg;
    
    PathPlanner(ros::NodeHandle& nh, Eigen::Quaterniond target_quat, Eigen::Vector3d target_pos, std::vector<Eigen::Vector3d> polygons_positions);
    geometry_msgs::PoseStamped get_initial_pos_ros_msg();
    std::vector<Eigen::Vector3d> get_planner_points();
    boustrophedon_msgs::PlanMowingPathGoal  ComputeGoal();
    void optimization_parameter(ros::NodeHandle& n);
    void publishInitialPose();
    nav_msgs::Path get_transformed_path(const nav_msgs::Path& originalPath);
    void see_target_flat();
    void set_strategique_position(ros::NodeHandle& n);
    bool convertStripingPlanToPath(const boustrophedon_msgs::StripingPlan& striping_plan, nav_msgs::Path& path);
    geometry_msgs::Quaternion headingToQuaternion(double x, double y, double z);

private:
    double flow_radius, limit_cycle_radius, optimum_radius, toolOffsetFromTarget;
    ros::Publisher initialPosePub_;
    ros::Publisher transformedPolygonPub;
    std::vector<Eigen::Vector3d> polygonsPositions;
    std::vector<Eigen::Vector3d> flatPolygons;
};

//This class managed the Dynamical system to return the desired veloctiy  in function of the eef pose and path
class DynamicalSystem {
public:
    bool finish =false;

    DynamicalSystem(ros::NodeHandle& nh, double freq);
    void parameter_initialization();
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& init_pose);
    void set_goal(nav_msgs::Path path ,Eigen::Quaterniond quat);
    geometry_msgs::Pose get_ros_msg_vel();
    void UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) ;
    Eigen::Vector3d get_DS_vel(nav_msgs::Path& path_transf, double radius);
    void publishPointStamped(const Eigen::Vector3d&  path_point );
    Eigen::Matrix3d quaternionToRotationMatrix(Eigen::Vector4d q);
    void updateLimitCycle3DPosVel_with2DLC(Eigen::Vector3d pose, Eigen::Vector3d target_pose_cricleDS, double radius);
    void set_linear_speed(double speed);
    void set_tolerance_next_point(double tol);

private:
    double fs;
    double toleranceToNextPoint = 0.2;
    double linearVelExpected  = 0.04;

    bool _firstRealPoseReceived;
    std::size_t i_follow = 0;

    geometry_msgs::PoseStamped initial_pose;
    geometry_msgs::Pose       msg_real_pose_;
    geometry_msgs::Twist msg_desired_vel_;
    geometry_msgs::Pose  msg_desired_vel_filtered_;

    Eigen::Vector3d desired_vel;
    Eigen::Vector3d real_pose;
    Eigen::Quaterniond realQuat;	

    Eigen::Vector3d centerLimitCycle;
    Eigen::Vector4d desired_ori_velocity_filtered_;
    std::string robot_name;

    double flow_radius, limit_cycle_radius, sum_rad, toolOffsetFromTarget, dt, Velocity_limit;
    bool targetReceived = false;
    std::vector<Eigen::Vector3d> polygons_positions;
    ros::Subscriber sub_real_pose;
    ros::Publisher point_pub;
    ros::NodeHandle nh;
};

#endif  // LIBRARY_PLANNER_H
