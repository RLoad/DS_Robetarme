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

class PathPlanner {
public:
    std::vector<double> firstPos;
    double sum_rad;
    Eigen::Quaterniond targetQuat;
    Eigen::Vector3d targetPos;
    geometry_msgs::PoseStamped msgP;
    geometry_msgs::PoseWithCovarianceStamped initialPoseMsg;
    PathPlanner(ros::NodeHandle& nh, Eigen::Quaterniond target_quat, Eigen::Vector3d target_pos, std::vector<Eigen::Vector3d> polygons_positions);
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

class DynamicalSystem {
public:
    bool got_initial_pose = false;
    bool _firstRealPoseReceived;
    bool finish =false;
    double fs;

    Eigen::Vector3d desired_vel_;
    Eigen::Vector3d desired_vel_filtered_;

    std::size_t i_follow = 1;

    Eigen::Vector3d centerLimitCycle;
    Eigen::Vector4d desired_ori_velocity_filtered_;
    Eigen::Vector3d vd;

    DynamicalSystem(ros::NodeHandle& nh);
    void parameter_initialization();
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& init_pose);
    void set_goal(nav_msgs::Path path ,Eigen::Quaterniond quat);
    geometry_msgs::Pose get_ros_msg_vel();
    void UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) ;
    Eigen::Vector3d get_DS_vel(nav_msgs::Path& path_transf, double radius);
    void publishPointStamped(const Eigen::Vector3d&  path_point );
    Eigen::Matrix3d quaternionToRotationMatrix(Eigen::Vector4d q);
    void updateLimitCycle3DPosVel_with2DLC(Eigen::Vector3d pose, Eigen::Vector3d target_pose_cricleDS, double radius);


private:
    geometry_msgs::PoseStamped initial_pose;
    geometry_msgs::Pose       msg_real_pose_;
    geometry_msgs::Twist msg_desired_vel_;
    geometry_msgs::Pose  msg_desired_vel_filtered_;

    Eigen::Vector3d real_pose_, x;
    Eigen::Vector4d real_pose_ori_;
    Eigen::Quaterniond q;	
    std::string robot_name;

    double flow_radius, limit_cycle_radius, sum_rad, toolOffsetFromTarget, dt, Velocity_limit;
    bool targetReceived = false;
    std::vector<Eigen::Vector3d> polygons_positions;
    ros::Subscriber sub_real_pose;
    ros::Publisher point_pub;
    ros::NodeHandle nh;
};

#endif  // LIBRARY_PLANNER_H
