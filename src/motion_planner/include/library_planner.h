#ifndef LIBRARY_PLANNER_H
#define LIBRARY_PLANNER_H

#include <ros/ros.h>
#include <ros/package.h>
#include "geometry_msgs/Pose.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PolygonStamped.h>
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
    std::vector<double> FirstPos;
    geometry_msgs::PoseStamped msgP;
    std::string name_base;
    PathPlanner(ros::NodeHandle& nh, Eigen::Quaterniond target_quat, Eigen::Vector3d target_pos, std::vector<Eigen::Vector3d> polygons_positions);
    std::vector<Eigen::Vector3d> get_planner_points();
    boustrophedon_msgs::PlanMowingPathGoal  ComputeGoal();
    void optimization_parameter(ros::NodeHandle& n);
    void publishInitialPose();
    nav_msgs::Path get_transformed_path(const nav_msgs::Path& originalPath);
    void see_target_flat();


private:
    double flow_radius, limit_cycle_radius, sum_rad, optimum_radius;
    ros::Publisher initialPosePub_;
    ros::Publisher transformedPolygonPub;
    std::vector<Eigen::Vector3d> polygonsPositions;
    std::vector<Eigen::Vector3d> flatPolygons;
    Eigen::Quaterniond targetQuat;
    Eigen::Vector3d targetPos;
    double toolOffsetFromTarget;
};

class DynamicalSystem {
public:
    DynamicalSystem(ros::NodeHandle& nh);
    
private:
    double flow_radius, limit_cycle_radius, sum_rad, optimum_radius;
    bool targetReceived = false;
    double height_target, width_target;
    Eigen::Quaterniond targetQuat;
    Eigen::Vector3d targetPos;
    std::vector<Eigen::Vector3d> polygons_positions;
    ros::Subscriber poseTargetSub;
    ros::Publisher originalPolygonPub;

};

#endif  // LIBRARY_PLANNER_H
