#include "library_planner.h"
#include <ros/package.h>

//TargetExrtaction functon
TargetExtraction::TargetExtraction(ros::NodeHandle& nh)
    : poseTargetSub(nh.subscribe("/vrpn_client_node/TargetRobetarme/pose_transform", 10, &TargetExtraction::CC_vrpn_target, this)){

    originalPolygonPub = nh.advertise<geometry_msgs::PolygonStamped>("/original_polygon", 1, true);
    // Get the path to the package
    std::string package_path = ros::package::getPath("motion_planner"); // Replace "your_package" with your actual package name

    // Load parameters from YAML file
    std::string yaml_path = package_path + "/config/config.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path);

    // Access parameters from the YAML file
    height_target = config["height_target"].as<double>();
    width_target = config["width_target"].as<double>();

    while(!targetReceived){
      ros::spinOnce();
    }
    std::cout << "rostopic for the target received" << std::endl;

}

std::vector<Eigen::Vector3d> TargetExtraction::get_polygons() {
    // Desired displacements
    std::vector<Eigen::Vector3d> displacements{
        Eigen::Vector3d(width_target / 2.0, height_target / 2.0, 0),
        Eigen::Vector3d(-width_target / 2.0, height_target / 2.0, 0),
        Eigen::Vector3d(-width_target / 2.0, -height_target / 2.0, 0),
        Eigen::Vector3d(width_target / 2.0, -height_target / 2.0, 0)
    };

    // Extract position and ensure quaternion is normalized
    Eigen::Vector3d position = targetPos;
    Eigen::Quaterniond normalizedQuat = targetQuat.normalized();
    Eigen::Matrix3d rotation_matrix = normalizedQuat.toRotationMatrix();

    // Calculate new positions
    polygons_positions.clear();  // Clear existing positions
    for (const auto& displacement : displacements) {
        polygons_positions.push_back(position + rotation_matrix * displacement);
    }
    std::cout<<"Polygons well computed"<<std::endl;
    // std::cout<< polygons_positions[0] <<std::endl;
    // std::cout<< polygons_positions[1] <<std::endl;
    // std::cout<< polygons_positions[2] <<std::endl;
    // std::cout<< polygons_positions[3] <<std::endl;

    return polygons_positions;
}
Eigen::Quaterniond TargetExtraction::get_quat_target() {
    return targetQuat;
}
Eigen::Vector3d TargetExtraction::get_pos_target() {
    return targetPos;
}

void TargetExtraction::CC_vrpn_target(const geometry_msgs::PoseStamped::ConstPtr msg) {
    targetPos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    targetQuat = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    targetReceived = true;
}
void TargetExtraction::see_target(){

      geometry_msgs::PolygonStamped visualpolygonTarget;
      visualpolygonTarget.header.frame_id = "base";  
      visualpolygonTarget.header.stamp = ros::Time::now();

      for (const auto& point : polygons_positions) {
          geometry_msgs::Point32 msg_point;
          msg_point.x = point.x();
          msg_point.y = point.y();
          msg_point.z = point.z();
          visualpolygonTarget.polygon.points.push_back(msg_point);
      }
      originalPolygonPub.publish(visualpolygonTarget);
  }

//path panning functon

// Constructor definition
PathPlanner::PathPlanner(ros::NodeHandle& nh, Eigen::Quaterniond target_quat, Eigen::Vector3d target_pos, std::vector<Eigen::Vector3d> polygons_positions) : 
  initialPosePub_(nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10))  {
    transformedPolygonPub = nh.advertise<geometry_msgs::PolygonStamped>("/flat_polygon", 1, true);

    std::string package_path = ros::package::getPath("motion_planner"); // Replace "your_package" with your actual package name
    // Load parameters from YAML file
    std::string yaml_path = package_path + "/config/config.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path);

    // Access parameters from the YAML file
    limit_cycle_radius = config["limit_cycle_radius"].as<double>();
    toolOffsetFromTarget = config["toolOffsetFromTarget"].as<double>();
    flow_radius = config["flow_radius"].as<double>();
    sum_rad = flow_radius + limit_cycle_radius;

    polygonsPositions = polygons_positions;
    targetQuat = target_quat ;
    targetPos = target_pos;
  }

std::vector<Eigen::Vector3d> PathPlanner::get_planner_points() {

    std::vector<Eigen::Vector3d> rotated_points;
    Eigen::Affine3d transformation = Eigen::Translation3d(targetPos(0),targetPos(1),targetPos(2)) * targetQuat;

    Eigen::MatrixXd points_matrix(polygonsPositions.size(), 3);
    for (size_t i = 0; i < polygonsPositions.size(); ++i) {
      Eigen::Vector3d rotated_point = transformation.inverse() * polygonsPositions[i];
      rotated_points.push_back(rotated_point);

    }

    return rotated_points;
}


boustrophedon_msgs::PlanMowingPathGoal  PathPlanner::ComputeGoal() {
  flatPolygons = get_planner_points();

  Eigen::Vector3d p1 = flatPolygons[0];
  Eigen::Vector3d p2 = flatPolygons[1];
  Eigen::Vector3d p3 = flatPolygons[2];
  Eigen::Vector3d p4 = flatPolygons[3];
  std::cout<< p4<< std::endl;
  //polygon for the server
  boustrophedon_msgs::PlanMowingPathGoal goal;

  goal.property.header.stamp = ros::Time::now();
  goal.property.header.frame_id = "base";
  goal.property.polygon.points.resize(4);
  goal.property.polygon.points[0].x = p1(0);
  goal.property.polygon.points[0].y = p1(1);
  goal.property.polygon.points[0].z = p1(2);
  goal.property.polygon.points[1].x = p2(0);
  goal.property.polygon.points[1].y = p2(1);
  goal.property.polygon.points[1].z = p2(2);           
  goal.property.polygon.points[2].x = p3(0);
  goal.property.polygon.points[2].y = p3(1);
  goal.property.polygon.points[2].z = p3(2);
  goal.property.polygon.points[3].x = p4(0);
  goal.property.polygon.points[3].y = p4(1);
  goal.property.polygon.points[3].z = p4(2);

  goal.robot_position.pose.orientation.x = 0.0;
  goal.robot_position.pose.orientation.y = 0.0;
  goal.robot_position.pose.orientation.z = 0.0;
  goal.robot_position.pose.orientation.w = 1.0;

  return goal;
}


// Method to calculate optimization parameters
void PathPlanner::optimization_parameter(ros::NodeHandle& n) {

    // Publish parameters as ROS parameters
    n.setParam("/boustrophedon_server/stripe_separation", 2 *sum_rad);
    n.setParam("optimum_radius", sum_rad);
}


void PathPlanner::publishInitialPose() {
    double maxZ = -std::numeric_limits<double>::infinity();
    std::vector<Eigen::Vector3d> highestZPoints;
    
    std::vector<Eigen::Vector3d> points = polygonsPositions;
    for (const auto& point : points) {
        if (point.z() > maxZ) {
            maxZ = point.z();
            highestZPoints.clear();  // Clear previous points with lower z
            highestZPoints.push_back(point);
        } else if (point.z() == maxZ) {
            highestZPoints.push_back(point);
        }
    }

   
    Eigen::Vector3d pointInitial = highestZPoints[0];
    double delta = 0.3;
    // Create a publisher for the /initialpose topic

    // Create and fill the message
    geometry_msgs::PoseWithCovarianceStamped initialPoseMsg;
    initialPoseMsg.header.seq = 0;
    initialPoseMsg.header.stamp = ros::Time(0);
    initialPoseMsg.header.frame_id = "base";

    initialPoseMsg.pose.pose.position.x = pointInitial(0)- delta;
    initialPoseMsg.pose.pose.position.y = pointInitial(1)- delta;
    initialPoseMsg.pose.pose.position.z = pointInitial(2);

    initialPoseMsg.pose.pose.orientation.x = 0.0;
    initialPoseMsg.pose.pose.orientation.y = 0.0;
    initialPoseMsg.pose.pose.orientation.z = 0.0;
    initialPoseMsg.pose.pose.orientation.w = 1.0;

    initialPoseMsg.pose.covariance.fill(0.0);  // Fill the covariance with zeros

    initialPosePub_.publish(initialPoseMsg);
}
  

nav_msgs::Path PathPlanner::get_transformed_path(const nav_msgs::Path& originalPath) {
    nav_msgs::Path transformedPath;
    Eigen::Affine3d transformation = Eigen::Translation3d(targetPos(0),targetPos(1),targetPos(2)) * targetQuat;

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

void PathPlanner::see_target_flat(){

    geometry_msgs::PolygonStamped visualpolygonTarget;
    visualpolygonTarget.header.frame_id = "base";  
    visualpolygonTarget.header.stamp = ros::Time::now();

    for (const auto& point : flatPolygons) {
        geometry_msgs::Point32 msg_point;
        msg_point.x = point.x();
        msg_point.y = point.y();
        msg_point.z = point.z();
        visualpolygonTarget.polygon.points.push_back(msg_point);
    }
    transformedPolygonPub.publish(visualpolygonTarget);
}

void PathPlanner::set_strategique_position(ros::NodeHandle& nh){
    //taking the first point of the path
    limitcycle.target_pose_(0)=path_transformed.poses[0].pose.position.x;
    limitcycle.target_pose_(1)=path_transformed.poses[0].pose.position.y;
    limitcycle.target_pose_(2)=path_transformed.poses[0].pose.position.z;
    
    // Set values for a initial orientation
    std::vector<double> parameter_quat = {targetQuat.x(),targetQuat.y(),targetQuat.z(),targetQuat.w()};
    nh.setParam("/initialQuat", parameter_quat);

    Eigen::Vector4f Quat4f = targetQuat.cast<float>();
    Eigen::Matrix3d rotationMatrix = targetQuat.toRotationMatrix();

    Eigen::Vector3d parameter_pos3f = targetPos - toolOffsetFromTarget  *  rotationMatrix.col(2);
    
    std::vector<double> parameter_pos;
    parameter_pos.reserve(parameter_pos3f.size());  // Reserve space for efficiency
    for (int i = 0; i < parameter_pos3f.size(); ++i) {
        parameter_pos.push_back(static_cast<double>(parameter_pos3f[i]));
    }
    n.setParam("/initialPos", parameter_pos);
    n.setParam("/finalPos", parameter_pos);

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



DynamicalSystem::DynamicalSystem(ros::NodeHandle& nh)
{

    // Get the path to the package
    std::string package_path = ros::package::getPath("motion_planner"); // Replace "your_package" with your actual package name
    // Load parameters from YAML file
    std::string yaml_path = package_path + "/config/config.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path);

    // Access parameters from the YAML file
    limit_cycle_radius = config["limit_cycle_radius"].as<double>();
    toolOffsetFromTarget = config["toolOffsetFromTarget"].as<double>();
    flow_radius = config["flow_radius"].as<double>();
    sum_rad = flow_radius + limit_cycle_radius;

}