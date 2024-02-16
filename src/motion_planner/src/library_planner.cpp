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
    double delta = 0.0;
    // Create a publisher for the /initialpose topic

    // Create and fill the message
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
    initialPos = initialPoseMsg.pose.;
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
      geometry_msgs::PoseStamped first_pose = transformedPath.poses[0];

      // Extract position (x, y, z)
      double x = first_pose.pose.position.x;
      double y = first_pose.pose.position.y;
      double z = first_pose.pose.position.z;
      firstPos = {x,y,z};
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

void PathPlanner::set_strategique_position(ros::NodeHandle& n){
    
    
    // Set values for a initial orientation
    std::vector<double> parameter_quat = {targetQuat.x(),targetQuat.y(),targetQuat.z(),targetQuat.w()};
    n.setParam("/initialQuat", parameter_quat);

    Eigen::Matrix3d rotationMatrix = targetQuat.toRotationMatrix();
    Eigen::Vector3d firstPosEigen(firstPos[0],firstPos[1],firstPos[2]);

    Eigen::Vector3d parameter_pos3f = firstPosEigen - toolOffsetFromTarget  *  rotationMatrix.col(2);
    
    std::vector<double> parameter_pos;
    parameter_pos.reserve(parameter_pos3f.size());  // Reserve space for efficiency
    for (int i = 0; i < parameter_pos3f.size(); ++i) {
        parameter_pos.push_back(static_cast<double>(parameter_pos3f[i]));
    }
    n.setParam("/initialPos", parameter_pos);
    n.setParam("/finalPos", parameter_pos);
}
 // server has a service to convert StripingPlan to Path, but all it does it call this method
bool PathPlanner::convertStripingPlanToPath(const boustrophedon_msgs::StripingPlan& striping_plan, nav_msgs::Path& path)
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


geometry_msgs::Quaternion PathPlanner::headingToQuaternion(double x, double y, double z)
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



DynamicalSystem::DynamicalSystem(ros::NodeHandle& n)
{
  // Subscribe to the Pose
  parameter_initialization();
  nh  = n;
  point_pub = nh.advertise<geometry_msgs::PointStamped>("path_point", 1);
  sub_real_pose= nh.subscribe<geometry_msgs::Pose>( robot_name + "/ee_info/Pose" , 1000, &DynamicalSystem::UpdateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());
}


void DynamicalSystem::parameter_initialization(){
  fs = 300;
  dt=1.0/fs;
  Velocity_limit=1.5;

  // Get the path to the package
  std::string package_path = ros::package::getPath("motion_planner"); // Replace "your_package" with your actual package name
  // Load parameters from YAML file
  std::string yaml_path = package_path + "/config/config.yaml";
  YAML::Node config = YAML::LoadFile(yaml_path);
  // Access parameters from the YAML file
  robot_name = config["robot_name"].as<std::string>();
  limit_cycle_radius = config["limit_cycle_radius"].as<double>();
  toolOffsetFromTarget = config["toolOffsetFromTarget"].as<double>();
  flow_radius = config["flow_radius"].as<double>();
  sum_rad = flow_radius + limit_cycle_radius;
}


void DynamicalSystem::set_goal(nav_msgs::Path path ,Eigen::Quaterniond quat)
{
  centerLimitCycle(0)=path.poses[0].pose.position.x;
  centerLimitCycle(1)=path.poses[0].pose.position.y;
  centerLimitCycle(2)=path.poses[0].pose.position.z;
  
        //--- here waiting for orinetation control
  desired_ori_velocity_filtered_(0) = quat.x();
  desired_ori_velocity_filtered_(1) = quat.y();
  desired_ori_velocity_filtered_(2) = quat.z();
  desired_ori_velocity_filtered_(3) = quat.w();
}
geometry_msgs::Pose DynamicalSystem::get_ros_msg_vel()
{
  msg_desired_vel_filtered_.position.x  = desired_vel_filtered_(0);
  msg_desired_vel_filtered_.position.y  = desired_vel_filtered_(1);
  msg_desired_vel_filtered_.position.z  = desired_vel_filtered_(2);
  msg_desired_vel_filtered_.orientation.x = desired_ori_velocity_filtered_(0);
  msg_desired_vel_filtered_.orientation.y = desired_ori_velocity_filtered_(1);  
  msg_desired_vel_filtered_.orientation.z = desired_ori_velocity_filtered_(2);  
  msg_desired_vel_filtered_.orientation.w = desired_ori_velocity_filtered_(3); 
  return msg_desired_vel_filtered_;
}

void DynamicalSystem::UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) {

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
  x << msg_real_pose_.position.x, msg_real_pose_.position.y, msg_real_pose_.position.z;
  q= Eigen::Quaterniond(msg_real_pose_.orientation.w, msg_real_pose_.orientation.x, msg_real_pose_.orientation.y, msg_real_pose_.orientation.z);

  Eigen::Quaterniond normalizedQuat = q.normalized();
  Eigen::Matrix3d rotation_matrix = normalizedQuat.toRotationMatrix();

  // _wRb = quaternionToRotationMatrix(_q);

  x = x+toolOffsetFromTarget*rotation_matrix.col(2);
  
  for (size_t i = 0; i < 3; i++)
  {
    real_pose_(i)=x(i);
  }

  if(!_firstRealPoseReceived)
  {
    _firstRealPoseReceived = true;
  }
}
  //----------------define all function-------------------------------------
   
    // this function take the path comoute from server and create a linear DS
    //when the eef is close the the next point it change the goal until the last point of the path
Eigen::Vector3d DynamicalSystem::get_DS_vel(nav_msgs::Path& path_transf,double radius)
{ 
  double tol = 0.2;
  double dx,dy,dz;
  double desired_vel=0.04;
  double norm;
  double scale_vel;
  Eigen::Vector3d d_vel_;
  Eigen::Vector3d path_point;
  

  if (i_follow < path_transf.poses.size() - 1)
  {
    path_point(0)=path_transf.poses[i_follow + 1].pose.position.x;
    path_point(1)=path_transf.poses[i_follow + 1].pose.position.y;
    path_point(2)=path_transf.poses[i_follow + 1].pose.position.z;
    publishPointStamped(path_point);
  
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
      centerLimitCycle+=d_vel_*dt;
    }

    std::cerr<<"target number: "<<i_follow<< std::endl;
    std::cerr<<"error"<<(std::sqrt((path_point - centerLimitCycle).norm()))<< std::endl;
    if (std::sqrt((path_point - centerLimitCycle).norm())<=tol)
    {
      i_follow+=1;
    }
    updateLimitCycle3DPosVel_with2DLC(real_pose_,centerLimitCycle, radius );

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

  if (vd.norm() > Velocity_limit) {
    vd = vd / vd.norm() * Velocity_limit;
      ROS_WARN_STREAM_THROTTLE(1.5, "TOO FAST");
  }
  return vd;
}

void DynamicalSystem::publishPointStamped(const Eigen::Vector3d&  path_point ) {

geometry_msgs::PointStamped point_stamped_msg;
point_stamped_msg.header.stamp = ros::Time::now();
point_stamped_msg.header.frame_id = "base"; // Set your desired frame_id

// Assign Eigen vector components to PointStamped message
point_stamped_msg.point.x = path_point(0);
point_stamped_msg.point.y = path_point(1);
point_stamped_msg.point.z = path_point(2);

// Publish the PointStamped message
point_pub.publish(point_stamped_msg);
}


void DynamicalSystem::updateLimitCycle3DPosVel_with2DLC(Eigen::Vector3d pose, Eigen::Vector3d target_pose_cricleDS, double radius) 
{
  double Convergence_Rate_LC_=10;
  double Cycle_radius_LC_=radius;//0.015;
  double Cycle_speed_LC_=2.5* 3.14;
  float a[2] = {1., 1.};
  float norm_a=std::sqrt(a[0]*a[0]+a[1]*a[1]);
  for (int i=0; i<2; i++)
      a[i]=a[i]/norm_a;

  Eigen::Vector3d velocity;
  Eigen::Vector3d pose_eig;

  //--- trans real ori to rotation matrix
  Eigen::Quaterniond new_quat;
  new_quat.w()=desired_ori_velocity_filtered_(3);
  new_quat.x()=desired_ori_velocity_filtered_(0);
  new_quat.y()=desired_ori_velocity_filtered_(1);
  new_quat.z()=desired_ori_velocity_filtered_(2);
  Eigen::Matrix3d rotMat = new_quat.toRotationMatrix();

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