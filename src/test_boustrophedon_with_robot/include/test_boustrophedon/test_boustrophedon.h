#ifndef __TEST_BOUSTROPHEDON_H__
#define __TEST_BOUSTROPHEDON_H__

#include "geometry_msgs/Pose.h"
//---- eigen and vector
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>

class test_mini_code {

private:
    //----- Messages
    geometry_msgs::Pose       msg_real_pose_;

    //---- input signal 
    Eigen::Vector3f real_pose_;

};

#endif