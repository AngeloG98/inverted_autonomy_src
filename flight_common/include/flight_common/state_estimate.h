#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

namespace flight_common
{

struct StateEstimate
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StateEstimate();
    virtual ~StateEstimate();

    void StateEstimateFromOdom(const nav_msgs::Odometry& odom_msg);
    void StateEstimateFromImu(const sensor_msgs::Imu& imu_msg);
    bool isValid() const;

    ros::Time timestamp;
    enum class CoordinateFrame
    {
        INVALID,
        WORLD,
        LOCAL
    } coordinate_frame;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d bodyrate;
    // mavros axises rotate
    Eigen::Matrix3d R_ENU_TO_NED; // R^e_e'
    Eigen::Matrix3d R_FLU_TO_FRD; // R^b_b'
};
}