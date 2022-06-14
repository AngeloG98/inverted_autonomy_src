#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <flight_msgs/TrajectoryPoint.h>
#include "flight_common/geometry_eigen_conversions.h"

namespace flight_common
{

struct TrajectoryPoint
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TrajectoryPoint();
    TrajectoryPoint(const flight_msgs::TrajectoryPoint& trajectory_point_msg);
    virtual ~TrajectoryPoint();

    //   quadrotor_msgs::TrajectoryPoint toRosMessage() const;

    ros::Duration time_from_start;

    // Pose
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    
    // Linear derivatives
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    Eigen::Vector3d jerk;
    Eigen::Vector3d snap;

    // Angular derivatives
    Eigen::Vector3d bodyrate;
    Eigen::Vector3d angular_acceleration;
    Eigen::Vector3d angular_jerk;
    Eigen::Vector3d angular_snap;

    // Euler yaw angle [rad]
    double yaw;
    double yaw_rate;
    double yaw_acceleration;
};

}