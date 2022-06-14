#include "flight_common/trajectory_point.h"

namespace flight_common
{

TrajectoryPoint::TrajectoryPoint() :
        time_from_start(ros::Duration(0.0)), position(Eigen::Vector3d::Zero()),
        orientation(Eigen::Quaterniond::Identity()),
        velocity(Eigen::Vector3d::Zero()), acceleration(
        Eigen::Vector3d::Zero()), jerk(Eigen::Vector3d::Zero()), snap(
        Eigen::Vector3d::Zero()), bodyrate(Eigen::Vector3d::Zero()),
        angular_acceleration(Eigen::Vector3d::Zero()),
        angular_jerk(Eigen::Vector3d::Zero()), angular_snap(
        Eigen::Vector3d::Zero()), yaw(0.0), yaw_rate(0.0),
        yaw_acceleration(0.0)
{
}

TrajectoryPoint::TrajectoryPoint(
        const flight_msgs::TrajectoryPoint& trajectory_point_msg)
{
    time_from_start = trajectory_point_msg.time_from_start;

    position = geometryToEigen(trajectory_point_msg.pose.position);
    orientation = geometryToEigen(trajectory_point_msg.pose.orientation);

    velocity = geometryToEigen(trajectory_point_msg.velocity.linear);
    acceleration = geometryToEigen(trajectory_point_msg.acceleration.linear);
    jerk = geometryToEigen(trajectory_point_msg.jerk.linear);
    snap = geometryToEigen(trajectory_point_msg.snap.linear);

    bodyrate = geometryToEigen(trajectory_point_msg.velocity.angular);
    angular_acceleration = geometryToEigen(
        trajectory_point_msg.acceleration.angular);
    angular_jerk = geometryToEigen(trajectory_point_msg.jerk.angular);
    angular_snap = geometryToEigen(trajectory_point_msg.snap.angular);

    yaw = trajectory_point_msg.heading;
    yaw_rate = trajectory_point_msg.heading_rate;
    yaw_acceleration = trajectory_point_msg.heading_acceleration;
}

TrajectoryPoint::~TrajectoryPoint()
{
}

}