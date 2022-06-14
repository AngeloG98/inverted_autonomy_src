#include "flight_common/math_common.h"
#include "flight_common/state_estimate.h"

namespace flight_common
{

StateEstimate::StateEstimate():
        timestamp(ros::Time::now()), coordinate_frame(CoordinateFrame::INVALID),
        position(Eigen::Vector3d::Zero()), velocity(Eigen::Vector3d::Zero()),
        orientation(Eigen::Quaterniond::Identity()),
        bodyrate(Eigen::Vector3d::Zero())
{
    // mavros axises rotate
    // R^e_e'
    R_ENU_TO_NED << 0, 1, 0,
        1, 0, 0,
        0, 0, -1;
    // R^b_b'
    R_FLU_TO_FRD << 1, 0, 0,
        0, -1, 0,
        0, 0, -1;
}

StateEstimate::~StateEstimate()
{
}

void StateEstimate::StateEstimateFromOdom(const nav_msgs::Odometry& odom_msg){
    // timestamp = ros::Time::now();
    coordinate_frame = CoordinateFrame::LOCAL;
    position = R_ENU_TO_NED * Eigen::Vector3d(odom_msg.pose.pose.position.x,
                                              odom_msg.pose.pose.position.y,
                                              odom_msg.pose.pose.position.z);
    velocity = quaternionToRotationMatrix(orientation) * R_FLU_TO_FRD * Eigen::Vector3d(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z);
    // velocity = Eigen::Vector3d(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z);
}

void StateEstimate::StateEstimateFromImu(const sensor_msgs::Imu& imu_msg){
    timestamp = ros::Time::now();
    coordinate_frame = CoordinateFrame::LOCAL;
    
    // Quaternion from FLU TO ENU, which means R_tmp is R^e'_b'
    Eigen::Quaterniond orientation_tmp;
    orientation_tmp.w() = imu_msg.orientation.w;
    orientation_tmp.x() = imu_msg.orientation.x;
    orientation_tmp.y() = imu_msg.orientation.y;
    orientation_tmp.z() = imu_msg.orientation.z;
    Eigen::Matrix3d R_tmp = quaternionToRotationMatrix(orientation_tmp);
    // R from FRD TO NED, R = R^e_e' * R^e'_b' * (R^b_b')^T
    Eigen::Matrix3d R = R_ENU_TO_NED * R_tmp * R_FLU_TO_FRD.transpose();
    orientation = RotationMatrixToQuaternion(R);

    bodyrate = R_FLU_TO_FRD * Eigen::Vector3d(imu_msg.angular_velocity.x,
                                              imu_msg.angular_velocity.y,
                                              imu_msg.angular_velocity.z);
}

bool StateEstimate::isValid() const
{
    if (coordinate_frame == CoordinateFrame::INVALID)
    {
        return false;
    }
    if (std::isnan(position.norm()))
    {
        return false;
    }
    if (std::isnan(velocity.norm()))
    {
        return false;
    }
    if (std::isnan(orientation.norm()))
    {
        return false;
    }
    if (std::isnan(bodyrate.norm()))
    {
        return false;
    }

    return true;
}
    
} // namespace fligtcommom
