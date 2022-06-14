#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/AttitudeTarget.h>

//attitude ignore
int IGNORE_ROLL_RATE = 1;
int IGNORE_PITCH_RATE = 2;
int IGNORE_YAW_RATE_ATT = 4;
int IGNORE_THRUST = 64;
int IGNORE_ATTITUDE = 128;

namespace flight_common
{

struct ControlCommand
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ControlCommand();
    virtual ~ControlCommand();

    void reset();
    // to mavros PWM
    mavros_msgs::RCOut toMavrosMessagePWM() const;
    mavros_msgs::AttitudeTarget toMavrosMessageAttRate() const;

    ros::Time timestamp;
    enum class ControlMode
    {
        NONE,
        ATTITUDE,
        BODY_RATE,
        PWM
    } control_mode;

    // if ok to arm
    bool armed;
    
    Eigen::Quaterniond orientation;
    Eigen::Vector3d bodyrate;
    Eigen::Vector3d angular_acceleration;
    double collective_thrust;
    double norm_collective_thrust;
    Eigen::Vector4d rotor_thrust;

    Eigen::Vector3d torque;
    Eigen::Vector4d PWM;

    double alpha_guess;

    // mavros axises rotate
    Eigen::Matrix3d R_ENU_TO_NED; // R^e_e'
    Eigen::Matrix3d R_FLU_TO_FRD; // R^b_b'
};
}