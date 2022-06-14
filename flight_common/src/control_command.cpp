#include "flight_common/control_command.h"

namespace flight_common
{

ControlCommand::ControlCommand() :
        control_mode(ControlMode::NONE), armed(false), timestamp(ros::Time::now()),
        orientation(Eigen::Quaterniond::Identity()), bodyrate(Eigen::Vector3d::Zero()),
        angular_acceleration(Eigen::Vector3d::Zero()), collective_thrust(0.0), norm_collective_thrust(0.0),
        rotor_thrust(Eigen::Vector4d::Zero()),
        torque(Eigen::Vector3d::Zero()), PWM(Eigen::Vector4d::Zero()),
        alpha_guess(0.0)
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

mavros_msgs::RCOut
ControlCommand::toMavrosMessagePWM() const{
    
    mavros_msgs::RCOut pwm_out;
    pwm_out.header.stamp = ros::Time::now();
    pwm_out.channels.resize(8);

    if (control_mode == ControlMode::PWM)
    {
        for (int i =0; i < 4; i++){
            pwm_out.channels[i] = (uint16_t)PWM(i);
        }
    }
    else
    {
        for (int i =0; i < 4; i++){
            pwm_out.channels[i] = (uint16_t)900;
        }
    }
    for (int i = 4; i < 8; i++){
        pwm_out.channels[i] = (uint16_t)900;
    }
    return pwm_out;
}

mavros_msgs::AttitudeTarget
ControlCommand::toMavrosMessageAttRate() const{

    mavros_msgs::AttitudeTarget att_rate;
    att_rate.header.stamp = ros::Time::now();
    att_rate.header.frame_id = "att_rate";
    att_rate.type_mask = IGNORE_ATTITUDE; // Ignore att messages

    if (control_mode == ControlMode::BODY_RATE)
    {
        Eigen::Vector3d bodyrate_flu = R_FLU_TO_FRD.transpose() * bodyrate;
        att_rate.body_rate.x = bodyrate_flu(0);
        att_rate.body_rate.y = bodyrate_flu(1);
        att_rate.body_rate.z = bodyrate_flu(2);
        att_rate.thrust = norm_collective_thrust;
    }
    else
    {
        att_rate.body_rate.x = 0.0;
        att_rate.body_rate.y = 0.0;
        att_rate.body_rate.z = 0.0;
        att_rate.thrust = 0.0;
    }
    return att_rate;
}

ControlCommand::~ControlCommand()
{
}

void ControlCommand::reset()
{
    timestamp = ros::Time::now();
    control_mode = ControlMode::BODY_RATE;
    armed = false;
    orientation = Eigen::Quaterniond::Identity();
    bodyrate = Eigen::Vector3d::Zero();
    angular_acceleration = Eigen::Vector3d::Zero();
    collective_thrust = 0.0;
    norm_collective_thrust = 0.0;
    rotor_thrust = Eigen::Vector4d::Zero();
    torque = Eigen::Vector3d::Zero();
    PWM = Eigen::Vector4d::Zero();
    alpha_guess = 0.0;
}
}