#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>
#include "flight_common/math_common.h"

namespace lwq_flatness
{
struct ModelHelper
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ModelHelper() {}
    ~ModelHelper() {}
    void setModelParams()
    {
        // body
        kappa = flight_common::degToRad(34.0);
        lwq_mass = 2.3; // [kg]
        lwq_Jxx = 0.0512;
        lwq_Jyy = 0.0554;
        lwq_Jzz = 0.076;
        gravity = 9.80;
        
        // rotor
        rotor_dx = 0.25; // [m]
        rotor_dy = 0.2125; // [m]
        rotor_Ct = 2.009e-5;
        rotor_Cm = 5.732e-7;
        rotor_Kmt = rotor_Cm / rotor_Ct; 
        // allocation_matrix << -1, -1, -1, -1, 
        //     -rotor_dy, rotor_dy, rotor_dy, -rotor_dy,
        //     rotor_dx, -rotor_dx, rotor_dx, -rotor_dx,
        //     rotor_Kmt, rotor_Kmt, -rotor_Kmt, -rotor_Kmt;
        // inversed_allocation_matrix = allocation_matrix.inverse();
        
        // motor
        motor_nolinear = true;
        motor_k0 = -32.5;
        motor_k1 = 1269.0;
        motor_k2 = -387.3;
        motorSpeed_max = motor_k0 + motor_k1 + motor_k2;
        single_T_max = rotor_Ct * pow(motorSpeed_max,2);
        thrust_max = 4 * single_T_max * 1.0;
        torque_max = 5.0;
        
        // aero dynamic
        rho = 1.22;
        S = 0.155;
        c_avr = 0.17;

        C_d0 = 0.1;
        C_y0 = 0.0;
        C_L = 1.2*2;
        C_dx = C_d0 * cos(kappa) * cos(kappa) + (C_L + C_d0) * sin(kappa) * sin(kappa);
        C_dz = C_d0 * sin(kappa) * sin(kappa) + (C_L + C_d0) * cos(kappa) * cos(kappa);
        C_dxz = C_L * sin(kappa) * cos(kappa);
        C_m0 = 0.144;
        C_m = -0.06543;
        C_l = 0.0;
        C_n = 0.0;
        ROS_INFO("Model parameters ready!");
    }

    // model parameters
    // body
    double kappa; // lift wing install angle [rad]
    double lwq_mass;
    double lwq_Jxx, lwq_Jyy, lwq_Jzz;
    double gravity;
    // aero dynamic
    double rho; // air density
    double S; // wing Size [m^2]
    double c_avr;

    double C_d0; // zero lifting drag param
    double C_y0; // zero lifting side param
    double C_L; // lift param
    double C_dx;
    double C_dz;
    double C_dxz;
    double C_m; // pitch torque param
    double C_m0;
    double C_l; // roll torque param
    double C_n; // yaw torque param

    // rotor
    // (3)    (1)
    //     \/
    //     /\
    // (2)    (4)
    double rotor_dx;
    double rotor_dy;
    double rotor_Ct;
    double rotor_Cm;
    double rotor_Kmt;
    // motor
    // 1) Nolinear motor model
    // motorSpeed = (thrust / rotor_Ct)^0.5
    // motorSpeed = k2 * throttle^2 + k1 * throttle + k0
    bool motor_nolinear;
    double motor_k2;
    double motor_k1;
    double motor_k0;
    double motorSpeed_max;
    double single_T_max;
    double thrust_max;
    double torque_max;
    // // control allocation
    // Eigen::Matrix4d allocation_matrix;
    // Eigen::Matrix4d inversed_allocation_matrix;
    // Eigen::Vector2d aa;
};

} // namespace lwq_flatness
