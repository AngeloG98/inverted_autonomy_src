#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

namespace lwq_flatness
{
struct ControlHelper
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ControlHelper()
    {
        position.reset();
        velocity.reset();
        attitude.reset();
        bodyrate.reset();
        freq = 1.0;
    }
    ~ControlHelper() {}
    void setCtrlParams()
    {
        // attitude 
        attitude.kpx = 5.0, attitude.kpy = 5.0, attitude.kpz = 5.5;
        // bodyrate
        bodyrate.kpx = 20.0, bodyrate.kpy = 20.0, bodyrate.kpz = 20.0;
        bodyrate.kix = 1.0, bodyrate.kiy = 1.0, bodyrate.kiz = 1.0;
        bodyrate.kdx = 0.0, bodyrate.kdy = 0.0, bodyrate.kdz = 0.0;
        bodyrate.saturI = 500.0;
        // position
        position.kpx = 0.5, position.kpy = 0.5, position.kpz = 1.0;
        // velocity
        velocity.kpx = 2.0, velocity.kpy = 2.0, velocity.kpz = 6.5;
        // velocity.kpx = 5.0, velocity.kpy = 5.0, velocity.kpz = 6.0;

        // control frequence
        freq = 300.0;
        ROS_INFO("Control parameters ready!");
    }

    // control parameters
    struct PidConfig{
        double kpx, kpy, kpz;
        double kix, kiy, kiz;
        double kdx, kdy, kdz;
        double saturI;
        double saturAll;
        double I_term_x, I_term_y, I_term_z;
        double last_Err_x, last_Err_y, last_Err_z;
        PidConfig() { reset(); }
        void reset(){
            kpx = 1.0, kpy = 1.0, kpz = 1.0;
            kix = 0.0, kix = 0.0, kix = 0.0;
            kdx = 0.0, kdy = 0.0, kdz = 0.0;
            saturI = 0.0;
            saturAll = 0.0;
            I_term_x = 0.0, I_term_y = 0.0, I_term_z = 0.0;
            last_Err_x = 0.0, last_Err_y = 0.0, last_Err_z = 0.0;
        }
    };
    PidConfig position;
    PidConfig velocity;
    PidConfig attitude;
    PidConfig bodyrate;
    double freq;
};

} // namespace lwq_flatness
