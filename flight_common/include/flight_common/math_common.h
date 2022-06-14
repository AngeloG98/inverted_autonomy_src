// math common from ethz quadrotor_common
#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>

namespace flight_common
{

double degToRad(const double deg);
double radToDeg(const double rad);

Eigen::Vector3d quaternionToEulerAnglesZYX(const Eigen::Quaterniond& q);
Eigen::Quaterniond eulerAnglesZYXToQuaternion(
    const Eigen::Vector3d& euler_angles);
Eigen::Vector3d rotationMatrixToEulerAnglesZYX(const Eigen::Matrix3d& R);
Eigen::Matrix3d eulerAnglesZYXToRotationMatrix(
    const Eigen::Vector3d& euler_angles);
Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Quaterniond& q);
Eigen::Quaterniond RotationMatrixToQuaternion(const Eigen::Matrix3d& R);

template <typename T>
void limit_min(T& a, T min)
{
    if(a < min)
        a = min;
}

template <typename T>
void limit_max(T& a, T max)
{
    if(a > max)
        a = max;
}

template <typename T>
void limit_positive(T& a, T max)
{
    if(a > max)
        a = max;
    if(a < 0)
        a = 0;
}

template <typename T>
void limit_negative(T& a, T max)
{
    if(a > 0)
        a = max;
    if(a < -max)
        a = -max;
}

template <typename T>
void limit(T& a, T max)
{
    if(a > max)
        a = max;
    if(a < -max)
        a = -max;
}

template <typename T>
void limit(T& a, T& b, T max)
{
    double c = sqrt(a * a + b * b);
    if (c>max)
    {
        a = a / c * max;
        b = b / c * max;
    }
}

template <typename T>
void limit(T& a, T& b, T& c, T max)
{
    double d = sqrt(a * a + b * b + c * c);
    if (d>max)
    {
        a = a / d * max;
        b = b / d * max;
        c = c / d * max;
    }
}

} // flight_common

