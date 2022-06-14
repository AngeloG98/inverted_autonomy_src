// math common from ethz quadrotor_common, thanks for opensource :)
#include "flight_common/math_common.h"

namespace flight_common
{

double degToRad(const double deg)
{
  return 0.017453292519943 * deg;
}

double radToDeg(const double rad)
{
  return 57.295779513082323 * rad;
}

Eigen::Vector3d quaternionToEulerAnglesZYX(const Eigen::Quaterniond& q)
{
  Eigen::Vector3d euler_angles;
  euler_angles(0) = atan2(
      2.0 * q.w() * q.x() + 2.0 * q.y() * q.z(),
      q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
  euler_angles(1) = -asin(2.0 * q.x() * q.z() - 2.0 * q.w() * q.y());
  euler_angles(2) = atan2(
      2.0 * q.w() * q.z() + 2.0 * q.x() * q.y(),
      q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
  return euler_angles;
}

Eigen::Quaterniond eulerAnglesZYXToQuaternion(
    const Eigen::Vector3d& euler_angles)
{
  Eigen::Quaterniond q;
  double r = euler_angles(0) / 2.0;
  double p = euler_angles(1) / 2.0;
  double y = euler_angles(2) / 2.0;
  q.w() = cos(r) * cos(p) * cos(y) + sin(r) * sin(p) * sin(y);
  q.x() = sin(r) * cos(p) * cos(y) - cos(r) * sin(p) * sin(y);
  q.y() = cos(r) * sin(p) * cos(y) + sin(r) * cos(p) * sin(y);
  q.z() = cos(r) * cos(p) * sin(y) - sin(r) * sin(p) * cos(y);
  return q;
}

Eigen::Vector3d rotationMatrixToEulerAnglesZYX(const Eigen::Matrix3d& R)
{
  Eigen::Vector3d euler_angles;
  euler_angles(0) = atan2(R(2, 1), R(2, 2));
  euler_angles(1) = -atan2(R(2, 0),
                           sqrt(pow(R(2, 1), 2.0) + pow(R(2, 2), 2.0)));
  euler_angles(2) = atan2(R(1, 0), R(0, 0));
  return euler_angles;
}

Eigen::Matrix3d eulerAnglesZYXToRotationMatrix(
    const Eigen::Vector3d& euler_angles)
{
  double r = euler_angles(0);
  double p = euler_angles(1);
  double y = euler_angles(2);

  Eigen::Matrix3d R;
  R(0, 0) = cos(y) * cos(p);
  R(1, 0) = sin(y) * cos(p);
  R(2, 0) = -sin(p);

  R(0, 1) = cos(y) * sin(p) * sin(r) - sin(y) * cos(r);
  R(1, 1) = sin(y) * sin(p) * sin(r) + cos(y) * cos(r);
  R(2, 1) = cos(p) * sin(r);

  R(0, 2) = cos(y) * sin(p) * cos(r) + sin(y) * sin(r);
  R(1, 2) = sin(y) * sin(p) * cos(r) - cos(y) * sin(r);
  R(2, 2) = cos(p) * cos(r);

  return R;
}

Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Quaterniond& q)
{
  Eigen::Matrix3d R;

  R(0, 0) = q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z();
  R(1, 0) = 2.0 * q.w() * q.z() + 2.0 * q.x() * q.y();
  R(2, 0) = 2.0 * q.x() * q.z() - 2.0 * q.w() * q.y();

  R(0, 1) = 2.0 * q.x() * q.y() - 2.0 * q.w() * q.z();
  R(1, 1) = q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z();
  R(2, 1) = 2.0 * q.w() * q.x() + 2.0 * q.y() * q.z();

  R(0, 2) = 2.0 * q.w() * q.y() + 2.0 * q.x() * q.z();
  R(1, 2) = 2.0 * q.y() * q.z() - 2.0 * q.w() * q.x();
  R(2, 2) = q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z();

  return R;
}

Eigen::Quaterniond RotationMatrixToQuaternion(const Eigen::Matrix3d& R)
{
  return Eigen::Quaterniond(R);
}

} // flight_common
