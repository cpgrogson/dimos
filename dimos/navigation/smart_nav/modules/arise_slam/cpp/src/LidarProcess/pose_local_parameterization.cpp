//
// Created by shiboz on 2021-02-06.
//

#include "arise_slam_mid360/LidarProcess/factor/pose_local_parameterization.h"

bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

      p = _p + dp;
      q = (_q * dq).normalized();

    return true;
}
#if ARISE_SLAM_HAS_CERES_MANIFOLD
bool PoseLocalParameterization::PlusJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}
int PoseLocalParameterization::AmbientSize() const
{
  return 7;
}
int PoseLocalParameterization::TangentSize() const
{
  return 6;
}

bool PoseLocalParameterization::Minus(const double *y, const double *x, double *y_minus_x) const
{
  Eigen::Map<const Eigen::Vector3d> p_x(x);
  Eigen::Quaterniond q_x = Eigen::Map<const Eigen::Quaterniond>(x + 3);
  Eigen::Map<const Eigen::Vector3d> p_y(y);
  Eigen::Quaterniond q_y = Eigen::Map<const Eigen::Quaterniond>(y + 3);

  q_x.normalize();
  q_y.normalize();

  Eigen::Quaterniond dq = q_x.conjugate() * q_y;
  if (dq.w() < 0.0) {
    dq.coeffs() *= -1.0;
  }
  dq.normalize();

  Eigen::Map<Eigen::Matrix<double, 6, 1>> delta(y_minus_x);
  delta.head<3>() = p_y - p_x;
  delta.tail<3>() = 2.0 * dq.vec();

  return true;
}

bool PoseLocalParameterization::MinusJacobian(const double* x, double* jacobian) const
{
  Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> j(jacobian);
  j.setZero();
  j.block<3, 3>(0, 0).setIdentity();
  j.block<3, 3>(3, 3) = 2.0 * Eigen::Matrix3d::Identity();

  return true;
}
#else
bool PoseLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
  j.topRows<6>().setIdentity();
  j.bottomRows<1>().setZero();

  return true;
}
int PoseLocalParameterization::GlobalSize() const
{
  return 7;
}
int PoseLocalParameterization::LocalSize() const
{
  return 6;
}
#endif
