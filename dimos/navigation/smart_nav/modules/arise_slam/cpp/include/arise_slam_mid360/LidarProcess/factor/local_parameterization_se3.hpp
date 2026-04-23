#ifndef SOPHUS_TEST_LOCAL_PARAMETERIZATION_SE3_HPP
#define SOPHUS_TEST_LOCAL_PARAMETERIZATION_SE3_HPP

#include <sophus/se3.hpp>
#include "arise_slam_mid360/utils/ceres_compat.h"

namespace Sophus {
namespace test {

class LocalParameterizationSE3 :
#if ARISE_SLAM_HAS_CERES_MANIFOLD
    public ceres::Manifold {
#else
    public ceres::LocalParameterization {
#endif
 public:
  virtual ~LocalParameterizationSE3() {}

  // SE3 plus operation for Ceres
  //
  //  T * exp(x)
  //
  virtual bool Plus(double const* T_raw, double const* delta_raw,
                    double* T_plus_delta_raw) const override {
    Eigen::Map<SE3d const> const T(T_raw);
    Eigen::Map<Vector6d const> const delta(delta_raw);
    Eigen::Map<SE3d> T_plus_delta(T_plus_delta_raw);
    T_plus_delta = T * SE3d::exp(delta);
    return true;
  }

  // Jacobian of SE3 plus operation for Ceres
  //
  // Dx T * exp(x)  with  x=0
  //
#if ARISE_SLAM_HAS_CERES_MANIFOLD
  virtual bool PlusJacobian(double const* T_raw,
                            double* jacobian_raw) const override {
    Eigen::Map<SE3d const> T(T_raw);
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> jacobian(
        jacobian_raw);
    jacobian = T.Dx_this_mul_exp_x_at_0();
    return true;
  }

  virtual bool Minus(double const* T_plus_delta_raw, double const* T_raw,
                     double* delta_raw) const override {
    Eigen::Map<SE3d const> const T_plus_delta(T_plus_delta_raw);
    Eigen::Map<SE3d const> const T(T_raw);
    Eigen::Map<Vector6d> delta(delta_raw);
    delta = (T.inverse() * T_plus_delta).log();
    return true;
  }

  virtual bool MinusJacobian(double const* T_raw,
                             double* jacobian_raw) const override {
    Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobian(
        jacobian_raw);
    jacobian.setZero();
    jacobian.block<3, 3>(0, 4).setIdentity();
    jacobian.block<3, 3>(3, 0) = 2.0 * Eigen::Matrix3d::Identity();
    return true;
  }

  virtual int AmbientSize() const override { return SE3d::num_parameters; }

  virtual int TangentSize() const override { return SE3d::DoF; }
#else
  virtual bool ComputeJacobian(double const* T_raw,
                               double* jacobian_raw) const override {
    Eigen::Map<SE3d const> T(T_raw);
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> jacobian(
        jacobian_raw);
    jacobian = T.Dx_this_mul_exp_x_at_0();
    return true;
  }

  virtual int GlobalSize() const override { return SE3d::num_parameters; }

  virtual int LocalSize() const override { return SE3d::DoF; }
#endif
};
}  // namespace test
}  // namespace Sophus

#endif
