//
// Created by shiboz on 2021-02-06.
//

#ifndef ARISE_SLAM_MID360_POSE_LOCAL_PARAMETERIZATION_H
#define ARISE_SLAM_MID360_POSE_LOCAL_PARAMETERIZATION_H


#include <eigen3/Eigen/Dense>
#include "arise_slam_mid360/utils/ceres_compat.h"
#include "../../utils/utility.h"
class PoseLocalParameterization :
#if ARISE_SLAM_HAS_CERES_MANIFOLD
    public ceres::Manifold
#else
    public ceres::LocalParameterization
#endif
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;
#if ARISE_SLAM_HAS_CERES_MANIFOLD
    virtual bool PlusJacobian(const double *x, double *jacobian) const override;
    virtual int AmbientSize() const override;
    virtual int TangentSize() const override;
    virtual bool Minus(const double *y, const double *x, double *y_minus_x) const override;
    virtual bool MinusJacobian(const double* x, double* jacobian) const override;
#else
    virtual bool ComputeJacobian(const double *x, double *jacobian) const override;
    virtual int GlobalSize() const override;
    virtual int LocalSize() const override;
#endif
};






#endif //ARISE_SLAM_MID360_POSE_LOCAL_PARAMETERIZATION_H
