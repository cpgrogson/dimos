//
// Created by shibo zhao on 2020-09-27.
//

#ifndef _LIDAR_OPTIMIZATION_ANALYTIC_H_
#define _LIDAR_OPTIMIZATION_ANALYTIC_H_

#include "arise_slam_mid360/utils/ceres_compat.h"
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

void getTransformFromSe3(const Eigen::Matrix<double,6,1>& se3, Eigen::Quaterniond& q, Eigen::Vector3d& t);

Eigen::Matrix3d skew(const Eigen::Vector3d& mat_in);

class EdgeAnalyticCostFunction : public ceres::SizedCostFunction<3, 7> {
	public:

		EdgeAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_, Eigen::Vector3d last_point_b_);
		virtual ~EdgeAnalyticCostFunction() {}
		virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

		Eigen::Vector3d curr_point;
		Eigen::Vector3d last_point_a;
		Eigen::Vector3d last_point_b;
};

class SurfNormAnalyticCostFunction : public ceres::SizedCostFunction<1, 7> {
	public:
		SurfNormAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_, double negative_OA_dot_norm_);
		virtual ~SurfNormAnalyticCostFunction() {}
		virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

		Eigen::Vector3d curr_point;
		Eigen::Vector3d plane_unit_norm;
		double negative_OA_dot_norm;
};

class PoseSE3Parameterization :
#if ARISE_SLAM_HAS_CERES_MANIFOLD
    public ceres::Manifold {
#else
    public ceres::LocalParameterization {
#endif
public:
	
    PoseSE3Parameterization() {}
    virtual ~PoseSE3Parameterization() {}
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const override;
#if ARISE_SLAM_HAS_CERES_MANIFOLD
    virtual bool PlusJacobian(const double* x, double* jacobian) const override;
    virtual int AmbientSize() const override { return 7; }
    virtual int TangentSize() const override { return 6; }
    virtual bool Minus(const double* y, const double* x, double* y_minus_x) const override;
    virtual bool MinusJacobian(const double* x, double* jacobian) const override;
#else
    virtual bool ComputeJacobian(const double* x, double* jacobian) const override;
    virtual int GlobalSize() const override { return 7; }
    virtual int LocalSize() const override { return 6; }
#endif
};



#endif // _LIDAR_OPTIMIZATION_ANALYTIC_H_
