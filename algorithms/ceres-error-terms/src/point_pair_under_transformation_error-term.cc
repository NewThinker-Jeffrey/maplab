#include "ceres-error-terms/point_pair_under_transformation_error-term.h"

#include <ceres/ceres.h>

#include <ceres-error-terms/parameterization/quaternion-param-eigen.h>
#include <maplab-common/geometry.h>

namespace ceres_error_terms {

bool PointPairUnderTransfromationErrorTerm::Evaluate(
    double const* const* parameters, double* residuals,
    double** jacobians) const {
  CHECK_NOTNULL(parameters);
  CHECK_NOTNULL(residuals);

  Eigen::Map<const Eigen::Quaterniond> orientation_current(
      parameters[kIdxOrientation]);
  Eigen::Map<const Eigen::Vector3d> translation_current(
      parameters[kIdxTranslation]);
  // Calculate residuals.
  Eigen::Map<Eigen::Vector3d> residual_vector(residuals);

  Eigen::Vector3d rotated_position =
      // orientation_current * m_position_src;// <---- maybe problematic??
      orientation_current.toRotationMatrix() * m_position_src;

  residual_vector = rotated_position + translation_current - m_position_dst;
  if (jacobians) {
    // Jacobian w.r.t. current pose.
    if (jacobians[kIdxOrientation]) {
      Eigen::Map<OrientationJacobian> J(jacobians[kIdxOrientation]);

      // Jacobian w.r.t elements of quaternion parameterization
      EigenQuaternionParameterization::LiftJacobian lift_jacobian;
      EigenQuaternionParameterization parameterization;
      parameterization.ComputeLiftJacobian(
          orientation_current.coeffs().data(), lift_jacobian.data());

      J.setZero();
      J.block<3, 4>(0, 0) = -common::skew(rotated_position) * lift_jacobian;
      // Add the weighting according to the square root of information matrix.
      J = sqrt_information_matrix_ * J;
    }

    if (jacobians[kIdxTranslation]) {
      Eigen::Map<TranslationJacobian> J(jacobians[kIdxTranslation]);
      J.setZero();
      J.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
      // Add the weighting according to the square root of information matrix.
      J = sqrt_information_matrix_ * J;
    }
  }
  return true;
}
}  // namespace ceres_error_terms
