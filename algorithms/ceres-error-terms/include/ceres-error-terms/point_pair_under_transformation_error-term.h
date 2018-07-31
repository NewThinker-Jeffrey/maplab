#ifndef CERES_ERROR_TERMS_POINT_PAIR_UNDER_TRANSFORMATION_ERROR_TERM_H_
#define CERES_ERROR_TERMS_POINT_PAIR_UNDER_TRANSFORMATION_ERROR_TERM_H_

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <ceres/sized_cost_function.h>

#include <ceres-error-terms/common.h>

namespace ceres_error_terms {

class PointPairUnderTransfromationErrorTerm
    : public ceres::SizedCostFunction<positionblocks::kResidualSize,
                                      positionblocks::kOrientationBlockSize,
                                      positionblocks::kPositionBlockSize> {
 public:
  PointPairUnderTransfromationErrorTerm(
      const Eigen::Vector3d& position_src, const Eigen::Vector3d& position_dst,
      const Eigen::Matrix<double, 3, 3>& covariance)
      : m_position_dst(position_dst), m_position_src(position_src) {
    // Getting inverse square root of covariance matrix.
    Eigen::Matrix<double, 3, 3> L = covariance.llt().matrixL();
    sqrt_information_matrix_.setIdentity();
    L.triangularView<Eigen::Lower>().solveInPlace(sqrt_information_matrix_);
  }

  virtual ~PointPairUnderTransfromationErrorTerm() {}

  virtual bool Evaluate(
      double const* const* parameters, double* residuals,
      double** jacobians) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Don't change the ordering of the enum elements, they have to be the
  // same as the order of the parameter blocks.
  enum { kIdxOrientation, kIdxTranslation };

  // The representation for Jacobian computed by this object.
  typedef Eigen::Matrix<double, positionblocks::kResidualSize,
                        positionblocks::kOrientationBlockSize, Eigen::RowMajor>
      OrientationJacobian;
  typedef Eigen::Matrix<double, positionblocks::kResidualSize,
                        positionblocks::kPositionBlockSize, Eigen::RowMajor>
      TranslationJacobian;

  Eigen::Matrix<double, positionblocks::kResidualSize,
                positionblocks::kResidualSize>
      sqrt_information_matrix_;
  Eigen::Vector3d m_position_dst;
  Eigen::Vector3d m_position_src;
};

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_POINT_PAIR_UNDER_TRANSFORMATION_ERROR_TERM_H_
