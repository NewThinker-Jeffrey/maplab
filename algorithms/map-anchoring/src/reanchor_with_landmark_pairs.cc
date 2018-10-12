#include "map-anchoring/reanchor_with_landmark_pairs.h"

#include <ceres/ceres.h>
#include <gflags/gflags.h>

#include <ceres-error-terms/parameterization/quaternion-param-eigen.h>
#include <ceres-error-terms/point_pair_under_transformation_error-term.h>
#include <ceres-error-terms/problem-information.h>
#include <maplab-common/threading-helpers.h>

DEFINE_double(
    reanchor_default_landmark_uncertainty, 0.3,
    "default uncertainty of the landmarks' positions");

DEFINE_int32(reanchor_num_iterations, 2000, "Max. number of iterations.");

DEFINE_bool(reanchor_use_jacobi_scaling, true, "Use jacobin scaling.");

DEFINE_bool(reanchor_use_cgnr_linear_solver, false, "Use CGNR linear solver?");

namespace {

ceres::Solver::Options initSolverOptionsFromFlags() {
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = FLAGS_reanchor_num_iterations;
  options.function_tolerance = 1e-12;
  options.gradient_tolerance = 1e-10;
  options.parameter_tolerance = 1e-8;
  options.num_threads = common::getNumHardwareThreads();
  options.num_linear_solver_threads = common::getNumHardwareThreads();
  options.jacobi_scaling = FLAGS_reanchor_use_jacobi_scaling;
  options.initial_trust_region_radius = 1e5;
  options.max_trust_region_radius = 1e20;

  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  if (FLAGS_reanchor_use_cgnr_linear_solver) {
    options.linear_solver_type = ceres::CGNR;
  } else {
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    // options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  }

  options.sparse_linear_algebra_library_type =
      ceres::SUITE_SPARSE;  // ceres::EIGEN_SPARSE

  // Do not copy the data back at every iteration. Remember to turn it on if
  // any callbacks consuming this data are added later.
  options.update_state_every_iteration = false;
  return options;
}

void calcAndPrintError(
    const Eigen::Isometry3d& transformation,
    const std::vector<Eigen::Vector3d>& src_point_group,
    const std::vector<Eigen::Vector3d>& dst_point_group,
    bool skipOutlier = true) {
  double err = 0;
  for (int i = 0; i < src_point_group.size(); i++) {
    Eigen::Vector3d p1 = transformation * src_point_group[i];
    Eigen::Vector3d p2 = dst_point_group[i];
    Eigen::Vector3d errVec = p1 - p2;
    double curErr = errVec.dot(errVec);
    if (curErr > 50.0 * 50.0 && skipOutlier) {
      LOG(INFO) << "Outlier " << i << ": p1(" << p1[0] << ", " << p1[1] << ", "
                << p1[2] << "), p2(" << p2[0] << ", " << p2[1] << ", " << p2[2]
                << ")";
      continue;
    }
    err += curErr;
  }
  LOG(INFO) << "total L2 " << err << ", average L2 "
            << err / src_point_group.size() << ", average err "
            << sqrt(err / src_point_group.size());
}

void calcAndPrintError(
    const Eigen::MatrixXd& RT,
    const std::vector<Eigen::Vector3d>& src_point_group,
    const std::vector<Eigen::Vector3d>& dst_point_group,
    bool skipOutlier = true) {
  Eigen::Isometry3d transformation = Eigen::Isometry3d::Identity();
  transformation.linear() = RT.block<3, 3>(0, 0);
  transformation.translation() = RT.col(3);
  calcAndPrintError(
      transformation, src_point_group, dst_point_group, skipOutlier);
}

void LOG_TRANSFORMATION(
    const Eigen::Isometry3d* pTransformation,
    const Eigen::Quaterniond* pQ = nullptr,
    const Eigen::Vector3d* pP = nullptr) {
  if (pTransformation) {
    LOG(INFO) << "Isometry3d:";
    LOG(INFO) << pTransformation->matrix().format(Eigen::IOFormat());
  }

  if (pQ) {
    LOG(INFO) << "Quaternion:";
    LOG(INFO) << pQ->coeffs().transpose().format(Eigen::IOFormat());
  }

  if (pP) {
    LOG(INFO) << "Translation:";
    LOG(INFO) << pP->transpose().format(Eigen::IOFormat());
  }
}

Eigen::Isometry3d preCalculate(
    const std::vector<Eigen::Vector3d>& src_point_group,
    const std::vector<Eigen::Vector3d>& dst_point_group) {
  // CHECK_EQ(src_point_group.size()==dst_point_group.size());

  int cols = src_point_group.size();
  Eigen::MatrixXd P1(4, cols), P2(3, cols);
  for (int i = 0; i < cols; i++) {
    auto& p1 = src_point_group[i];
    auto& p2 = dst_point_group[i];
    P1.col(i) = Eigen::Vector4d(p1[0], p1[1], p1[2], 1);
    P2.col(i) = Eigen::Vector3d(p2[0], p2[1], p2[2]);
  }
  Eigen::Matrix<double, 3, 4> RT;

  bool debug = true;
  if (debug) {
    Eigen::MatrixXd P2_x_P1t = P2 * P1.transpose();

    LOG(INFO) << "calc P1_x_P1t";
    Eigen::MatrixXd P1_x_P1t = P1 * P1.transpose();
    LOG(INFO) << P1_x_P1t.format(Eigen::IOFormat());

    LOG(INFO) << "calc inv_P1_x_P1t";
    Eigen::MatrixXd inv_P1_x_P1t = P1_x_P1t.inverse();
    LOG(INFO) << inv_P1_x_P1t.format(Eigen::IOFormat());

    // To see if the inverse gets sigular.
    // We know inv_P1_x_P1t * P1_x_P1t should be Identity approximately.
    LOG(INFO) << "calc inv_P1_x_P1t * P1_x_P1t for test";
    LOG(INFO) << (inv_P1_x_P1t * P1_x_P1t).format(Eigen::IOFormat());

    RT = P2_x_P1t * inv_P1_x_P1t;
  } else {
    RT = P2 * P1.transpose() * (P1 * P1.transpose()).inverse();
  }

  Eigen::Isometry3d ret = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond q(RT.block<3, 3>(0, 0));
  q.normalize();
  ret.linear() = q.toRotationMatrix();
  ret.translation() = RT.col(3);

  LOG(INFO)
      << "Unregularized transformation calculated through pure linear-algebra:";
  LOG(INFO) << RT.format(Eigen::IOFormat());
  LOG(INFO) << "average err(rotation part not regularized): ";
  calcAndPrintError(RT, src_point_group, dst_point_group);
  LOG(INFO)
      << "Regularized transformation calculated through pure linear-algebra:";
  LOG_TRANSFORMATION(&ret);
  LOG(INFO) << "average err(with rotation part regularized): ";
  calcAndPrintError(ret, src_point_group, dst_point_group);

  // Currently the pure linear-algebra MSE solver doesn't work because
  // there are drastic outliers in the landmark-pairs. As a workaround
  // an Identity is returned here. It's kind of risky because without a
  // good initial estimation, the ceres-solver may not always converge.
  // Fixing this with RANSAC later on is being considered.
  return Eigen::Isometry3d::Identity();

  // return ret;
}

Eigen::Isometry3d calculate(
    const std::vector<Eigen::Vector3d>& src_point_group,
    const std::vector<Eigen::Vector3d>& dst_point_group) {
  // CHECK_EQ(src_point_group.size()==dst_point_group.size());
  Eigen::Isometry3d prior_transformation =
      preCalculate(src_point_group, dst_point_group);
  Eigen::Quaterniond Q(prior_transformation.linear());
  Eigen::Vector3d T(prior_transformation.translation());
  // return prior_transformation;

  ceres_error_terms::ProblemInformation problem_information;
  double* orientation = Q.coeffs().data();
  double* translation = T.data();

  std::vector<double*> cost_term_args = {orientation, translation};
  double huber_loss_delta = 3.0;
  std::shared_ptr<ceres::LocalParameterization> orientation_parameterization(
      new ceres_error_terms::EigenQuaternionParameterization());

  int point_pairs = src_point_group.size();
  for (int i = 0; i < point_pairs; i++) {
    // todo: maybe we can give different uncertainties to landmarks with
    // different quality.
    double point_uncertainty = FLAGS_reanchor_default_landmark_uncertainty;
    double point_variance = point_uncertainty * point_uncertainty;
    Eigen::Matrix<double, 3, 3> covariance =
        point_variance * Eigen::Matrix3d::Identity();
    std::shared_ptr<ceres::CostFunction> point_pair_term_cost(
        new ceres_error_terms::PointPairUnderTransfromationErrorTerm(
            src_point_group[i], dst_point_group[i], covariance));

    std::shared_ptr<ceres::LossFunction> loss_function(
        new ceres::LossFunctionWrapper(
            new ceres::HuberLoss(huber_loss_delta), ceres::TAKE_OWNERSHIP));

    problem_information.addResidualBlock(
        ceres_error_terms::ResidualType::kInvalid, point_pair_term_cost,
        loss_function,  // set to nullptr to disable loss_function
        cost_term_args);
  }

  problem_information.setParameterization(
      orientation, orientation_parameterization);

  ceres::Solver::Options solver_options = initSolverOptionsFromFlags();
  ceres::Problem problem(
      ceres_error_terms::getDefaultProblemOptions());  // don't take ownership
                                                       // of
  // cost/loss/parameterization.
  ceres_error_terms::buildCeresProblemFromProblemInformation(
      &problem_information, &problem);

  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);
  LOG(INFO) << summary.FullReport();

  Eigen::Isometry3d transformation = Eigen::Isometry3d::Identity();
  transformation.linear() = Q.toRotationMatrix();
  transformation.translation() = T;

  LOG(INFO) << "Transformation calculated through optimization:";
  LOG_TRANSFORMATION(&transformation, &Q, &T);
  LOG(INFO) << "average err: ";
  calcAndPrintError(transformation, src_point_group, dst_point_group);

  return transformation;
}

void test() {
  std::vector<Eigen::Vector3d> src_point_group;
  std::vector<Eigen::Vector3d> dst_point_group;
  std::vector<Eigen::Vector3d> err;
  src_point_group.push_back(Eigen::Vector3d(1.2, 2.3, 1.7));
  err.push_back(Eigen::Vector3d(0.11, -0.17, 0.12));
  src_point_group.push_back(Eigen::Vector3d(2.2, 8.2, -3.7));
  err.push_back(Eigen::Vector3d(0.12, 0.21, -0.22));
  src_point_group.push_back(Eigen::Vector3d(-0.2, 9.2, 3.2));
  err.push_back(Eigen::Vector3d(-0.23, 0.21, 0.17));
  src_point_group.push_back(Eigen::Vector3d(10.2, 4.2, 0));
  err.push_back(Eigen::Vector3d(0.1, -0.01, 0.12));
  src_point_group.push_back(Eigen::Vector3d(-7.2, 5.2, -6.2));
  err.push_back(Eigen::Vector3d(0.01, -0.31, 0.13));
  src_point_group.push_back(Eigen::Vector3d(-8.2, -2.2, 2.2));
  err.push_back(Eigen::Vector3d(0.07, -0.11, 0.33));
  src_point_group.push_back(Eigen::Vector3d(-1.4, -2.8, 7.2));
  err.push_back(Eigen::Vector3d(0.09, -0.14, 0.13));
  Eigen::Quaterniond q(0.5, 0.5, 0.5, 0.5);
  Eigen::Vector3d t(3, 2, 2);
  Eigen::Isometry3d transform;
  transform.linear() = q.toRotationMatrix();
  transform.translation() = t;
  for (int i = 0; i < src_point_group.size(); i++) {
    dst_point_group.push_back(transform * src_point_group[i] + err[i]);
  }

  Eigen::Isometry3d res = calculate(src_point_group, dst_point_group);
  LOG(INFO) << "real Transformation:";
  LOG_TRANSFORMATION(&transform);
  LOG(INFO) << "Transformation calculated:";
  LOG_TRANSFORMATION(&res);
}
}  // namespace

namespace map_anchoring {

void reanchorMissionsWithLandmarkPairs(
    vi_map::VIMap* map, const std::vector<Eigen::Vector3d>& src_G_p,
    const std::vector<Eigen::Vector3d>& dst_G_p) {
  Eigen::Isometry3d T_dstG_srcG = calculate(src_G_p, dst_G_p);
  vi_map::MissionIdSet all_mission_ids;
  map->getAllMissionIds(&all_mission_ids);
  for (auto mission_id : all_mission_ids) {
    vi_map::VIMission& mission = map->getMission(mission_id);
    vi_map::MissionBaseFrame& baseframe =
        map->getMissionBaseFrame(mission.getBaseFrameId());
    Eigen::Isometry3d T_srcG_M, T_dstG_M;
    T_srcG_M.linear() = baseframe.get_q_G_M().toRotationMatrix();
    T_srcG_M.translation() = baseframe.get_p_G_M();
    LOG(INFO) << "---- " << mission_id << ":";
    LOG_TRANSFORMATION(
        nullptr, &(baseframe.get_q_G_M()), &(baseframe.get_p_G_M()));
    T_dstG_M = T_dstG_srcG * T_srcG_M;
    baseframe.set_q_G_M(Eigen::Quaterniond(T_dstG_M.linear()));
    baseframe.set_p_G_M(Eigen::Vector3d(T_dstG_M.translation()));
    LOG(INFO) << "++++ " << mission_id << ":";
    LOG_TRANSFORMATION(
        nullptr, &(baseframe.get_q_G_M()), &(baseframe.get_p_G_M()));
  }
}

}  // namespace map_anchoring
