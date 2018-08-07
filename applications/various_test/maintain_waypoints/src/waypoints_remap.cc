#include "maintain_waypoints/waypoints_remap.h"

#include <fstream>
#include <iostream>
#include <random>
#include <sstream>

#define BOUND_COMPUTATION

namespace waypoints {

namespace {
double distanceBetween(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
  return (p1 - p2).norm();
}

template <class VertexID_Container>
void get_T_G_Is(
    const vi_map::VIMap* vimap, const VertexID_Container& query_ids,
    std::vector<pose::Transformation>* T_G_Is) {
  T_G_Is->clear();
  T_G_Is->reserve(query_ids.size());
  for (auto id : query_ids) {
    T_G_Is->push_back(vimap->getVertex_T_G_I(id));
  }
}

void calculateTransformations(
    const std::vector<pose::Transformation>& old_T_G_Is,
    const std::vector<pose::Transformation>& new_T_G_Is,
    aslam::TransformationVector* T_N_Os) {
  // CHECK_EQ(old_T_G_Is.size(), new_T_G_Is.size());
  T_N_Os->clear();
  T_N_Os->reserve(old_T_G_Is.size());
  for (int i = 0; i < old_T_G_Is.size(); i++) {
    pose::Transformation old_T_G_I = old_T_G_Is[i];
    pose::Transformation new_T_G_I = new_T_G_Is[i];
    pose::Transformation transformation = new_T_G_I * old_T_G_I.inverse();
    T_N_Os->push_back(transformation);
  }
}

Eigen::Vector3d remapWaypoint(
    const Eigen::Vector3d& old_waypoint, const pose::Transformation& T_N_O) {
  Eigen::Vector3d old_p = old_waypoint;
  old_p[2] = 0;
  Eigen::Vector3d old_dir;
  old_dir[0] = cos(old_waypoint[2]);
  old_dir[1] = sin(old_waypoint[2]);
  old_dir[2] = 0;
  Eigen::Vector3d new_p = T_N_O * old_p;
  Eigen::Vector3d new_dir = T_N_O * old_dir;
  Eigen::Vector3d new_waypoint(
      new_p[0], new_p[1], atan2(new_dir[1], new_dir[0]));
  return new_waypoint;
}

void remapWaypoints(
    const std::vector<Eigen::Vector3d>& old_waypoints,
    const aslam::TransformationVector& T_N_Os,
    std::vector<Eigen::Vector3d>* new_waypoints) {
  // CHECK_EQ(old_waypoints.size(), T_N_Os.size());
  new_waypoints->clear();
  new_waypoints->reserve(old_waypoints.size());

  for (int i = 0; i < old_waypoints.size(); i++) {
    const pose::Transformation& T_N_O = T_N_Os[i];
    const Eigen::Vector3d& old_waypoint = old_waypoints[i];
    Eigen::Vector3d new_waypoint = remapWaypoint(old_waypoint, T_N_O);
    new_waypoints->push_back(new_waypoint);
  }
}
}  // namespace

WaypointsRemapper::WaypointsRemapper(
    const vi_map::VIMap* old_vimap, const vi_map::VIMap* new_vimap)
    : m_old_vimap(old_vimap), m_new_vimap(new_vimap) {
  // CHECK_NE(m_old_vimap, nullptr);
  // CHECK_NE(m_new_vimap, nullptr);

  initSpatialDatabase();
  m_old_vimap->getAllVertexIds(&m_all_vertex_ids);
}

WaypointsRemapper::~WaypointsRemapper() {
  releaseSpatialDatabase();
}

void WaypointsRemapper::initSpatialDatabase() {
  Eigen::Vector3d grid_cell_size(0.5, 0.5, 2.0);
  m_spatial_database =
      new vi_map_helpers::SpatialDatabase<pose_graph::VertexId>(
          *m_old_vimap, grid_cell_size);
}

void WaypointsRemapper::releaseSpatialDatabase() {
  delete m_spatial_database;
}

pose_graph::VertexId WaypointsRemapper::getNearestVertexForWaypoint(
    const Eigen::Vector3d& waypoint) {
  Eigen::Vector3d p = waypoint;
  p[2] = 0;
  pose_graph::VertexId nearest_id;
  double min_dis = -1;
  for (auto vertexId : m_all_vertex_ids) {
    double dis = distanceBetween(m_old_vimap->getVertex_G_p_I(vertexId), p);
    if (min_dis < 0 || dis < min_dis) {
      min_dis = dis;
      nearest_id = vertexId;
    }
  }
  return nearest_id;
}

Eigen::Vector3d WaypointsRemapper::remapWaypointToNewMap(
    const Eigen::Vector3d& old_waypoint) {
  static const int max_neiboures = 30;
  Eigen::Vector3d old_p = old_waypoint;
  old_p[2] = 0;

  // find neighbouring vertices
  constexpr double kRadiusMeters = 2.0;
  pose_graph::VertexIdSet neigboring_observers;
  m_spatial_database->getObjectIdsInRadius(
      old_p, kRadiusMeters, &neigboring_observers);

  if (neigboring_observers.size() < 2) {
    pose_graph::VertexId nearest_id;
    if (neigboring_observers.size() == 0) {
      std::cout << "There is No neigboring observers found, use the nearest "
                   "one for work around"
                << std::endl;
      nearest_id = getNearestVertexForWaypoint(old_waypoint);
    } else {
      std::cout << "There is only one neigboring observer found" << std::endl;
      nearest_id = *neigboring_observers.begin();
    }
    pose::Transformation old_T_G_I = m_old_vimap->getVertex_T_G_I(nearest_id);
    pose::Transformation new_T_G_I = m_new_vimap->getVertex_T_G_I(nearest_id);
    pose::Transformation T_N_O = new_T_G_I * old_T_G_I.inverse();
    return remapWaypoint(old_waypoint, T_N_O);
  }

#ifdef BOUND_COMPUTATION
  if (neigboring_observers.size() > max_neiboures) {
    pose_graph::VertexIdSet tmp_neigboring_observers;
    tmp_neigboring_observers.clear();
    auto iter = neigboring_observers.begin();
    for (int i = 0; i < max_neiboures; i++, iter++) {
      tmp_neigboring_observers.insert(*iter);
    }
    neigboring_observers.swap(tmp_neigboring_observers);
  }
// CHECK_LE(neigboring_observers.size(), max_neiboures);
#endif

  // calc transformation matrix between old frame and new frame.
  std::vector<pose::Transformation> old_T_G_Is, new_T_G_Is;
  aslam::TransformationVector T_N_Os;
  get_T_G_Is(m_old_vimap, neigboring_observers, &old_T_G_Is);
  get_T_G_Is(m_new_vimap, neigboring_observers, &new_T_G_Is);
  calculateTransformations(old_T_G_Is, new_T_G_Is, &T_N_Os);

  // calc the best transformation.
  // todo: maybe ceres-solver works better than ransac.
  constexpr int kNumRansacIterations = 100;              // 2000
  constexpr double kPositionErrorThresholdMeters = 0.2;  // 2
  constexpr double kOrientationErrorThresholdRadians =
      0.0174 * 5.0;  // 0.174=10 deg.
  std::random_device device;
  const int ransac_seed = device();
  int num_inliers = 0;
  pose::Transformation T_N_O_LS;
  common::transformationRansac(
      T_N_Os, kNumRansacIterations, kOrientationErrorThresholdRadians,
      kPositionErrorThresholdMeters, ransac_seed, &T_N_O_LS, &num_inliers);
  // todo: check the number of inliers here.
  std::cout << "Inliers " << num_inliers << ", Total " << T_N_Os.size()
            << std::endl;

  // here we come to the last stage, trasform the waypoint
  return remapWaypoint(old_waypoint, T_N_O_LS);
}

void WaypointsRemapper::remapWaypointsToNewMap(
    const std::vector<Eigen::Vector3d>& old_waypoints,
    std::vector<Eigen::Vector3d>* new_waypoints) {
  new_waypoints->clear();
  new_waypoints->reserve(old_waypoints.size());
  for (auto old_waypoint : old_waypoints) {
    new_waypoints->push_back(remapWaypointToNewMap(old_waypoint));
  }
}
}  // namespace waypoints
