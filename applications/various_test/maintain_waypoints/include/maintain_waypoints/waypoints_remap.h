#ifndef MAINTAIN_WAYPOINTS_WAYPOINTS_REMAP_H_
#define MAINTAIN_WAYPOINTS_WAYPOINTS_REMAP_H_

#include <Eigen/Geometry>
#include <vector>
#include <vi-map-helpers/spatial-database.h>
#include <vi-map/vi-map.h>

namespace waypoints {

class WaypointsRemapper {
 public:
  WaypointsRemapper(
      const vi_map::VIMap* old_vimap, const vi_map::VIMap* new_vimap);
  ~WaypointsRemapper();

  Eigen::Vector3d remapWaypointToNewMap(const Eigen::Vector3d& old_waypoint);

  void remapWaypointsToNewMap(
      const std::vector<Eigen::Vector3d>& old_waypoint,
      std::vector<Eigen::Vector3d>* new_waypoints);

 private:
  void initSpatialDatabase();
  void releaseSpatialDatabase();
  pose_graph::VertexId getNearestVertexForWaypoint(
      const Eigen::Vector3d& waypoints);

 private:
  vi_map_helpers::SpatialDatabase<pose_graph::VertexId>* m_spatial_database;
  pose_graph::VertexIdList m_all_vertex_ids;
  const vi_map::VIMap* m_old_vimap;
  const vi_map::VIMap* m_new_vimap;
};
}  // namespace waypoints

#endif  // MAINTAIN_WAYPOINTS_WAYPOINTS_REMAP_H_
