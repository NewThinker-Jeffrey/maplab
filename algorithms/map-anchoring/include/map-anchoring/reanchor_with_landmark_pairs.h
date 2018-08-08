#ifndef MAP_ANCHORING_REANCHOR_WITH_LANDMARK_PAIRS_H_
#define MAP_ANCHORING_REANCHOR_WITH_LANDMARK_PAIRS_H_

#include <Eigen/Core>
#include <vector>

#include <vi-map/vi-map.h>

namespace map_anchoring {

void reanchorMissionsWithLandmarkPairs(
    vi_map::VIMap* map, const std::vector<Eigen::Vector3d>& src_G_p,
    const std::vector<Eigen::Vector3d>& dst_G_p);

}  // namespace map_anchoring

#endif  // MAP_ANCHORING_REANCHOR_WITH_LANDMARK_PAIRS_H_
