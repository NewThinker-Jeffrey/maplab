#ifndef MAP_ANCHORING_DETECT_LANDMARK_PAIRS_H_
#define MAP_ANCHORING_DETECT_LANDMARK_PAIRS_H_

#include <localization-summary-map/localization-summary-map.h>
#include <vi-map/vi-map.h>

namespace map_anchoring {

std::vector<std::pair<vi_map::LandmarkId, vi_map::LandmarkId>>
detectLandmarkIdPairs(
    const vi_map::VIMap& query_map,
    const summary_map::LocalizationSummaryMap* base_map);

size_t detectLandmarkPositionPairs(
    const vi_map::VIMap& query_map,
    const summary_map::LocalizationSummaryMap& base_map,
    std::vector<Eigen::Vector3d>* query_landmark_positions,
    std::vector<Eigen::Vector3d>* base_landmark_positions);

}  // namespace map_anchoring

#endif  // MAP_ANCHORING_DETECT_LANDMARK_PAIRS_H_
