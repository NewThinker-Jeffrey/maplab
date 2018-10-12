#ifndef MAP_ANCHORING_DETECT_LANDMARK_PAIRS_H_
#define MAP_ANCHORING_DETECT_LANDMARK_PAIRS_H_

#include <utility>
#include <vector>

#include <localization-summary-map/localization-summary-map.h>
#include <vi-map/vi-map.h>

namespace map_anchoring {

std::vector<std::pair<vi_map::LandmarkId, vi_map::LandmarkId>>
detectLandmarkIdPairs(
    const vi_map::VIMap& query_map, vi_map::VIMap* base_vimap,
    const summary_map::LocalizationSummaryMap* base_summary_map = nullptr);

size_t detectLandmarkPositionPairs(
    const vi_map::VIMap& query_map,
    const summary_map::LocalizationSummaryMap& base_summary_map,
    std::vector<Eigen::Vector3d>* query_landmark_positions,
    std::vector<Eigen::Vector3d>* base_landmark_positions);

size_t detectLandmarkPositionPairs(
    const vi_map::VIMap& query_map, vi_map::VIMap* base_vimap,
    std::vector<Eigen::Vector3d>* query_landmark_positions,
    std::vector<Eigen::Vector3d>* base_landmark_positions);

}  // namespace map_anchoring

#endif  // MAP_ANCHORING_DETECT_LANDMARK_PAIRS_H_
