#include "map-anchoring/detect_landmark_pairs.h"
#include <loop-closure-handler/loop-detector-node.h>

namespace map_anchoring {

std::vector<std::pair<vi_map::LandmarkId, vi_map::LandmarkId>>
detectLandmarkIdPairs(
    const vi_map::VIMap& query_map,
    const summary_map::LocalizationSummaryMap* base_map) {
  std::map<vi_map::LandmarkId, std::set<vi_map::LandmarkId>>
      base_landmark_id_to_query_landmark_ids;
  loop_detector_node::LoopDetectorNode loop_detector_node;
  loop_detector_node.addLocalizationSummaryMapToDatabase(*base_map);
  pose_graph::VertexIdList vertex_ids;
  query_map.getAllVertexIds(&vertex_ids);
  for (pose_graph::VertexId vertex_id : vertex_ids) {
    const vi_map::Vertex& vertex = query_map.getVertex(vertex_id);
    bool skip_untracked_keypoints = false;
    pose::Transformation T_G_I;
    unsigned int num_of_lc_matches;
    vi_map::VertexKeyPointToStructureMatchList inlier_structure_matches;
    bool lc_success = loop_detector_node.findNFrameInSummaryMapDatabase(
        vertex.getVisualNFrame(), skip_untracked_keypoints, *base_map, &T_G_I,
        &num_of_lc_matches, &inlier_structure_matches);
    if (lc_success) {
      for (vi_map::VertexKeyPointToStructureMatch match :
           inlier_structure_matches) {
        vi_map::LandmarkId query_landmark_id = vertex.getObservedLandmarkId(
            match.frame_index_query, match.keypoint_index_query);
        if (!query_landmark_id.isValid()) {
          continue;
        }
        vi_map::LandmarkId base_landmark_id = match.landmark_result;
        auto it = base_landmark_id_to_query_landmark_ids.find(base_landmark_id);
        if (it != base_landmark_id_to_query_landmark_ids.end()) {
          std::set<vi_map::LandmarkId>& query_set = it->second;
          query_set.insert(query_landmark_id);
        } else {
          std::set<vi_map::LandmarkId> query_set;
          query_set.insert(query_landmark_id);
          base_landmark_id_to_query_landmark_ids[base_landmark_id] = query_set;
        }
      }
    }
  }

  std::vector<std::pair<vi_map::LandmarkId, vi_map::LandmarkId>>
      landmark_id_pairs;
  for (auto landmark_set : base_landmark_id_to_query_landmark_ids) {
    vi_map::LandmarkId base_landmark_id = landmark_set.first;
    if (landmark_set.second.size() > 1) {
      std::ostringstream oss;
      oss << "More than 1 landmarks associated with base landmark "
          << base_landmark_id << ": ";
      for (auto query_landmark_id : landmark_set.second) {
        oss << query_landmark_id << ", ";
      }
      LOG(WARNING) << oss.str();
    }

    for (auto query_landmark_id : landmark_set.second) {
      landmark_id_pairs.push_back(
          std::make_pair(base_landmark_id, query_landmark_id));
    }
  }

  LOG(INFO) << "Total " << base_landmark_id_to_query_landmark_ids.size()
            << " base landmarks have their associated query landmarks";
  return landmark_id_pairs;
}

size_t detectLandmarkPositionPairs(
    const vi_map::VIMap& query_map,
    const summary_map::LocalizationSummaryMap& base_map,
    std::vector<Eigen::Vector3d>* query_landmark_positions,
    std::vector<Eigen::Vector3d>* base_landmark_positions) {
  query_landmark_positions->clear();
  base_landmark_positions->clear();
  std::vector<std::pair<vi_map::LandmarkId, vi_map::LandmarkId>>
      landmark_id_pairs = detectLandmarkIdPairs(query_map, &base_map);

  for (auto pair : landmark_id_pairs) {
    vi_map::LandmarkId base_landmark_id = pair.first;
    vi_map::LandmarkId query_landmark_id = pair.second;
    query_landmark_positions->push_back(
        query_map.getLandmark_G_p_fi(query_landmark_id));
    base_landmark_positions->push_back(
        base_map.getGLandmarkPosition(base_landmark_id));
  }

  LOG(INFO) << "Total " << landmark_id_pairs.size() << "landmark pairs";
  return landmark_id_pairs.size();
}

}  // namespace map_anchoring
