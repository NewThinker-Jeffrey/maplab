#include <localization-summary-map/localization-summary-map.h>
#include <map-anchoring/detect_landmark_pairs.h>
#include <map-anchoring/reanchor_with_landmark_pairs.h>
#include <map-manager/map-manager.h>
#include <vi-map/landmark-quality-metrics.h>
#include <vi-map/vi-map-serialization.h>
#include <vi-map/vi-map.h>

DEFINE_string(
    another_vimap, "", "The vi-map you want to align the selected map to.");

DEFINE_string(
    another_summary_map, "",
    "The summary map you want to align the selected map to. "
    "If the --another_vimap flag has been specified, this flag would be "
    "omitted.");

namespace map_anchoring_plugin {

bool alignLandmarksToAnotherVIMap(std::string selected_map_key) {
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  vi_map::VIMap another_map;
  if (FLAGS_another_vimap != "") {
    if (!vi_map::serialization::loadMapFromFolder(
            FLAGS_another_vimap, &another_map)) {
      LOG(WARNING) << "Loading VI map failed: " << FLAGS_another_vimap;
      return false;
    }
  } else {
    return false;
  }

  vi_map::LandmarkIdSet another_landmark_ids;
  another_map.getAllLandmarkIds(&another_landmark_ids);
  vi_map::LandmarkIdSet selected_landmark_ids;
  map.get()->getAllLandmarkIds(&selected_landmark_ids);
  std::vector<Eigen::Vector3d> P_src, P_dst;

  for (auto landmark_id : another_landmark_ids) {
    if (!vi_map::isLandmarkWellConstrained(
            another_map, another_map.getLandmark(landmark_id)) ||
        selected_landmark_ids.count(landmark_id) == 0 ||
        !vi_map::isLandmarkWellConstrained(
            *(map.get()), map.get()->getLandmark(landmark_id))) {
      continue;
    }

    P_src.push_back(map.get()->getLandmark_G_p_fi(landmark_id));
    P_dst.push_back(another_map.getLandmark_G_p_fi(landmark_id));
  }

  LOG(INFO) << "Find " << P_src.size() << " original landmark pairs";
  if (P_src.size() < 0.1 * another_landmark_ids.size()) {
    LOG(INFO) << "Re-detect landmark pairs";
    map_anchoring::detectLandmarkPositionPairs(
        *(map.get()), &another_map, &P_src, &P_dst);
  }

  LOG(INFO) << "reanchor missions with " << P_src.size() << " landmark pairs";
  map_anchoring::reanchorMissionsWithLandmarkPairs(map.get(), P_src, P_dst);
  return true;
}

bool alignLandmarksToAnotherSummaryMap(std::string selected_map_key) {
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  std::unique_ptr<summary_map::LocalizationSummaryMap> another_map_ptr;
  bool has_landmark_id = false;
  if (FLAGS_another_summary_map != "") {
    another_map_ptr.reset(new summary_map::LocalizationSummaryMap);
    if (!another_map_ptr->loadFromFolder(
            FLAGS_another_summary_map, &has_landmark_id)) {
      LOG(WARNING) << "Could not load a localization summary map from "
                   << FLAGS_another_summary_map;
      return false;
    }
  } else {
    return false;
  }

  if (!has_landmark_id) {
    LOG(WARNING)
        << "The specified summary map doesn't record landmarks' IDs in it. "
           "Reanchoring a vi-map to such a summary map is not supported "
           "because "
           "there's no clue to find connections between the landmarks in these "
           "two maps";
    return false;
  }

  summary_map::LocalizationSummaryMap& another_map = *(another_map_ptr.get());

  vi_map::LandmarkIdList another_landmark_ids;
  another_map.getAllLandmarkIds(&another_landmark_ids);
  vi_map::LandmarkIdSet selected_landmark_ids;
  map.get()->getAllLandmarkIds(&selected_landmark_ids);
  std::vector<Eigen::Vector3d> P_src, P_dst;
  for (auto landmark_id : another_landmark_ids) {
    if (selected_landmark_ids.count(landmark_id) == 0 ||
        !vi_map::isLandmarkWellConstrained(
            *(map.get()), map.get()->getLandmark(landmark_id))) {
      continue;
    }
    P_src.push_back(map.get()->getLandmark_G_p_fi(landmark_id));
    P_dst.push_back(another_map.getGLandmarkPosition(landmark_id));
  }

  LOG(INFO) << "Find " << P_src.size() << " original landmark pairs";
  if (P_src.size() < 0.1 * another_landmark_ids.size()) {
    LOG(INFO) << "Re-detect landmark pairs";
    map_anchoring::detectLandmarkPositionPairs(
        *(map.get()), *another_map_ptr, &P_src, &P_dst);
  }

  LOG(INFO) << "reanchor missions with " << P_src.size() << " landmark pairs";
  map_anchoring::reanchorMissionsWithLandmarkPairs(map.get(), P_src, P_dst);
  return true;
}
}  // namespace map_anchoring_plugin
