#include <localization-summary-map/localization-summary-map.h>
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

  LOG(INFO) << "reanchor missions with " << P_src.size() << " landmark pairs";
  map_anchoring::reanchorMissionsWithLandmarkPairs(map.get(), P_src, P_dst);
  return true;
}

bool alignLandmarksToAnotherSummaryMap(std::string selected_map_key) {
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  LOG(WARNING)
      << "Currently reanchoring to a summary map is not supported yet, "
         "because the landmarks' IDs in a summary map are mutable.";
  return false;

  std::unique_ptr<summary_map::LocalizationSummaryMap> another_map_ptr;
  if (FLAGS_another_summary_map != "") {
    another_map_ptr.reset(new summary_map::LocalizationSummaryMap);
    if (!another_map_ptr->loadFromFolder(FLAGS_another_summary_map)) {
      LOG(WARNING) << "Could not load a localization summary map from "
                   << FLAGS_another_summary_map;
      return false;
    }
  } else {
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

  LOG(INFO) << "reanchor missions with " << P_src.size() << " landmark pairs";
  map_anchoring::reanchorMissionsWithLandmarkPairs(map.get(), P_src, P_dst);
  return true;
}
}  // namespace map_anchoring_plugin
