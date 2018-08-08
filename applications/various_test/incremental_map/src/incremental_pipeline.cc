#include "incremental_map/incremental_pipeline.h"

#include "localization-summary-map/localization-summary-map-creation.h"
#include "map-anchoring/map-anchoring.h"
#include "map-anchoring/reanchor_with_landmark_pairs.h"
#include "map-optimization/vi-map-optimizer.h"
#include "map-optimization/vi-map-relaxation.h"
#include "map-sparsification/keyframe-pruning.h"
#include "vi-map/landmark-quality-metrics.h"
#include "vi-map/vi-map-serialization.h"

namespace incremental_map {

IncrementalPipeline::IncrementalPipeline()
    : m_vimaps(nullptr), m_plotter(nullptr), m_result_map(nullptr) {}

IncrementalPipeline::IncrementalPipeline(
    std::vector<vi_map::VIMap::Ptr>* vimaps,
    visualization::ViwlsGraphRvizPlotter* plotter)
    : m_vimaps(vimaps), m_plotter(plotter), m_result_map(nullptr) {}

IncrementalPipeline::~IncrementalPipeline() {}

void IncrementalPipeline::setVimapsToMerge(
    std::vector<vi_map::VIMap::Ptr>* vimaps) {
  m_vimaps = vimaps;
}

vi_map::VIMap::Ptr IncrementalPipeline::operator()() {
  LOG(INFO) << "---------[Generating base map]---------";
  summary_map::LocalizationSummaryMap::Ptr base_summary_map =
      generateSummaryMap(*(m_vimaps->at(0)));

  // join-all-maps
  LOG(INFO) << "---------[Joining all maps]---------";
  if (!joinAllMaps())
    return nullptr;
  plotResultMap();

  // sbk
  LOG(INFO) << "---------[Set first baseframe known]---------";
  if (!setFirstBaseFrameKnown())
    return nullptr;

  // aam
  LOG(INFO) << "---------[Anchoring all missions]---------";
  if (!anchorAllMissions())
    return nullptr;
  plotResultMap();

  // kfh
  LOG(INFO) << "---------[Keyframing]---------";
  if (!keyframeMapBasedOnHeuristics())
    return nullptr;
  plotResultMap();

  // relax
  LOG(INFO) << "---------[Relax]---------";
  if (!relaxMap()) {
    LOG(WARNING) << "Relax failed. There's no BIG loopclosures found.";
  } else {
    plotResultMap();
  }

  // lc
  LOG(INFO) << "---------[Loopclosure 1]---------";
  if (!findLoopClosuresBetweenAllMissions())
    return nullptr;
  plotResultMap();

  // optvi
  LOG(INFO) << "---------[Optimize visual inertial 1]---------";
  if (!optimizeVisualInertial())
    return nullptr;
  plotResultMap();

  // repeated lc
  LOG(INFO) << "---------[Loopclosure 2]---------";
  if (!findLoopClosuresBetweenAllMissions())
    return nullptr;
  plotResultMap();

  // repeated optvi
  LOG(INFO) << "---------[Optimize visual inertial 2]---------";
  if (!optimizeVisualInertial())
    return nullptr;
  plotResultMap();

  LOG(INFO) << "---------[Align landmarks to base map]---------";
  alignLandmarksToBaseMap(*base_summary_map);
  plotResultMap();

  return m_result_map;
}

void IncrementalPipeline::plotResultMap() {
  if (m_plotter != nullptr) {
    m_plotter->visualizeMap(*m_result_map);
  }
}

void IncrementalPipeline::plotPartitioning(
    const pose_graph::VertexIdList& keyframe_ids) {
  // Optionally, visualize the selected keyframes.
  if (m_plotter != nullptr) {
    std::vector<pose_graph::VertexIdList> partitions;
    partitions.emplace_back(keyframe_ids);
    m_plotter->plotPartitioning(*m_result_map, partitions);
  }
}

bool IncrementalPipeline::joinAllMaps() {
  // Let's check if there are more than one vi-maps in the list.
  if (m_vimaps->size() <= 1u) {
    LOG(ERROR) << "There is no map or only one map in the vimaps_to_merge list."
               << "Joining all maps is aborted as there is nothing to do.";
    return false;
  }

  // Merge all other maps into the first map and delete them.
  for (auto iter = m_vimaps->begin(); iter != m_vimaps->end(); iter++) {
    if (iter == m_vimaps->begin()) {
      m_result_map = *iter;
      continue;
    }
    m_result_map->mergeAllMissionsFromMap(**iter);
    iter->reset();
  }

  return true;
}

bool IncrementalPipeline::setFirstBaseFrameKnown() {
  vi_map::MissionId mission_id = m_result_map->getIdOfFirstMission();
  if (!mission_id.isValid()) {
    LOG(ERROR) << "Mission ID error";
    return false;
  }

  map_anchoring::setMissionBaseframeKnownState(
      mission_id, true, m_result_map.get());
  return true;
}

bool IncrementalPipeline::anchorAllMissions() {
  return map_anchoring::anchorAllMissions(m_result_map.get(), m_plotter);
}

bool IncrementalPipeline::keyframeMapBasedOnHeuristics() {
  vi_map::MissionIdList missions_to_keyframe;
  m_result_map->getAllMissionIds(&missions_to_keyframe);
  map_sparsification::KeyframingHeuristicsOptions options =
      map_sparsification::KeyframingHeuristicsOptions::initializeFromGFlags();

  for (const vi_map::MissionId& mission_id : missions_to_keyframe) {
    VLOG(1) << "Keyframing mission " << mission_id << '.';

    size_t num_initial_vertices =
        m_result_map->numVerticesInMission(mission_id);
    pose_graph::VertexId root_vertex_id =
        m_result_map->getMission(mission_id).getRootVertexId();
    CHECK(root_vertex_id.isValid());
    pose_graph::VertexId last_vertex_id =
        m_result_map->getLastVertexIdOfMission(mission_id);

    // Select keyframes along the mission. Unconditionally add the last vertex
    // as
    // a keyframe if it isn't a keyframe already.
    pose_graph::VertexIdList keyframe_ids;
    map_sparsification::selectKeyframesBasedOnHeuristics(
        *m_result_map, root_vertex_id, last_vertex_id, options, &keyframe_ids);
    if (keyframe_ids.empty()) {
      LOG(WARNING) << "Keyframing of mission " << mission_id
                   << " failed: No keyframes found.";
      continue;
    }

    if (keyframe_ids.back() != last_vertex_id) {
      keyframe_ids.emplace_back(last_vertex_id);
    }

    plotPartitioning(keyframe_ids);
    LOG(INFO) << "Selected " << keyframe_ids.size() << " keyframes of "
              << num_initial_vertices << " vertices.";

    // Remove non-keyframe vertices.
    const size_t num_removed_keyframes =
        map_sparsification::removeVerticesBetweenKeyframes(
            keyframe_ids, m_result_map.get());
    LOG(INFO) << "Removed " << num_removed_keyframes << " vertices of "
              << num_initial_vertices << " vertices.";
  }

  return true;
}

bool IncrementalPipeline::relaxMap() {
  bool kSignalHandlerEnabled = true;
  map_optimization::VIMapRelaxation relaxation(
      m_plotter, kSignalHandlerEnabled);
  vi_map::MissionIdList mission_id_list;
  m_result_map->getAllMissionIds(&mission_id_list);
  return relaxation.relax(mission_id_list, m_result_map.get());
}

bool IncrementalPipeline::findLoopClosuresBetweenAllMissions() {
  vi_map::LandmarkIdList landmark_ids;
  m_result_map->getAllLandmarkIds(&landmark_ids);

  // Check all landmarks to ensure that their quality is not unknown.
  for (const vi_map::LandmarkId& landmark_id : landmark_ids) {
    if (m_result_map->getLandmark(landmark_id).getQuality() ==
        vi_map::Landmark::Quality::kUnknown) {
      LOG(ERROR) << "Some landmarks are of unknown quality";
      return false;
    }
  }

  vi_map::MissionIdList mission_ids;
  m_result_map->getAllMissionIds(&mission_ids);
  return findLoopClosuresBetweenMissions(mission_ids);
}

bool IncrementalPipeline::findLoopClosuresBetweenMissions(
    const vi_map::MissionIdList& mission_ids) {
  for (vi_map::MissionIdList::const_iterator it = mission_ids.begin();
       it != mission_ids.end(); ++it) {
    CHECK(it->isValid());
    loop_detector_node::LoopDetectorNode loop_detector;
    if (m_plotter != nullptr) {
      loop_detector.instantiateVisualizer();
    }
    loop_detector.addMissionToDatabase(*it, *m_result_map);
    for (vi_map::MissionIdList::const_iterator jt = it; jt != mission_ids.end();
         ++jt) {
      loop_detector.detectLoopClosuresAndMergeLandmarks(
          *jt, m_result_map.get());
    }
  }
  return true;
}

bool IncrementalPipeline::optimizeVisualInertial(bool visual_only) {
  vi_map::MissionIdList missions_to_optimize_list;
  m_result_map->getAllMissionIds(&missions_to_optimize_list);

  vi_map::MissionIdSet missions_to_optimize(
      missions_to_optimize_list.begin(), missions_to_optimize_list.end());

  map_optimization::ViProblemOptions options =
      map_optimization::ViProblemOptions::initFromGFlags();

  if (visual_only) {
    options.add_inertial_constraints = false;
  }

  bool kSignalHandlerEnabled = true;
  map_optimization::VIMapOptimizer optimizer(m_plotter, kSignalHandlerEnabled);
  bool success;
  map_optimization::OutlierRejectionSolverOptions outlier_rejection_options =
      map_optimization::OutlierRejectionSolverOptions::initFromFlags();
  success = optimizer.optimizeVisualInertial(
      options, missions_to_optimize, &outlier_rejection_options,
      m_result_map.get());
  return success;
}

bool IncrementalPipeline::alignLandmarksToBaseMap(
    const summary_map::LocalizationSummaryMap& base_summary_map) {
  vi_map::LandmarkIdList base_landmark_ids;
  base_summary_map.getAllLandmarkIds(&base_landmark_ids);
  LOG(INFO) << "Total " << base_landmark_ids.size() << " landmarks in base map";
  vi_map::LandmarkIdSet result_landmark_ids;
  m_result_map->getAllLandmarkIds(&result_landmark_ids);
  LOG(INFO) << "Total " << result_landmark_ids.size()
            << " landmarks in result map";
  std::vector<Eigen::Vector3d> P_result, P_base;
  for (auto landmark_id : base_landmark_ids) {
    if (result_landmark_ids.count(landmark_id) == 0) {
      LOG(INFO) << "landmark with ID " << landmark_id
                << " not existed in selected map";
      continue;
    }

    if (!vi_map::isLandmarkWellConstrained(
            *m_result_map, m_result_map->getLandmark(landmark_id))) {
      LOG(INFO) << "landmark with ID " << landmark_id
                << " not WellConstrained in selected map";
      continue;
    }

    P_result.push_back(m_result_map->getLandmark_G_p_fi(landmark_id));
    P_base.push_back(base_summary_map.getGLandmarkPosition(landmark_id));
  }

  LOG(INFO) << "reanchor result vi-map with " << P_result.size()
            << " landmark pairs";
  map_anchoring::reanchorMissionsWithLandmarkPairs(
      m_result_map.get(), P_result, P_base);
  return true;
}

summary_map::LocalizationSummaryMap::Ptr
IncrementalPipeline::generateSummaryMap(const vi_map::VIMap& vimap) {
  summary_map::LocalizationSummaryMap::Ptr summary_map(
      new summary_map::LocalizationSummaryMap());

  summary_map::createLocalizationSummaryMapForWellConstrainedLandmarks(
      vimap, summary_map.get());
  return summary_map;
}

void IncrementalPipeline::saveResultMap(
    const std::string& save_folder, bool also_save_summary_map,
    bool overwrite) {
  saveMap(m_result_map.get(), save_folder, also_save_summary_map, overwrite);
}

void IncrementalPipeline::saveMap(
    vi_map::VIMap* vimap, const std::string& save_folder,
    bool also_save_summary_map, bool overwrite) {
  std::string vimap_path = save_folder + std::string("/vi_map");
  backend::SaveConfig save_config;
  save_config.overwrite_existing_files = overwrite;
  vi_map::serialization::saveMapToFolder(vimap_path, save_config, vimap);
  LOG(INFO) << "Result VI-map saved to: " << vimap_path;

  if (also_save_summary_map) {
    LOG(INFO)
        << "Generating localization summary map for the result vi-map ... "
        << "please wait.";

    std::string summary_map_path = save_folder + std::string("/summary_map");
    summary_map::LocalizationSummaryMap::Ptr summary_map =
        generateSummaryMap(*vimap);
    if (!summary_map->saveToFolder(summary_map_path, save_config)) {
      LOG(WARNING) << "Saving summary map failed.";
    }
  }
}

}  // namespace incremental_map
