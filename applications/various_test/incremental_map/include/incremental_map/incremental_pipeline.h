#ifndef INCREMENTAL_MAP_INCREMENTAL_PIPELINE_H_
#define INCREMENTAL_MAP_INCREMENTAL_PIPELINE_H_

#include <string>
#include <vector>

#include <gflags/gflags.h>
#include <localization-summary-map/localization-summary-map.h>
#include <vi-map/vi-map.h>
#include <visualization/viwls-graph-plotter.h>

namespace incremental_map {

class IncrementalPipeline {
 public:
  IncrementalPipeline();
  IncrementalPipeline(
      std::vector<vi_map::VIMap::Ptr>* vimaps,
      visualization::ViwlsGraphRvizPlotter* plotter = nullptr);

  ~IncrementalPipeline();

  void setVimapsToMerge(std::vector<vi_map::VIMap::Ptr>* vimaps);

  vi_map::VIMap::Ptr operator()();

  void saveResultMap(
      const std::string& save_folder, bool also_save_summary_map = false,
      bool overwrite = true);

 public:
  static void saveMap(
      vi_map::VIMap* vimap, const std::string& save_folder,
      bool also_save_summary_map = false, bool overwrite = true);

  static summary_map::LocalizationSummaryMap::Ptr generateSummaryMap(
      const vi_map::VIMap& vimap);

 private:
  void plotResultMap();
  void plotPartitioning(const pose_graph::VertexIdList& keyframe_ids);

  bool joinAllMaps();
  bool setFirstBaseFrameKnown();
  bool anchorAllMissions();
  bool keyframeMapBasedOnHeuristics();
  bool relaxMap();
  bool findLoopClosuresBetweenAllMissions();
  bool findLoopClosuresBetweenMissions(
      const vi_map::MissionIdList& mission_ids);
  bool optimizeVisualInertial(bool visual_only = false);
  bool alignLandmarksToBaseMap(
      const summary_map::LocalizationSummaryMap& base_summary_map);

 private:
  /// vimaps to be merged.
  std::vector<vi_map::VIMap::Ptr>* m_vimaps;

  /// the result vimap.
  vi_map::VIMap::Ptr m_result_map;

  /// visulizer.
  visualization::ViwlsGraphRvizPlotter* m_plotter;
};

}  // namespace incremental_map

#endif  // INCREMENTAL_MAP_INCREMENTAL_PIPELINE_H_
