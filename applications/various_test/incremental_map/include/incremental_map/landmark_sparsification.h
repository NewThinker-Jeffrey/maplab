#ifndef INCREMENTAL_MAP_LANDMARK_SPARSIFICATION_H_
#define INCREMENTAL_MAP_LANDMARK_SPARSIFICATION_H_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

#include "map-sparsification/heuristic/scoring/scoring-function.h"
#include "map-sparsification/sampler-base.h"

#include "map-sparsification/visualization/map-sparsification-visualization.h"
// namespace map_sparsification_visualization {
// class MapSparsificationVisualizer;
//}

namespace incremental_map {

// Forward declaration of a wrapper that contains a pointer to a lp_solve
// lprec structure. By using it, we avoid populating all the
// global namespace definitions of lp_solve to our project.
struct LprecWrapper;
typedef map_sparsification::SamplerBase SamplerBase;
typedef map_sparsification::scoring::ScoringFunction ScoringFunction;

class LandmarkSparsification {
 public:
  struct Paramset {
    int keyframe_score_method;
    unsigned int min_keypoints_per_keyframe;
    double min_score_per_keyframe;
    double expected_min_score_per_landmark;
    double max_landmark_score_against_keyframe;
    static Paramset initializeFromGflag();
  };

  explicit LandmarkSparsification(const Paramset& paramset)
      : paramset_(paramset) {}

  void operator()(vi_map::VIMap* vimap);

  void extra_test(vi_map::VIMap* vimap, const Paramset& paramset);

  void instantiateVisualizer();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  typedef std::unordered_map<vi_map::LandmarkId, unsigned int>
      StoreLandmarkIdToIndexMap;

  void partitionMapIfNecessary(const vi_map::VIMap& map);
  void setDefaultScoringFunctions(
      std::vector<ScoringFunction::ConstPtr>* scoring_functions);
  void addKeyframeConstraint(
      pose_graph::VertexId vertex_id, const Paramset& paramset,
      const vi_map::LandmarkIdSet& keyframe_landmarks,
      const StoreLandmarkIdToIndexMap& landmark_ids_to_indices,
      unsigned int num_variables, const LprecWrapper& lprec_ptr) const;
  void addObjectiveFunction(
      const vi_map::VIMap& map, const Paramset& paramset,
      const StoreLandmarkIdToIndexMap& landmark_ids_to_indices,
      unsigned int num_variables, const LprecWrapper& lprec_ptr) const;
  int removeLandmarksExcept(
      vi_map::VIMap* vimap, const vi_map::LandmarkIdSet& landmarks_to_keep);
  int removeLandmarksExcept(
      vi_map::VIMap* vimap, const vi_map::LandmarkIdList& landmarks_to_keep);
  void prepareForSampling(vi_map::VIMap* map);
  void scoreLandmarks(
      const vi_map::VIMap& map,
      const std::vector<ScoringFunction::ConstPtr>& scoring_functions);

  // To illustrate distribution of landmarks' scores.
  void histogramLandmarkScores(
      const vi_map::LandmarkIdList& landmark_ids, bool print = false,
      std::vector<int>* histogram = nullptr) const;
  void sample(
      const vi_map::VIMap& map,
      vi_map::LandmarkIdSet* summary_store_landmark_ids,
      const Paramset& paramset);
  void sampleMapSegment(
      const vi_map::VIMap& map, unsigned int time_limit_seconds,
      const vi_map::LandmarkIdSet& segment_store_landmark_id_set,
      const pose_graph::VertexIdList& segment_vertex_id_list,
      vi_map::LandmarkIdSet* summary_store_landmark_ids,
      const Paramset& paramset);
  double retrieveScoreAgainstWholeMap(vi_map::LandmarkId landmark_id) const;
  double retrieveScoreAgainstKeyframe(
      pose_graph::VertexId vertex_id, vi_map::LandmarkId landmark_id) const;

 private:
  /// Each landmark is assigned with a global score, i.e. a score
  /// evalated against the whole map;
  typedef std::map<vi_map::LandmarkId, double> LandmarkScores;
  LandmarkScores landmark_scores_against_whole_map_;

  /// Besides, each landmark also has a score evaluated against each
  /// keyframe.
  /// We here organize the landmark-to-keyframe score in a nested map structure.
  typedef std::map<pose_graph::VertexId, LandmarkScores>
      LandmarkScoresAgainstKeyframes;
  LandmarkScoresAgainstKeyframes landmark_scores_against_every_keyframe_;

  /// paramset
  Paramset paramset_;

  /// Total well-constranined landmarks. Only for logging purpose.
  int total_well_constrained_;

  /// partitioning
  std::vector<pose_graph::VertexIdList> posegraph_partitioning_;
  std::vector<vi_map::LandmarkIdSet> partition_landmarks_;
  std::unique_ptr<map_sparsification_visualization::MapSparsificationVisualizer>
      visualizer_;
  static constexpr size_t kMaxNumLandmarkPerPartition = 5000u;
};

}  // namespace incremental_map

#endif  // INCREMENTAL_MAP_LANDMARK_SPARSIFICATION_H_
