#include "incremental_map/landmark_sparsification.h"

#include <limits>
#include <memory>
#include <vector>

#include <glog/logging.h>
#include <lp_solve/lp_lib.h>

#include "aslam/common/timer.h"
#include "map-sparsification/heuristic/scoring/observation-count-scoring.h"
#include "map-sparsification/sampler-factory.h"
#include "vi-map-helpers/vi-map-partitioner.h"
#include "vi-map-helpers/vi-map-queries.h"
// #include
// "map-sparsification/visualization/map-sparsification-visualization.h"

DEFINE_int32(
    keyframe_score_method, 2, "Select the method to score a keyframe.");
DEFINE_int32(
    min_keypoints_kept_per_keyframe, 90,
    "Minimum desired number of keypoints kept per each "
    "map keyframe, except for the keyframes which own less "
    "keypoints at the begining. (strictly constrained)");
DEFINE_double(
    min_score_kept_per_keyframe, 180.0,
    "Minimum desired score per each map keyframe, "
    "except for the keyframes which score less "
    "at the begining. (strictly constrained)");
DEFINE_double(
    expected_min_score_per_landmark, 20.0,
    "Minimum score per landmark. (loosely constrained)");
DEFINE_double(
    max_landmark_score_against_keyframe, 20.0,
    "Maximum score per landmark contribute to per keyframe.");

namespace incremental_map {

struct LprecWrapper {
  lprec* lprec_ptr;
};

void LandmarkSparsification::instantiateVisualizer() {
  visualizer_.reset(
      new map_sparsification_visualization::MapSparsificationVisualizer());
}

void LandmarkSparsification::partitionMapIfNecessary(const vi_map::VIMap& map) {
  // We've ensured that all the landmarks still kept in the vimap
  // are well constrained before stepping into this function. So
  // here we treat every landmark as well constrained implicitly.
  const size_t num_landmarks = map.numLandmarks();
  LOG(INFO) << "Number of well constrained landmarks before sparsification: "
            << num_landmarks;

  if (num_landmarks < kMaxNumLandmarkPerPartition) {
    posegraph_partitioning_.resize(1u);
    map.getAllVertexIds(&(posegraph_partitioning_[0]));
  } else {
    const size_t num_partitions = std::ceil(
        static_cast<double>(num_landmarks) / kMaxNumLandmarkPerPartition);
    LOG(INFO) << "Number of well constrained landmarks " << num_landmarks
              << " exceeds " << kMaxNumLandmarkPerPartition
              << ". Will partition the graph "
              << "into " << num_partitions << " partitions.";
    vi_map_helpers::VIMapPartitioner partitioner;
    partitioner.partitionMapWithMetis(
        map, num_partitions, &posegraph_partitioning_);
    CHECK_EQ(num_partitions, posegraph_partitioning_.size());
  }
}

void LandmarkSparsification::histogramLandmarkScores(
    const vi_map::LandmarkIdList& landmark_ids, bool print,
    std::vector<int>* histogram) const {
  std::vector<int> tmp_histogram;
  if (!histogram)
    histogram = &tmp_histogram;
  histogram->clear();
  for (auto landmark_id : landmark_ids) {
    double score = retrieveScoreAgainstWholeMap(landmark_id);
    int floor_score = static_cast<int>(score);
    floor_score = floor_score > 0 ? floor_score : 0;
    if (floor_score >= histogram->size()) {
      histogram->resize(floor_score + 1, 0);
    }
    (*histogram)[floor_score]++;
  }

  if (print) {
    std::ostringstream oss;
    for (int i = 0; i < histogram->size(); i++) {
      if ((*histogram)[i] > 0)
        oss << i << "(" << (*histogram)[i] << "), ";
    }
    oss << "total(" << landmark_ids.size() << ")";
    LOG(INFO) << "score distribution: " << oss.str();
  }
}

void LandmarkSparsification::scoreLandmarks(
    const vi_map::VIMap& map,
    const std::vector<ScoringFunction::ConstPtr>& scoring_functions) {
  LOG(INFO) << "Scoring against the whole map ...";
  vi_map::LandmarkIdSet all_landmark_ids;
  map.getAllLandmarkIds(&all_landmark_ids);
  for (vi_map::LandmarkId landmark_id : all_landmark_ids) {
    double landmark_score = 0.0;
    for (unsigned int i = 0; i < scoring_functions.size(); ++i) {
      landmark_score += (*scoring_functions[i])(landmark_id, map);
    }
    landmark_scores_against_whole_map_[landmark_id] = landmark_score;
  }

  LOG(INFO) << "Scoring against every keyframe ...";
  pose_graph::VertexIdList vertex_ids;
  map.getAllVertexIds(&vertex_ids);
  for (pose_graph::VertexId vertex_id : vertex_ids) {
    pose_graph::VertexIdSet tmp_vertex_ids;
    vi_map::LandmarkIdSet observed_landmark_ids;
    LandmarkScores empty_landmark_scores_against_keyframe;
    landmark_scores_against_every_keyframe_[vertex_id] =
        empty_landmark_scores_against_keyframe;
    LandmarkScores& landmark_scores_against_keyframe =
        landmark_scores_against_every_keyframe_[vertex_id];
    tmp_vertex_ids.emplace(vertex_id);
    map.getAllLandmarkIdsObservedAtVertices(
        tmp_vertex_ids, &observed_landmark_ids);
    for (vi_map::LandmarkId landmark_id : observed_landmark_ids) {
      // Currently we assign the landmark-to-keyframe score with
      // the global score of the landmark.
      double score = landmark_scores_against_whole_map_[landmark_id];
      landmark_scores_against_keyframe[landmark_id] = score;
    }
  }
}

void LandmarkSparsification::prepareForSampling(vi_map::VIMap* vimap) {
  LOG(INFO) << "Retriving well constrained landmarks ...";
  vi_map_helpers::VIMapQueries queries(*vimap);
  vi_map::LandmarkIdList well_constrained_landmarks;
  queries.getAllWellConstrainedLandmarkIds(&well_constrained_landmarks);
  total_well_constrained_ = well_constrained_landmarks.size();

  LOG(INFO) << "Removing not well constrained landmarks ...";
  int total = vimap->numLandmarks();
  int removed = removeLandmarksExcept(vimap, well_constrained_landmarks);
  LOG(INFO) << "Pre-removed " << removed << " landmarks from " << total;

  LOG(INFO) << "Begin partitioning the map （if necessary) ...";
  partitionMapIfNecessary(*vimap);

  LOG(INFO) << "Clearing previous scores ...";
  landmark_scores_against_whole_map_.clear();
  landmark_scores_against_every_keyframe_.clear();

  LOG(INFO) << "Begin re-calculating the scores ...";
  std::vector<ScoringFunction::ConstPtr> scoring_functions;
  setDefaultScoringFunctions(&scoring_functions);
  scoreLandmarks(*vimap, scoring_functions);
  histogramLandmarkScores(well_constrained_landmarks, true);
}

void LandmarkSparsification::setDefaultScoringFunctions(
    std::vector<ScoringFunction::ConstPtr>* scoring_functions) {
  scoring_functions->clear();

  using map_sparsification::scoring::ObservationCountScoringFunction;

  const double kObsCountWeight = 1.0;
  ObservationCountScoringFunction::Ptr obs_count_score(
      new ObservationCountScoringFunction);
  obs_count_score->setWeight(kObsCountWeight);
  scoring_functions->push_back(obs_count_score);
}

void LandmarkSparsification::operator()(vi_map::VIMap* vimap) {
  int total = vimap->numLandmarks();

  // partitionMapIfNecessary(*vimap);
  prepareForSampling(vimap);

  vi_map::LandmarkIdSet landmarks_to_keep;
  sample(*vimap, &landmarks_to_keep, paramset_);

  int removed = removeLandmarksExcept(vimap, landmarks_to_keep);
  LOG(INFO) << "Removed " << removed << " landmarks from " << total;
  LOG(INFO) << "Number of original well constrained landmarks : "
            << total_well_constrained_;
}

void LandmarkSparsification::extra_test(
    vi_map::VIMap* vimap, const Paramset& paramset) {
  int total = vimap->numLandmarks();
  vi_map::LandmarkIdSet landmarks_to_keep;
  sample(*vimap, &landmarks_to_keep, paramset);

  int removed = removeLandmarksExcept(vimap, landmarks_to_keep);
  LOG(INFO) << "Removed " << removed << " landmarks from " << total;
  LOG(INFO) << "Number of original well constrained landmarks : "
            << total_well_constrained_;
}

double LandmarkSparsification::retrieveScoreAgainstWholeMap(
    vi_map::LandmarkId landmark_id) const {
  auto iter = landmark_scores_against_whole_map_.find(landmark_id);
  if (iter != landmark_scores_against_whole_map_.end()) {
    double score = iter->second;
    return score;
  }

  LOG(WARNING) << "Can't find score for landmark " << landmark_id;
  return 0.0;
}

double LandmarkSparsification::retrieveScoreAgainstKeyframe(
    pose_graph::VertexId vertex_id, vi_map::LandmarkId landmark_id) const {
  auto vertex_iter = landmark_scores_against_every_keyframe_.find(vertex_id);
  if (vertex_iter != landmark_scores_against_every_keyframe_.end()) {
    auto& landmark_scores = vertex_iter->second;
    auto iter = landmark_scores.find(landmark_id);
    if (iter != landmark_scores.end()) {
      double score = iter->second;
      return score;
    }
  }

  LOG(WARNING) << "Can't find score for landmark " << landmark_id
               << " against vertex " << vertex_id;
  return 0.0;
}

void LandmarkSparsification::sampleMapSegment(
    const vi_map::VIMap& map, unsigned int /*time_limit_seconds*/,
    const vi_map::LandmarkIdSet& segment_landmark_id_set,
    const pose_graph::VertexIdList& segment_vertex_id_list,
    vi_map::LandmarkIdSet* summary_landmark_ids, const Paramset& paramset) {
  CHECK_NOTNULL(summary_landmark_ids)->clear();

  // Reset variables.
  StoreLandmarkIdToIndexMap landmark_ids_to_indices;
  unsigned int num_variables;
  landmark_ids_to_indices.clear();
  num_variables = 0u;

  vi_map::LandmarkIdList all_landmark_ids(
      segment_landmark_id_set.begin(), segment_landmark_id_set.end());

  for (unsigned int i = 0; i < all_landmark_ids.size(); ++i) {
    // +1 to keep consistency with lp-solve variable indexing (starts with 1).
    landmark_ids_to_indices.emplace(all_landmark_ids[i], i + 1);
  }

  // We don't know the exact constraint number right now, the problem will be
  // extended on the fly.
  const unsigned int kInitialConstraintCount = 0;
  // Adding 1 because our state consists of switch variable for each landmark
  // AND a slack variable.
  num_variables = all_landmark_ids.size() + 1;

  // Construct lprec object.
  LprecWrapper lprec_ptr;
  lprec_ptr.lprec_ptr = make_lp(kInitialConstraintCount, num_variables);
  CHECK_NOTNULL(lprec_ptr.lprec_ptr);

  // Set verbose to show only critical messages.
  set_verbose(lprec_ptr.lprec_ptr, CRITICAL);

  std::string lp_name = "ILP landmark summary";
  char* c_lp_name = &lp_name[0u];
  set_lp_name(lprec_ptr.lprec_ptr, c_lp_name);

  addObjectiveFunction(
      map, paramset, landmark_ids_to_indices, num_variables, lprec_ptr);

  for (const pose_graph::VertexId& vertex_id : segment_vertex_id_list) {
    vi_map::LandmarkIdList observer_landmarks;
    map.getVertex(vertex_id).getAllObservedLandmarkIds(&observer_landmarks);
    vi_map::LandmarkIdSet landmark_observers;
    for (const vi_map::LandmarkId& landmark_id : observer_landmarks) {
      if (landmark_id.isValid()) {
        landmark_observers.emplace(landmark_id);
      }
    }
    addKeyframeConstraint(
        vertex_id, paramset, landmark_observers, landmark_ids_to_indices,
        num_variables, lprec_ptr);
  }

  // setLandmarkSwitchVariablesToBinary
  for (unsigned int i = 1; i <= landmark_ids_to_indices.size(); ++i) {
    set_binary(lprec_ptr.lprec_ptr, i, TRUE);
  }
  // Slack variable is an integer one.
  set_int(lprec_ptr.lprec_ptr, num_variables, TRUE);

  // Using Basis Factorization Package (LU decomposition).
  static char kBfpConfigurationString[] = "bfp_LUSOL";
  set_BFP(lprec_ptr.lprec_ptr, kBfpConfigurationString);
  set_scaling(lprec_ptr.lprec_ptr, SCALE_CURTISREID);
  // Example presolver call from lp_solve documentation.
  set_presolve(
      lprec_ptr.lprec_ptr, PRESOLVE_ROWS | PRESOLVE_COLS | PRESOLVE_LINDEP,
      get_presolveloops(lprec_ptr.lprec_ptr));

  // It seems NODE_RANGESELECT does to trick for the optimizer to realize it
  // need to adapt the slack variable. Other flags are default ones for
  // lp_solve.
  set_bb_rule(
      lprec_ptr.lprec_ptr,
      NODE_RANGESELECT | NODE_GREEDYMODE | NODE_DYNAMICMODE | NODE_RCOSTFIXING);

  LOG(INFO) << "Begin solving ...";
  int ret = solve(lprec_ptr.lprec_ptr);
  switch (ret) {
    case NOMEMORY:
      LOG(ERROR) << "Couldn't solve ILP summarization problem,"
                 << " ran out of memory.";
      return;
      break;
    case OPTIMAL:
      LOG(INFO) << "Optimal solution found.";
      break;
    case SUBOPTIMAL:
      LOG(WARNING) << "Possibly suboptimal solution found.";
      break;
    case INFEASIBLE:
      LOG(ERROR) << "ILP summarization problem infeasible.";
      return;
      break;
    case UNBOUNDED:
      LOG(ERROR) << "ILP summarization problem unbounded.";
      return;
      break;
    case DEGENERATE:
      LOG(ERROR) << "ILP summarization problem degenerate.";
      return;
      break;
    case NUMFAILURE:
      LOG(ERROR) << "Numerical failure when solving ILP summarization problem.";
      return;
      break;
    case TIMEOUT:
      LOG(ERROR) << "ILP summarization solver timeout.";
      return;
      break;
    default:
      LOG(ERROR) << "Solver returned an error with code: " << ret;
      break;
  }

  REAL* variables_ptr;
  get_ptr_variables(lprec_ptr.lprec_ptr, &variables_ptr);
  CHECK_NOTNULL(variables_ptr);

  if (variables_ptr[num_variables - 1] > 0) {
    LOG(WARNING) << "Slack variable isn't zero, but this doens't matter. "
                 << "value: " << variables_ptr[num_variables - 1];
  }

  unsigned int num_landmarks_left = 0;
  for (const StoreLandmarkIdToIndexMap::value_type& landmark_id_with_index :
       landmark_ids_to_indices) {
    // Substract 1 as variables_ptr is pointing to lprec struct data array
    // which is indexed from 0 (contrary to row indexing in the problem).
    if (fabs(variables_ptr[landmark_id_with_index.second - 1] - 1.0) <
        std::numeric_limits<float>::epsilon()) {
      summary_landmark_ids->emplace(landmark_id_with_index.first);
      ++num_landmarks_left;
    }
  }

  delete_lp(lprec_ptr.lprec_ptr);
}

void LandmarkSparsification::sample(
    const vi_map::VIMap& map, vi_map::LandmarkIdSet* summary_landmark_ids,
    const Paramset& paramset) {
  CHECK_NOTNULL(summary_landmark_ids)->clear();

  const size_t num_store_landmarks = map.numLandmarks();
  if (num_store_landmarks == 0) {
    LOG(WARNING) << "No landmarks in the map, bailing out early.";
  }

  // Reset plotting data.
  if (visualizer_) {
    partition_landmarks_.clear();
    const bool kGloballySelectedLandmarks = false;
    vi_map::LandmarkIdSet empty_set;
    visualizer_->plotLandmarks(
        map, -1, empty_set, posegraph_partitioning_, partition_landmarks_,
        kGloballySelectedLandmarks);
  }
  partition_landmarks_.resize(posegraph_partitioning_.size());

  // TODO(dymczykm) Parallelize this.
  for (unsigned int i = 0; i < posegraph_partitioning_.size(); ++i) {
    LOG(INFO) << "Sampling cluster " << (i + 1) << " of "
              << posegraph_partitioning_.size();
    if (visualizer_) {
      visualizer_->plotSegment(map, posegraph_partitioning_, i);
    }
    vi_map::LandmarkIdSet segment_landmark_id_set;
    unsigned int num_segment_landmarks = 0;
    LOG(INFO) << "\tBuilding the landmark set for the segment";
    for (const pose_graph::VertexId& vertex_id : posegraph_partitioning_[i]) {
      for (const vi_map::Landmark landmark :
           map.getVertex(vertex_id).getLandmarks()) {
        ++num_segment_landmarks;
        if (map.getLandmark(landmark.id()).getQuality() ==
            vi_map::Landmark::Quality::kGood) {
          segment_landmark_id_set.insert(landmark.id());
        }
      }
    }

    // Time limit of the sampling process of a single map partition.
    const unsigned int kSegmentTimeLimitSeconds = 8;
    LOG(INFO) << "\tVertices in current segment: "
              << posegraph_partitioning_[i].size();
    LOG(INFO) << "\tLandmarks in current segment: " << num_segment_landmarks;
    LOG(INFO) << "\tWell-constrained landmarks in current segment: "
              << segment_landmark_id_set.size();
    LOG(INFO) << "\tSampling out of " << segment_landmark_id_set.size()
              << " landmarks";
    vi_map::LandmarkIdSet segment_summary_landmark_ids;

    timing::Timer sampling_timer(
        "GraphPartitionSampler: " +
        std::to_string(posegraph_partitioning_.size()) +
        "partitions_sampling_timer");
    this->sampleMapSegment(
        map, kSegmentTimeLimitSeconds, segment_landmark_id_set,
        posegraph_partitioning_[i], &segment_summary_landmark_ids, paramset);
    sampling_timer.Stop();

    LOG(INFO) << "\t" << segment_summary_landmark_ids.size()
              << " landmarks inserted from this segment.";
    summary_landmark_ids->insert(
        segment_summary_landmark_ids.begin(),
        segment_summary_landmark_ids.end());

    if (visualizer_) {
      partition_landmarks_[i].insert(
          segment_landmark_id_set.begin(), segment_landmark_id_set.end());

      const bool kGloballySelectedLandmarks = false;
      visualizer_->plotLandmarks(
          map, i, segment_landmark_id_set, posegraph_partitioning_,
          partition_landmarks_, kGloballySelectedLandmarks);
    }
  }

  if (posegraph_partitioning_.size() > 1) {
    LOG(WARNING) << "Global optimization -- from "
                 << summary_landmark_ids->size() << " landmarks.";
    if (visualizer_) {
      const int kMarkAllVertices = -1;
      visualizer_->plotSegment(map, posegraph_partitioning_, kMarkAllVertices);
    }

    vi_map::LandmarkIdSet partitioned_summary_landmarks(
        summary_landmark_ids->begin(), summary_landmark_ids->end());

    pose_graph::VertexIdList all_vertex_ids;
    map.getAllVertexIds(&all_vertex_ids);

    // Time limit of the sampling process of a final, global sampling stage.
    const unsigned int kGlobalTimeLimitSeconds = 600;
    this->sampleMapSegment(
        map, kGlobalTimeLimitSeconds, partitioned_summary_landmarks,
        all_vertex_ids, summary_landmark_ids, paramset);
  }
  if (visualizer_) {
    const bool kGloballySelectedLandmarks = true;
    visualizer_->plotLandmarks(
        map, -1, *summary_landmark_ids, posegraph_partitioning_,
        partition_landmarks_, kGloballySelectedLandmarks);
  }
  LOG(INFO) << "Partitioned graph sampling done, landmarks left: "
            << summary_landmark_ids->size();
}

void LandmarkSparsification::addKeyframeConstraint(
    pose_graph::VertexId vertex_id, const Paramset& paramset,
    const vi_map::LandmarkIdSet& keyframe_landmarks,
    const StoreLandmarkIdToIndexMap& landmark_ids_to_indices,
    unsigned int num_variables, const LprecWrapper& lprec_ptr) const {
  CHECK_NOTNULL(lprec_ptr.lprec_ptr);

  // Add 1 as lp_solve is skipping value at index 0.
  const unsigned int kRowSize = 1 + num_variables;
  // Initialize the row with kRowSize zeros.
  std::vector<REAL> row(kRowSize, 0.0);
  std::vector<double> unordered_powered_scores;

  double power = paramset.keyframe_score_method;
  double powered_score_thr = pow(paramset.min_score_per_keyframe, power);
  unsigned int landmarks_in_the_segment = 0;
  double score_against_keyframe;
  double powered_score;
  double initial_keyframe_powered_score = 0.0;
  for (const vi_map::LandmarkId& landmark_id : keyframe_landmarks) {
    StoreLandmarkIdToIndexMap::const_iterator it =
        landmark_ids_to_indices.find(landmark_id);
    if (it != landmark_ids_to_indices.end()) {
      // This landmark is present in the segment being summarized.
      score_against_keyframe = std::min(
          retrieveScoreAgainstKeyframe(vertex_id, landmark_id),
          paramset.max_landmark_score_against_keyframe);
      ++landmarks_in_the_segment;
      row[it->second] = 1.0;
      powered_score = pow(score_against_keyframe, power);
      unordered_powered_scores.push_back(powered_score);
      initial_keyframe_powered_score += powered_score;
    }
  }

  // Set lower-bound for number of observed landmarks per keyframe.
  // Slack variable with weight 1.0.
  // row[num_variables] = 1.0;
  row[num_variables] = 0.0;  // We don't need slack variable

  // Row mode editing is useful (and recommended) to make the constraint
  // addition faster.
  unsigned int lower_bound = paramset.min_keypoints_per_keyframe;
  if (initial_keyframe_powered_score <= powered_score_thr ||
      landmarks_in_the_segment <= paramset.min_keypoints_per_keyframe) {
    lower_bound = landmarks_in_the_segment;
  } else {
    double average_powered_score;
    size_t middle = unordered_powered_scores.size() / 2;
    std::nth_element(
        unordered_powered_scores.begin(),
        unordered_powered_scores.begin() + middle,
        unordered_powered_scores.end());
    average_powered_score = unordered_powered_scores[middle];
    unsigned int expected = powered_score_thr / average_powered_score + 1;
    lower_bound = std::max(lower_bound, expected);
    lower_bound = std::min(lower_bound, landmarks_in_the_segment);
  }

  set_add_rowmode(lprec_ptr.lprec_ptr, TRUE);
  add_constraint(lprec_ptr.lprec_ptr, &row.front(), GE, lower_bound);
  // Return from row mode to standard mode.
  set_add_rowmode(lprec_ptr.lprec_ptr, FALSE);
}

void LandmarkSparsification::addObjectiveFunction(
    const vi_map::VIMap& map, const Paramset& paramset,
    const StoreLandmarkIdToIndexMap& landmark_ids_to_indices,
    unsigned int num_variables, const LprecWrapper& lprec_ptr) const {
  CHECK_NOTNULL(lprec_ptr.lprec_ptr);

  const unsigned int kRowSize = 1 + num_variables;
  std::vector<REAL> row(kRowSize, 0.0);

  // const int64_t kSlackVariableCost = 1e10;
  const int64_t kSlackVariableCost = 0;  // We don't use slack variable.

  vi_map::LandmarkIdList landmark_ids;
  for (const StoreLandmarkIdToIndexMap::value_type& landmark_id_with_index :
       landmark_ids_to_indices) {
    const vi_map::LandmarkId& landmark_id = landmark_id_with_index.first;
    landmark_ids.push_back(landmark_id);
    REAL score = retrieveScoreAgainstWholeMap(landmark_id);
    row[landmark_id_with_index.second] =
        score - paramset.expected_min_score_per_landmark;
  }
  histogramLandmarkScores(landmark_ids, true);

  // Slack variable with penalty weight. We maximize our objective so
  // slack variable introduces negative score.
  row[num_variables] = -kSlackVariableCost;

  set_obj_fn(lprec_ptr.lprec_ptr, &row.front());
  set_maxim(lprec_ptr.lprec_ptr);
}

int LandmarkSparsification::removeLandmarksExcept(
    vi_map::VIMap* vimap, const vi_map::LandmarkIdSet& landmarks_to_keep) {
  vi_map::LandmarkIdSet all_landmark_ids;
  vimap->getAllLandmarkIds(&all_landmark_ids);
  int removed = 0;
  for (const vi_map::LandmarkId& landmark_id : all_landmark_ids) {
    if (landmarks_to_keep.count(landmark_id) == 0) {
      vimap->removeLandmark(landmark_id);
      removed++;
    }
  }
  return removed;
}

int LandmarkSparsification::removeLandmarksExcept(
    vi_map::VIMap* vimap,
    const vi_map::LandmarkIdList& landmarks_to_keep_list) {
  vi_map::LandmarkIdSet landmarks_to_keep;
  for (auto landmark : landmarks_to_keep_list) {
    landmarks_to_keep.emplace(landmark);
  }
  return removeLandmarksExcept(vimap, landmarks_to_keep);
}

LandmarkSparsification::Paramset
LandmarkSparsification::Paramset::initializeFromGflag() {
  Paramset paramset;
  paramset.min_keypoints_per_keyframe = FLAGS_min_keypoints_kept_per_keyframe;
  paramset.max_landmark_score_against_keyframe =
      FLAGS_min_keypoints_kept_per_keyframe;
  paramset.min_score_per_keyframe = FLAGS_min_score_kept_per_keyframe;
  paramset.expected_min_score_per_landmark =
      FLAGS_expected_min_score_per_landmark;
  paramset.keyframe_score_method = FLAGS_keyframe_score_method;
  return paramset;
}

}  // namespace incremental_map
