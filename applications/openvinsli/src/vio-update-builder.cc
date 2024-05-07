#include "openvinsli/vio-update-builder.h"

#include <maplab-common/interpolation-helpers.h>

namespace openvinsli {

VioUpdateBuilder::VioUpdateBuilder()
    : last_received_timestamp_synced_nframe_queue_(
          aslam::time::getInvalidTime()),
      last_received_timestamp_openvins_estimate_queue(
          aslam::time::getInvalidTime()) {}

void VioUpdateBuilder::processSynchronizedNFrameImu(
    const vio::SynchronizedNFrameImu::ConstPtr& synced_nframe_imu) {
  CHECK(synced_nframe_imu != nullptr);
  const int64_t timestamp_nframe_ns =
      synced_nframe_imu->nframe->getMaxTimestampNanoseconds();
  CHECK_GT(timestamp_nframe_ns, last_received_timestamp_synced_nframe_queue_);
  last_received_timestamp_synced_nframe_queue_ = timestamp_nframe_ns;

  std::lock_guard<std::recursive_mutex> lock(queue_mutex_);
  synced_nframe_imu_queue_.push(synced_nframe_imu);
  // std::cout << "DEBUG VioUpdateBuilder::processSynchronizedNFrameImu " << std::endl;
  findMatchAndPublish();
}

void VioUpdateBuilder::processOpenvinsEstimate(
    const OpenvinsEstimate::ConstPtr& openvins_estimate) {
  CHECK(openvins_estimate != nullptr);
  CHECK_GT(
      openvins_estimate->timestamp_ns,
      last_received_timestamp_openvins_estimate_queue);
  last_received_timestamp_openvins_estimate_queue = openvins_estimate->timestamp_ns;

  std::lock_guard<std::recursive_mutex> lock(queue_mutex_);
  openvins_estimate_queue_.push_back(openvins_estimate);
  // std::cout << "DEBUG VioUpdateBuilder::processOpenvinsEstimate " << std::endl;
  findMatchAndPublish();
}

void VioUpdateBuilder::processLocalizationResult(
    const vio::LocalizationResult::ConstPtr& localization_result) {
  CHECK(localization_result);
  std::lock_guard<std::mutex> lock(mutex_last_localization_state_);
  switch (localization_result->localization_mode) {
    case common::LocalizationMode::kGlobal:
      last_localization_state_ = common::LocalizationState::kLocalized;
      break;
    case common::LocalizationMode::kMapTracking:
      last_localization_state_ = common::LocalizationState::kMapTracking;
      break;
    default:
      LOG(FATAL) << "Unknown localization mode: "
                 << static_cast<int>(localization_result->localization_mode);
  }
}

void VioUpdateBuilder::findMatchAndPublish() {
  std::lock_guard<std::recursive_mutex> lock(queue_mutex_);
  if (synced_nframe_imu_queue_.empty() || openvins_estimate_queue_.empty()) {
    // Nothing to do.
    return;
  }

  const int64_t MAX_PRECISION_NS = 10;  // Maybe 1 is enough.
  const OpenvinsEstimate::ConstPtr& oldest_unmatched_estimate =
      openvins_estimate_queue_.front();
  const int64_t timestamp_estimate_ns =
      oldest_unmatched_estimate->timestamp_ns - MAX_PRECISION_NS;

  // remove frames earlier than the oldest_unmatched_estimate;
  while(!synced_nframe_imu_queue_.empty()) {
    const vio::SynchronizedNFrameImu::ConstPtr& front_frame =
        synced_nframe_imu_queue_.front();
    const int64_t tmp_timestamp_nframe_ns = front_frame->nframe->getMinTimestampNanoseconds();
    // std::cout << "DEBUG findMatchAndPublish(),  timestamp_estimate_ns = " << timestamp_estimate_ns << ",  timestamp_estimate_ns - tmp_timestamp_nframe_ns = " << timestamp_estimate_ns - tmp_timestamp_nframe_ns << std::endl;
    if (tmp_timestamp_nframe_ns < timestamp_estimate_ns) {
      synced_nframe_imu_queue_.pop();
      LOG(WARNING) << "findMatchAndPublish: abandon synced_nframe_imu at " << tmp_timestamp_nframe_ns
                   << ",  delay (relative to oldest_unmatched_estimate): "
                   << timestamp_estimate_ns - tmp_timestamp_nframe_ns
                   << ".  (For openvinsli, this might only happen in the beginning!)";
    } else {
      break;
    }
  }

  if (synced_nframe_imu_queue_.empty()) {
    // Nothing to do.
    return;
  }

  const vio::SynchronizedNFrameImu::ConstPtr& oldest_unmatched_synced_nframe =
      synced_nframe_imu_queue_.front();
  const int64_t timestamp_nframe_ns =
      oldest_unmatched_synced_nframe->nframe->getMinTimestampNanoseconds();

  // We need to use iterator instead of const_iterator because erase isn't
  // defined for const_iterators in g++ 4.8.
  OpenvinsEstimateQueue::iterator it_openvins_estimate_before_nframe =
      openvins_estimate_queue_.end();
  OpenvinsEstimateQueue::iterator it_openvins_estimate_after_nframe =
      openvins_estimate_queue_.end();

  bool found_exact_match = false;
  bool found_matches_to_interpolate = false;
  // Need at least two values for interpolation.
  for (it_openvins_estimate_before_nframe = openvins_estimate_queue_.begin();
       it_openvins_estimate_before_nframe != openvins_estimate_queue_.end();
       ++it_openvins_estimate_before_nframe) {
    it_openvins_estimate_after_nframe = it_openvins_estimate_before_nframe + 1;
    // Check if exact match.
    if (abs((*it_openvins_estimate_before_nframe)->timestamp_ns -
        timestamp_nframe_ns) <= MAX_PRECISION_NS) {
      LOG(INFO) << "findMatchAndPublish: Found exact_match. time_err_ns = "
                << timestamp_nframe_ns - (*it_openvins_estimate_before_nframe)->timestamp_ns;
      found_exact_match = true;
      break;
    }
    if (it_openvins_estimate_after_nframe != openvins_estimate_queue_.end() &&
        (*it_openvins_estimate_before_nframe)->timestamp_ns + MAX_PRECISION_NS <=
            timestamp_nframe_ns &&
        (*it_openvins_estimate_after_nframe)->timestamp_ns > timestamp_nframe_ns) {

      // Found matching vi nodes.
      LOG(WARNING) << "findMatchAndPublish: Found matching vi nodes. "
                   << "time_gaps = "
                   << timestamp_nframe_ns - (*it_openvins_estimate_before_nframe)->timestamp_ns
                   << ", "
                   << (*it_openvins_estimate_after_nframe)->timestamp_ns - timestamp_nframe_ns;
      found_matches_to_interpolate = true;
      break;
    }
  }

  if (!found_exact_match && !found_matches_to_interpolate) {
  // if (!found_exact_match) {  // only accecpt exact matches for openvinsli.
    LOG(WARNING) << "findMatchAndPublish: Find no match. "
                 << "queue_size: sync_frame=" << synced_nframe_imu_queue_.size()
                 << "  estimate=" << openvins_estimate_queue_.size() << std::endl;
    return;
  }

  CHECK(it_openvins_estimate_before_nframe != openvins_estimate_queue_.end());
  CHECK(
      found_exact_match ||
      it_openvins_estimate_after_nframe != openvins_estimate_queue_.end());
  CHECK(it_openvins_estimate_before_nframe != it_openvins_estimate_after_nframe);
  const OpenvinsEstimate::ConstPtr& openvins_estimate_before_nframe =
      *it_openvins_estimate_before_nframe;
  const OpenvinsEstimate::ConstPtr& openvins_estimate_after_nframe =
      *it_openvins_estimate_after_nframe;

  // Build MapUpdate.
  vio::MapUpdate::Ptr vio_update = aligned_shared<vio::MapUpdate>();
  vio_update->timestamp_ns = timestamp_nframe_ns;
  vio_update->keyframe = std::make_shared<vio::SynchronizedNFrame>(
      oldest_unmatched_synced_nframe->nframe,
      oldest_unmatched_synced_nframe->motion_wrt_last_nframe);
  vio_update->imu_timestamps = oldest_unmatched_synced_nframe->imu_timestamps;
  vio_update->imu_measurements =
      oldest_unmatched_synced_nframe->imu_measurements;

  if (found_exact_match) {
    vio_update->vinode = openvins_estimate_before_nframe->vinode;

    if (openvins_estimate_before_nframe->has_T_G_M) {
      vio_update->T_G_M = openvins_estimate_before_nframe->T_G_M;
    }
  } else {
    // Need to interpolate ViNode.
    const int64_t t_before = openvins_estimate_before_nframe->timestamp_ns;
    const int64_t t_after = openvins_estimate_after_nframe->timestamp_ns;

    vio::ViNodeState interpolated_vi_node;
    interpolateViNodeState(
        t_before, openvins_estimate_before_nframe->vinode, t_after,
        openvins_estimate_after_nframe->vinode, timestamp_nframe_ns,
        &interpolated_vi_node);
    vio_update->vinode = interpolated_vi_node;

    if (openvins_estimate_before_nframe->has_T_G_M &&
        openvins_estimate_after_nframe->has_T_G_M) {
      common::interpolateTransformation(
          t_before, openvins_estimate_before_nframe->T_G_M, t_after,
          openvins_estimate_after_nframe->T_G_M, timestamp_nframe_ns,
          &vio_update->T_G_M);
    }
  }

  vio_update->vio_state = vio::EstimatorState::kRunning;
  vio_update->map_update_type = vio::UpdateType::kNormalUpdate;
  {
    std::lock_guard<std::mutex> lock(mutex_last_localization_state_);
    vio_update->localization_state = last_localization_state_;
    last_localization_state_ = common::LocalizationState::kUninitialized;
  }

  // Publish VIO update.
  CHECK(vio_update_publish_function_);
  // std::cout << "DEBUG MapBuilderFlow findMatchAndPublish: Publish VIO update" << std::endl;
  vio_update_publish_function_(vio_update);

  // Clean up queues.

  // For openvinsli, we need different clean up logic from that in rovioli.
  // We should ensure every estimate corresponds to at most one nframe, so that
  // the number of nframes added to vimap will be almost 1:1 to the number of
  // frames used in vio.
  // 
  // Thus, we should erase all before it_openvins_estimate_before_nframe (including it self).
  openvins_estimate_queue_.erase(
      openvins_estimate_queue_.begin(), it_openvins_estimate_before_nframe + 1);

  /// Original code from rovioli:
  // if (it_rovio_estimate_before_nframe != rovio_estimate_queue_.begin()) {
  //   if (found_exact_match) {
  //     rovio_estimate_queue_.erase(
  //         rovio_estimate_queue_.begin(), it_rovio_estimate_before_nframe);
  //   } else {
  //     // Keep the two ViNodeStates that were used for interpolation as a
  //     // subsequent SynchronizedNFrameImu may need to be interpolated between
  //     // those two points again.

  //     // NOTE(jeffrey): it might be a minor mistake in rovoili: ' - 1 ' is unnecessary.
  //     // Maybe the code shuold be modified as below:
  //     //   rovio_estimate_queue_.erase(
  //     //       rovio_estimate_queue_.begin(), it_rovio_estimate_before_nframe);
  //     // However, it doesn't cause any problem, so we keep the original code here.

  //     rovio_estimate_queue_.erase(
  //         rovio_estimate_queue_.begin(), it_rovio_estimate_before_nframe - 1);
  //   }
  // }

  synced_nframe_imu_queue_.pop();
}

void VioUpdateBuilder::interpolateViNodeState(
    const int64_t timestamp_ns_a, const vio::ViNodeState& vi_node_a,
    const int64_t timestamp_ns_b, const vio::ViNodeState& vi_node_b,
    const int64_t timestamp_ns_interpolated,
    vio::ViNodeState* vi_node_interpolated) {
  CHECK_NOTNULL(vi_node_interpolated);
  CHECK_LT(timestamp_ns_a, timestamp_ns_b);
  CHECK_LE(timestamp_ns_a, timestamp_ns_interpolated);
  CHECK_LE(timestamp_ns_interpolated, timestamp_ns_b);

  // Interpolate pose.
  aslam::Transformation interpolated_T_M_I;
  common::interpolateTransformation(
      timestamp_ns_a, vi_node_a.get_T_M_I(), timestamp_ns_b,
      vi_node_b.get_T_M_I(), timestamp_ns_interpolated, &interpolated_T_M_I);
  vi_node_interpolated->set_T_M_I(interpolated_T_M_I);

  // Interpolate velocity.
  Eigen::Vector3d interpolated_v_M_I;
  common::linearInterpolation(
      timestamp_ns_a, vi_node_a.get_v_M_I(), timestamp_ns_b,
      vi_node_b.get_v_M_I(), timestamp_ns_interpolated, &interpolated_v_M_I);
  vi_node_interpolated->set_v_M_I(interpolated_v_M_I);

  // Interpolate biases.
  Eigen::Vector3d interpolated_acc_bias, interpolated_gyro_bias;
  common::linearInterpolation(
      timestamp_ns_a, vi_node_a.getAccBias(), timestamp_ns_b,
      vi_node_b.getAccBias(), timestamp_ns_interpolated,
      &interpolated_acc_bias);
  common::linearInterpolation(
      timestamp_ns_a, vi_node_a.getGyroBias(), timestamp_ns_b,
      vi_node_b.getGyroBias(), timestamp_ns_interpolated,
      &interpolated_gyro_bias);
  vi_node_interpolated->setAccBias(interpolated_acc_bias);
  vi_node_interpolated->setGyroBias(interpolated_gyro_bias);
}

}  // namespace openvinsli
