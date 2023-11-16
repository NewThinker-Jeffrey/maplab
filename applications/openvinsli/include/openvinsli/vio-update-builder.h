#ifndef OPENVINSLI_VIO_UPDATE_BUILDER_H_
#define OPENVINSLI_VIO_UPDATE_BUILDER_H_

#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <utility>

#include <aslam/common/memory.h>
#include <maplab-common/localization-result.h>
#include <vio-common/map-update.h>
#include <vio-common/vio-types.h>

#include "openvinsli/openvins-estimate.h"

namespace openvinsli {

class VioUpdateBuilder {
 public:
  typedef std::function<void(const vio::MapUpdate::ConstPtr&)>
      VioUpdatePublishFunction;

  VioUpdateBuilder();

  void registerVioUpdatePublishFunction(
      const VioUpdatePublishFunction& vio_update_publish_function) {
    vio_update_publish_function_ = vio_update_publish_function;
  }

  void processSynchronizedNFrameImu(
      const vio::SynchronizedNFrameImu::ConstPtr& synced_nframe_imu);
  void processOpenvinsEstimate(const OpenvinsEstimate::ConstPtr& openvins_estimate);
  void processLocalizationResult(
      const vio::LocalizationResult::ConstPtr& localization_result);

  void clearSynchronizedNFrameImuQueue() {
    // For unit tests.
    SynchronizedNFrameImuQueue empty_queue;
    synced_nframe_imu_queue_.swap(empty_queue);
  }

 private:
  typedef std::queue<vio::SynchronizedNFrameImu::ConstPtr>
      SynchronizedNFrameImuQueue;
  typedef Aligned<std::deque, OpenvinsEstimate::ConstPtr> OpenvinsEstimateQueue;

  void findMatchAndPublish();
  void interpolateViNodeState(
      const int64_t timestamp_ns_a, const vio::ViNodeState& vi_node_a,
      const int64_t timestamp_ns_b, const vio::ViNodeState& vi_node_b,
      const int64_t timestamp_ns_interpolated,
      vio::ViNodeState* vi_node_interpolated);

  std::recursive_mutex queue_mutex_;
  SynchronizedNFrameImuQueue synced_nframe_imu_queue_;
  OpenvinsEstimateQueue openvins_estimate_queue_;
  // These values indicate the timestamp of the last message in the given topic
  // so that we can enforce that the timestamps are strictly monotonically
  // increasing
  int64_t last_received_timestamp_synced_nframe_queue_;
  int64_t last_received_timestamp_openvins_estimate_queue;

  VioUpdatePublishFunction vio_update_publish_function_;

  std::mutex mutex_last_localization_state_;
  common::LocalizationState last_localization_state_;
};

}  // namespace openvinsli

#endif  // OPENVINSLI_VIO_UPDATE_BUILDER_H_
