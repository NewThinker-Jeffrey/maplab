#ifndef OPENVINSLI_FEATURE_TRACKING_FLOW_H_
#define OPENVINSLI_FEATURE_TRACKING_FLOW_H_

#include <aslam/cameras/ncamera.h>
#include <message-flow/message-flow.h>
#include <sensors/imu.h>
#include <vio-common/vio-types.h>

#include "openvinsli/feature-tracking.h"
#include "openvinsli/flow-topics.h"

namespace openvinsli {

class FeatureTrackingFlow {
 public:
  FeatureTrackingFlow(
      const aslam::NCamera::Ptr& camera_system, const vi_map::Imu& imu_sensor,
      double max_feattrack_frequency_hz = -1.0)
      : tracking_pipeline_(camera_system, imu_sensor),
        max_feattrack_frequency_hz_(max_feattrack_frequency_hz) {
    CHECK(camera_system);
  }

  void attachToMessageFlow(message_flow::MessageFlow* flow) {
    CHECK_NOTNULL(flow);
    static constexpr char kSubscriberNodeName[] = "FeatureTrackingFlow";

    std::function<void(vio::SynchronizedNFrameImu::ConstPtr)> publish_result =
        flow->registerPublisher<message_flow_topics::TRACKED_NFRAMES_AND_IMU>();

    // NOTE: the publisher function pointer is copied intentionally; otherwise
    // we would capture a reference to a temporary.
    flow->registerSubscriber<message_flow_topics::SYNCED_NFRAMES_AND_IMU>(
        kSubscriberNodeName, message_flow::DeliveryOptions(),
        [publish_result,
         this](const vio::SynchronizedNFrameImu::Ptr& nframe_imu) {
          CHECK(nframe_imu);
          int64_t ts = nframe_imu->nframe->getMinTimestampNanoseconds();
          LOG(INFO) << "FeatureTrackingFlow: Received nframe with ts "
                    << ts;

          if (max_feattrack_frequency_hz_ > 0) {
            std::unique_lock<std::mutex> lock(m_prev_processed_nframe_ts_);
            if (prev_processed_nframe_ts_ > 0 &&
                (ts - prev_processed_nframe_ts_) <
                  kSecondsToNanoSeconds / max_feattrack_frequency_hz_) {
              return;
            }
            prev_processed_nframe_ts_ = ts;
          }

          // LOG(INFO) << "FeatureTrackingFlow: Before processing nframe with ts "
          //           << ts;
          const bool success =
              this->tracking_pipeline_.trackSynchronizedNFrameImuCallback(
                  nframe_imu);
          LOG(INFO) << "FeatureTrackingFlow: Processed nframe with ts "
                    << ts << ", success = " << success;
          
          if (success) {
            // This will only fail for the first frame.
            publish_result(nframe_imu);
          }
        });

    flow->registerSubscriber<message_flow_topics::OPENVINS_ESTIMATES>(
        kSubscriberNodeName, message_flow::DeliveryOptions(),
        [this](const OpenvinsEstimate::ConstPtr& estimate) {
          CHECK(estimate);
          this->tracking_pipeline_.setCurrentImuBias(estimate);
        });
  }

 private:
  FeatureTracking tracking_pipeline_;

  double max_feattrack_frequency_hz_ = -1.0;  // negative for no throttling
  int64_t prev_processed_nframe_ts_ = -1;
  std::mutex m_prev_processed_nframe_ts_;
};

}  // namespace openvinsli

#endif  // OPENVINSLI_FEATURE_TRACKING_FLOW_H_
