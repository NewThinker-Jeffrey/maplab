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
      const aslam::NCamera::Ptr& camera_system, const vi_map::Imu& imu_sensor)
      : tracking_pipeline_(camera_system, imu_sensor) {
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

          LOG(INFO) << "FeatureTrackingFlow: Received nframe with ts "
                    << nframe_imu->nframe->getMinTimestampNanoseconds();
          const bool success =
              this->tracking_pipeline_.trackSynchronizedNFrameImuCallback(
                  nframe_imu);
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
};

}  // namespace openvinsli

#endif  // OPENVINSLI_FEATURE_TRACKING_FLOW_H_
