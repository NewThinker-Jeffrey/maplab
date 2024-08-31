#ifndef OPENVINSLI_OPENVINS_FLOW_H_
#define OPENVINSLI_OPENVINS_FLOW_H_

#include <functional>
#include <memory>
#include <vector>

#include <maplab-common/bidirectional-map.h>
#include <message-flow/message-flow.h>
#include <sensors/imu.h>
#include <vio-common/vio-types.h>

#include "openvinsli/flow-topics.h"
#include "openvinsli/openvins-estimate.h"
#include "openvinsli/openvins-factory.h"
#include "openvinsli/openvins-health-monitor.h"
#include "openvinsli/openvins-maplab-timetranslation.h"

#include "core/VioManager.h"         // ov_msckf
#include "openvinsli/viewer.h"       
#include "hear_slam/vtag/vtag_factory.h"
#include "hear_slam/basic/work_queue.h"

namespace openvinsli {
class OpenvinsLocalizationHandler;

class OpenvinsFlow {
 public:
  explicit OpenvinsFlow(
      const aslam::NCamera& camera_calibration,
      const vi_map::ImuSigmas& imu_sigmas,
      const aslam::Transformation& odom_calibration);
  ~OpenvinsFlow();

  void attachToMessageFlow(message_flow::MessageFlow* flow);

  void processAndPublishOpenvinsUpdate(const ov_msckf::VioManager::Output& output);

  ov_msckf::VioManager* openvinsInterface() const {
    return openvins_interface_.get();
  }

 private:
  void processTag(ov_core::CameraData cam);

 private:
  std::unique_ptr<ov_msckf::VioManager> openvins_interface_;
  OpenvinsEstimate::Ptr openvins_estimate_;
  
  int openvins_num_cameras_;
  std::function<void(const OpenvinsEstimate::ConstPtr&)> publish_openvins_estimates_;

  OpenvinsMaplabTimeTranslation time_translation_;
  // OpenvinsHealthMonitor health_monitor_;

  // A camera without a mapping is not being used for motion tracking in OPENVINS.
  common::BidirectionalMap<size_t, size_t> maplab_to_openvins_cam_indices_mapping_;

  std::unique_ptr<OpenvinsLocalizationHandler> localization_handler_;

  // External OPENVINS odometry calibration
  aslam::Transformation odom_calibration_;

  // camera synchronizing
  using ImageQueue = std::deque<vio::ImageMeasurement::ConstPtr>;
  std::map<size_t, ImageQueue> cam_id_to_image_queue_;

  // vtag

  std::shared_ptr<hear_slam::TagDetectorInterface> vtag_detector_;
  // hear_slam::SimpleCameraParams simple_camera_params_;
  std::shared_ptr<hear_slam::WorkQueue<ov_core::CameraData>> vtag_work_queue_;

  std::function<void(const StampedTagDetections::ConstPtr&)> publish_tag_detections_;
};
}  // namespace openvinsli
#endif  // OPENVINSLI_OPENVINS_FLOW_H_
