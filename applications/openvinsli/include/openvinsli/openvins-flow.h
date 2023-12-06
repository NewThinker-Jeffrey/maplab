#ifndef OPENVINSLI_OPENVINS_FLOW_H_
#define OPENVINSLI_OPENVINS_FLOW_H_

#include <functional>
#include <memory>
#include <vector>

#include <maplab-common/bidirectional-map.h>
#include <message-flow/message-flow.h>
#include <sensors/imu.h>
#include <vio-common/vio-types.h>

#include "openvinsli/openvins-estimate.h"
#include "openvinsli/openvins-factory.h"
#include "openvinsli/openvins-health-monitor.h"
#include "openvinsli/openvins-maplab-timetranslation.h"

#include "core/VioManager.h"         // ov_msckf
#include "openvinsli/viewer.h"       
namespace openvinsli {
class OpenvinsLocalizationHandler;
class Nav2dFlow;

class OpenvinsFlow {
 public:
  explicit OpenvinsFlow(
      const aslam::NCamera& camera_calibration,
      const vi_map::ImuSigmas& imu_sigmas,
      const aslam::Transformation& odom_calibration);
  ~OpenvinsFlow();

  void attachToMessageFlow(message_flow::MessageFlow* flow);

  void processAndPublishOpenvinsUpdate(const ov_msckf::VioManager::Output& output);

  // visualization for nav
  void setNavForViewer(Nav2dFlow* nav) {
    if (gl_viewer_) {
      gl_viewer_->setNav(nav);
    }
  }

 private:
  std::unique_ptr<ov_msckf::VioManager> openvins_interface_;

  // gl visualization
  std::shared_ptr<OpenvinsliViewer> gl_viewer_;
  std::shared_ptr<std::thread> vis_thread_;
  std::atomic<bool> stop_viz_request_;
  double last_visualization_timestamp_ = 0;



  int openvins_num_cameras_;
  std::function<void(const OpenvinsEstimate::ConstPtr&)> publish_openvins_estimates_;

  OpenvinsMaplabTimeTranslation time_translation_;
  // OpenvinsHealthMonitor health_monitor_;

  // A camera without a mapping is not being used for motion tracking in OPENVINS.
  common::BidirectionalMap<size_t, size_t> maplab_to_openvins_cam_indices_mapping_;

  std::unique_ptr<OpenvinsLocalizationHandler> localization_handler_;

  // External OPENVINS odometry calibration
  aslam::Transformation odom_calibration_;
};
}  // namespace openvinsli
#endif  // OPENVINSLI_OPENVINS_FLOW_H_
