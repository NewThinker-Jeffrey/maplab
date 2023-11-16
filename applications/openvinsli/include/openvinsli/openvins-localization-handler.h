#ifndef OPENVINSLI_OPENVINS_LOCALIZATION_HANDLER_H_
#define OPENVINSLI_OPENVINS_LOCALIZATION_HANDLER_H_

#include "openvinsli/openvins-flow.h"

#include <memory>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <aslam/common/pose-types.h>
#include <aslam/common/stl-helpers.h>
#include <aslam/common/time.h>
#include <aslam/common/unique-id.h>
#include <gflags/gflags.h>
#include <maplab-common/fixed-size-queue.h>
#include <maplab-common/geometry.h>
#include <maplab-common/string-tools.h>
#include <message-flow/message-flow.h>
#include <vio-common/pose-lookup-buffer.h>
#include <vio-common/vio-types.h>

#include "openvinsli/flow-topics.h"
#include "openvinsli/openvins-factory.h"
#include "openvinsli/openvins-maplab-timetranslation.h"

#include "core/VioManager.h"         // ov_msckf

namespace openvinsli {

// Logic and state machine to feed localization constraints to OPENVINS.
class OpenvinsLocalizationHandler {
 public:
  OpenvinsLocalizationHandler(
      ov_msckf::VioManager* openvins_interface,
      OpenvinsMaplabTimeTranslation* time_translator,
      const aslam::NCamera& camera_calibration,
      const common::BidirectionalMap<size_t, size_t>&
          maplab_to_openvins_cam_indices_mapping);

  void processLocalizationResult(
      const vio::LocalizationResult::ConstPtr& localization_result);

  vio_common::PoseLookupBuffer* T_M_I_buffer_mutable() {
    return &T_M_I_buffer_;
  }

  void buffer_T_G_M(const pose::Transformation& T_G_M_filter) {
    std::lock_guard<std::mutex> lock(m_T_G_M_filter_buffer_);
    T_G_M_filter_buffer_.insert(T_G_M_filter);
  }

 private:
  bool initializeBaseframe(
      const vio::LocalizationResult::ConstPtr& localization_result);
  bool processAsUpdate(
      const vio::LocalizationResult::ConstPtr& localization_result);

  // Returns the ratio of successfully reprojected matches.
  double getLocalizationReprojectionErrors(
      const vio::LocalizationResult& localization_result,
      const aslam::Transformation& T_M_I_filter,
      std::vector<double>* lc_reprojection_errors,
      std::vector<double>* filter_reprojection_errors);

  ov_msckf::VioManager* const openvins_interface_;
  OpenvinsMaplabTimeTranslation* const time_translator_;

  common::LocalizationState localization_state_;

  static constexpr int64_t kBufferPoseHistoryNs = aslam::time::seconds(5);
  static constexpr int64_t kBufferMaxPropagationNs =
      aslam::time::milliseconds(500);
  vio_common::PoseLookupBuffer T_M_I_buffer_;

  static constexpr size_t kFilterBaseframeBufferSize = 1u;
  common::FixedSizeQueue<aslam::Transformation> T_G_M_filter_buffer_;
  std::mutex m_T_G_M_filter_buffer_;

  common::FixedSizeQueue<aslam::Transformation> T_G_M_loc_buffer_;

  const aslam::NCamera& camera_calibration_;

  const common::BidirectionalMap<size_t, size_t>&
      maplab_to_openvins_cam_indices_mapping_;

  static constexpr size_t kInitializationMaxNumRansacIterations = 3u;
  static constexpr double kInitializationRansacPositionErrorThresholdMeters =
      5.0;
  static constexpr double
      kInitializationRansacOrientationErrorThresholdRadians = 20.0 * kDegToRad;
  static constexpr double kInitializationRansacInlierRatioThreshold = 0.75;
};

bool extractLocalizationFromOpenvinsState(
    const ov_msckf::VioManager::Output& output, aslam::Transformation* T_G_M);

}  //  namespace openvinsli

#endif  // OPENVINSLI_OPENVINS_LOCALIZATION_HANDLER_H_
