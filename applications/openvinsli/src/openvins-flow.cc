#include "openvinsli/openvins-flow.h"

#include <memory>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <aslam/common/pose-types.h>
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
#include "openvinsli/openvins-health-monitor.h"
#include "openvinsli/openvins-localization-handler.h"
#include "openvinsli/openvins-maplab-timetranslation.h"

DEFINE_bool(
    openvins_update_filter_on_imu, true,
    "Update the filter state for IMU measurement; if false the IMU measurements"
    " are queued and the state is only forward propagated before the next "
    "update.");
DEFINE_string(
    openvins_active_camera_indices, "0",
    "Comma separated indices of cameras to use for motion tracking.");
DEFINE_bool(
    openvinsli_enable_health_checking, false,
    "Perform health checking on the estimator output and reset if necessary.");

namespace openvinsli {
OpenvinsFlow::OpenvinsFlow(
    const aslam::NCamera& camera_calibration,
    const vi_map::ImuSigmas& imu_sigmas,
    const aslam::Transformation& odom_calibration) {
  // Multi-camera support in OPENVINS is still experimental. Therefore, only a
  // single camera will be used for motion tracking per default.
  const size_t num_cameras = camera_calibration.getNumCameras();
  LOG_IF(WARNING, num_cameras > 1u)
      << "Multi-camera support of OPENVINS is still experimental. Per default "
      << "only the first camera will be used for motion tracking. However, all "
      << "cameras will be used for mapping and localization. You can override  "
      << "the active used for motion tracking with the flag: "
      << "--openvins_active_camera_indices";

  CHECK(!FLAGS_openvins_active_camera_indices.empty());
  constexpr char kDelimiter = ',';
  constexpr bool kRemoveEmpty = true;
  std::vector<std::string> tokens;
  common::tokenizeString(
      FLAGS_openvins_active_camera_indices, kDelimiter, kRemoveEmpty, &tokens);
  LOG_IF(WARNING, tokens.size() > 1u)
      << "Selected more than one camera for motion tracking. Consider only "
      << "using a single camera if latency issues develop.";

  // Build NCamera of active cameras.
  int openvins_camera_index = 0;
  std::vector<aslam::Camera::Ptr> active_cameras;
  aslam::TransformationVector active_T_C_Bs;
  for (const std::string& camera_id_str : tokens) {
    const int maplab_camera_idx = std::stoi(camera_id_str);
    CHECK_GE(maplab_camera_idx, 0);
    CHECK_LT(maplab_camera_idx, static_cast<int>(num_cameras));
    CHECK(maplab_to_openvins_cam_indices_mapping_.insert(
        maplab_camera_idx, openvins_camera_index))
        << "--openvins_active_camera_indices contains duplicates.";

    active_cameras.emplace_back(
        camera_calibration.getCameraShared(maplab_camera_idx)->clone());
    active_T_C_Bs.emplace_back(camera_calibration.get_T_C_B(maplab_camera_idx));
    ++openvins_camera_index;
  }
  CHECK_EQ(static_cast<int>(active_cameras.size()), openvins_camera_index);
  CHECK_EQ(static_cast<int>(active_T_C_Bs.size()), openvins_camera_index);

  aslam::NCameraId id;
  aslam::generateId<aslam::NCameraId>(&id);
  aslam::NCamera motion_tracking_ncamera(
      id, active_T_C_Bs, active_cameras, "Cameras active for motion tracking.");

  // Construct OPENVINS interface using only the active cameras.
  openvins_interface_.reset(
      constructAndConfigureOpenvins(motion_tracking_ncamera, imu_sigmas));
  openvins_interface_->setEnablePatchUpdateOutput(false);
  openvins_interface_->setEnableFeatureUpdateOutput(true);  // For health checking.
  localization_handler_.reset(new OpenvinsLocalizationHandler(
      openvins_interface_.get(), &time_translation_, camera_calibration,
      maplab_to_openvins_cam_indices_mapping_));

  // Store external OPENVINS odometry calibration
  odom_calibration_ = odom_calibration;
}

OpenvinsFlow::~OpenvinsFlow() {}

void OpenvinsFlow::attachToMessageFlow(message_flow::MessageFlow* flow) {
  CHECK_NOTNULL(flow);
  static constexpr char kSubscriberNodeName[] = "OpenvinsFlow";

  // All data input subscribers are put in an exclusivity group such that the
  // delivery ordering for all messages (cam, imu, localization) are
  // corresponding to the publishing order and no sensor can be left behind.
  message_flow::DeliveryOptions openvins_subscriber_options;
  openvins_subscriber_options.exclusivity_group_id =
      kExclusivityGroupIdOpenvinsSensorSubscribers;

  // Input IMU.
  flow->registerSubscriber<message_flow_topics::IMU_MEASUREMENTS>(
      kSubscriberNodeName, openvins_subscriber_options,
      [this](const vio::ImuMeasurement::ConstPtr& imu) {
        // Do not apply the predictions but only queue them. They will be
        // applied before the next update.
        const double openvins_timestamp_sec =
            time_translation_.convertMaplabToOpenvinsTimestamp(imu->timestamp);
        const bool measurement_accepted =
            this->openvins_interface_->processImuUpdate(
                imu->imu_data.head<3>(), imu->imu_data.tail<3>(),
                openvins_timestamp_sec, FLAGS_openvins_update_filter_on_imu);
        LOG_IF(
            WARNING, !measurement_accepted && openvins_interface_->isInitialized())
            << "OPENVINS rejected IMU measurement. Latency is too large.";

        localization_handler_->T_M_I_buffer_mutable()->bufferImuMeasurement(
            *imu);
      });

  // Input Odometry.
  flow->registerSubscriber<message_flow_topics::ODOMETRY_MEASUREMENTS>(
      kSubscriberNodeName, openvins_subscriber_options,
      [this](const vio::OdometryMeasurement::ConstPtr& odometry) {
        const Eigen::Vector3d& t_I_O = odom_calibration_.getPosition();
        const Eigen::Matrix3d& R_I_O = odom_calibration_.getRotationMatrix();

        Eigen::Vector3d velocity_linear_I =
            R_I_O * odometry->velocity_linear_O -
            (R_I_O * odometry->velocity_angular_O).cross(t_I_O);

        const double openvins_timestamp_sec =
            time_translation_.convertMaplabToOpenvinsTimestamp(
                odometry->timestamp);
        const bool measurement_accepted =
            this->openvins_interface_->processVelocityUpdate(
                velocity_linear_I, openvins_timestamp_sec);
        LOG_IF(
            WARNING, !measurement_accepted && openvins_interface_->isInitialized())
            << "OPENVINS rejected Odometry measurement. Latency is too large.";
      });

  // Input camera.
  flow->registerSubscriber<message_flow_topics::IMAGE_MEASUREMENTS>(
      kSubscriberNodeName, openvins_subscriber_options,
      [this](const vio::ImageMeasurement::ConstPtr& image) {
        const size_t maplab_cam_idx = image->camera_index;
        const size_t* openvins_cam_index =
            maplab_to_openvins_cam_indices_mapping_.getRight(maplab_cam_idx);
        if (openvins_cam_index == nullptr) {
          // Skip this image, as the camera was marked as inactive.
          return;
        }

        const double openvins_timestamp_sec =
            time_translation_.convertMaplabToOpenvinsTimestamp(image->timestamp);
        const bool measurement_accepted =
            this->openvins_interface_->processImageUpdate(
                *openvins_cam_index, image->image, openvins_timestamp_sec);
        LOG_IF(
            WARNING, !measurement_accepted && openvins_interface_->isInitialized())
            << "OPENVINS rejected image measurement of camera " << maplab_cam_idx
            << " (openvins cam idx: " << *openvins_cam_index << ") at time "
            << aslam::time::timeNanosecondsToString(image->timestamp)
            << ". Latency is too large.";
      });
  // Input localization updates.
  flow->registerSubscriber<message_flow_topics::LOCALIZATION_RESULT>(
      kSubscriberNodeName, openvins_subscriber_options,
      std::bind(
          &OpenvinsLocalizationHandler::processLocalizationResult,
          localization_handler_.get(), std::placeholders::_1));

  // Output OPENVINS estimates.
  publish_openvins_estimates_ =
      flow->registerPublisher<message_flow_topics::OPENVINS_ESTIMATES>();
  CHECK(openvins_interface_);
  openvins_interface_->registerStateUpdateCallback(std::bind(
      &OpenvinsFlow::processAndPublishOpenvinsUpdate, this, std::placeholders::_1));
}

void OpenvinsFlow::processAndPublishOpenvinsUpdate(const openvins::OpenvinsState& state) {
  if (!state.getIsInitialized()) {
    LOG(WARNING) << "OPENVINS not yet initialized. Discarding state update.";
    return;
  }

  if (FLAGS_openvinsli_enable_health_checking &&
      health_monitor_.shouldResetEstimator(state)) {
    health_monitor_.resetOpenvinsToLastHealthyPose(openvins_interface_.get());
    return;
  }

  // OPENVINS coordinate frames:
  //  - I: Inertial frame of pose update
  //  - V: Body frame of pose update sensor
  //  - W: Inertial frame of odometry
  //  - B: IMU-coordinate frame
  // OPENVINS and maplab both use passive Hamilton quaternion convention; no
  // conversion is necessary.
  aslam::Transformation T_M_I(
      state.get_qBW().inverted().toImplementation(), state.get_WrWB());
  common::ensurePositiveQuaternion(&T_M_I.getRotation());
  const Eigen::Vector3d v_M = T_M_I.getRotation().rotate(state.get_BvB());

  OpenvinsEstimate::Ptr openvins_estimate(new OpenvinsEstimate);
  // VIO states.
  const int64_t timestamp_ns =
      time_translation_.convertOpenvinsToMaplabTimestamp(state.getTimestamp());
  openvins_estimate->timestamp_ns = timestamp_ns;
  openvins_estimate->vinode.setTimestamp(timestamp_ns);
  openvins_estimate->vinode.set_T_M_I(T_M_I);
  openvins_estimate->vinode.set_v_M_I(v_M);
  openvins_estimate->vinode.setAccBias(state.getAcb());
  openvins_estimate->vinode.setGyroBias(state.getGyb());
  openvins_estimate->vinode.setCovariancesFromOpenvinsMatrix(
      state.getImuCovariance());

  // Camera extrinsics.
  for (size_t openvins_cam_idx = 0u; openvins_cam_idx < state.numCameras();
       ++openvins_cam_idx) {
    const size_t* maplab_cam_idx =
        maplab_to_openvins_cam_indices_mapping_.getLeft(openvins_cam_idx);
    CHECK_NOTNULL(maplab_cam_idx);

    aslam::Transformation T_B_C(
        state.get_qCM(openvins_cam_idx).inverted().toImplementation(),
        state.get_MrMC(openvins_cam_idx));
    common::ensurePositiveQuaternion(&T_B_C.getRotation());
    CHECK(openvins_estimate->maplab_camera_index_to_T_C_B
              .emplace(*maplab_cam_idx, T_B_C.inverse())
              .second);
  }

  // Optional localizations.
  openvins_estimate->has_T_G_M =
      extractLocalizationFromOpenvinsState(state, &openvins_estimate->T_G_M);
  localization_handler_->T_M_I_buffer_mutable()->bufferOdometryEstimate(
      openvins_estimate->vinode);
  if (openvins_estimate->has_T_G_M) {
    localization_handler_->buffer_T_G_M(openvins_estimate->T_G_M);
  }

  publish_openvins_estimates_(openvins_estimate);
}
}  // namespace openvinsli
