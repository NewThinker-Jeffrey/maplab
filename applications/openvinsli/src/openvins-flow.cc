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

#include "openvinsli/mini-nav2d-flow.h"


#include "core/VioManager.h"         // ov_msckf
#include "core/VioManagerOptions.h"  // ov_msckf
#include "state/Propagator.h"        // ov_msckf
#include "state/State.h"             // ov_msckf
#include "state/StateHelper.h"       // ov_msckf
#include "utils/print.h"             // ov_core
#include "utils/sensor_data.h"       // ov_core

// DEFINE_bool(
//     openvins_update_filter_on_imu, true,
//     "Update the filter state for IMU measurement; if false the IMU measurements"
//     " are queued and the state is only forward propagated before the next "
//     "update.");
// DEFINE_string(
//     openvins_active_camera_indices, "0",
//     "Comma separated indices of cameras to use for motion tracking.");
DEFINE_bool(
    openvinsli_enable_health_checking, false,
    "Perform health checking on the estimator output and reset if necessary.");

namespace openvinsli {
OpenvinsFlow::OpenvinsFlow(
    const aslam::NCamera& camera_calibration,
    const vi_map::ImuSigmas& imu_sigmas,
    const aslam::Transformation& odom_calibration) {
  const size_t num_cameras = camera_calibration.getNumCameras();
  LOG_IF(WARNING, num_cameras > 2u)
      << "OPENVINS supports only 1 or 2 (for stereo) cameras. "
         "If more than 2 are provided, we only use the first camera.";
  
  openvins_num_cameras_ = (num_cameras > 2u ? 1 : num_cameras);
  // Build NCamera of active cameras.
  int openvins_camera_index = 0;
  std::vector<aslam::Camera::Ptr> active_cameras;
  aslam::TransformationVector active_T_C_Bs;
  for (size_t maplab_camera_idx = 0; maplab_camera_idx < openvins_num_cameras_; maplab_camera_idx ++) {
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

  localization_handler_.reset(new OpenvinsLocalizationHandler(
      openvins_interface_.get(), &time_translation_, camera_calibration,
      maplab_to_openvins_cam_indices_mapping_));

  // Store external OPENVINS odometry calibration
  odom_calibration_ = odom_calibration;
}

OpenvinsFlow::~OpenvinsFlow() {
}

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
        ov_core::ImuData ov_imu;
        ov_imu.timestamp =
            time_translation_.convertMaplabToOpenvinsTimestamp(imu->timestamp);
        vio::ImuData data = imu->imu_data;
        ov_imu.am << data[0], data[1], data[2];
        ov_imu.wm << data[3], data[4], data[5];
        openvins_interface_->feed_measurement_imu(ov_imu);

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
        // // todo(jeffrey): Maybe we'll deal with odometry input later.
        // const bool measurement_accepted =
        //     this->openvins_interface_->processVelocityUpdate(
        //         velocity_linear_I, openvins_timestamp_sec);
        // LOG_IF(
        //     WARNING, !measurement_accepted && openvins_interface_->isInitialized())
        //     << "OPENVINS rejected Odometry measurement. Latency is too large.";
      });

  // Input camera.
  // flow->registerSubscriber<message_flow_topics::IMAGE_MEASUREMENTS>(
  //     kSubscriberNodeName, openvins_subscriber_options,
  //     [this](const vio::ImageMeasurement::ConstPtr& image) {
  //       const size_t maplab_cam_idx = image->camera_index;
  //       const size_t* openvins_cam_index =
  //           maplab_to_openvins_cam_indices_mapping_.getRight(maplab_cam_idx);
  //       if (openvins_cam_index == nullptr) {
  //         // Skip this image, as the camera was marked as inactive.
  //         return;
  //       }

  //       const double openvins_timestamp_sec =
  //           time_translation_.convertMaplabToOpenvinsTimestamp(image->timestamp);
  //       const bool measurement_accepted =
  //           this->openvins_interface_->processImageUpdate(
  //               *openvins_cam_index, image->image, openvins_timestamp_sec);
  //       LOG_IF(
  //           WARNING, !measurement_accepted && openvins_interface_->isInitialized())
  //           << "OPENVINS rejected image measurement of camera " << maplab_cam_idx
  //           << " (openvins cam idx: " << *openvins_cam_index << ") at time "
  //           << aslam::time::timeNanosecondsToString(image->timestamp)
  //           << ". Latency is too large.";
  //     });

  // We need synced images (while imu is not required)
  flow->registerSubscriber<message_flow_topics::SYNCED_NFRAMES_AND_IMU>(
      kSubscriberNodeName, openvins_subscriber_options,
      [this](const vio::SynchronizedNFrameImu::Ptr& nframe_imu) {
        CHECK(nframe_imu);
        aslam::VisualNFrame::Ptr nframe = nframe_imu->nframe;
        CHECK(nframe);
        LOG(INFO) << "OpenvinsFlow: Received nframe with ts "
                  << nframe->getMinTimestampNanoseconds();

        std::map<size_t, cv::Mat> ov_idx_to_img;
        for (size_t i=0; i<nframe->getNumFrames(); i++) {
          const size_t* openvins_cam_index =
              maplab_to_openvins_cam_indices_mapping_.getRight(i);
          if (openvins_cam_index == nullptr) {
            // Skip this image, as the camera was marked as inactive.
            continue;
          }
          const aslam::VisualFrame& frame = nframe->getFrame(i);
          ov_idx_to_img[*openvins_cam_index] = frame.getRawImage();
        }
        CHECK_EQ(openvins_num_cameras_, ov_idx_to_img.size());

        ov_core::CameraData cam;
        cam.timestamp =
            time_translation_.convertMaplabToOpenvinsTimestamp(
                nframe->getMinTimestampNanoseconds());
        for (const auto& item : ov_idx_to_img) {
          cam.sensor_ids.push_back(item.first);
          cam.images.push_back(item.second);
          int rows = item.second.rows;
          int cols = item.second.cols;
          cam.masks.push_back(cv::Mat::zeros(rows, cols, CV_8UC1));
        }
        openvins_interface_->feed_measurement_camera(cam);
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
  openvins_interface_->setUpdateCallback(std::bind(
      &OpenvinsFlow::processAndPublishOpenvinsUpdate, this, std::placeholders::_1));
}

void OpenvinsFlow::processAndPublishOpenvinsUpdate(const ov_msckf::VioManager::Output& output) {
  if (!output.status.initialized) {
    LOG(WARNING) << "OPENVINS not yet initialized. Discarding state update.";
    return;
  }

  // todo(jeffrey): support healthy check.
  // if (FLAGS_openvinsli_enable_health_checking &&
  //     health_monitor_.shouldResetEstimator(state)) {
  //   health_monitor_.resetOpenvinsToLastHealthyPose(openvins_interface_.get());
  //   return;
  // }

  aslam::Transformation T_M_I(
      output.state_clone->_imu->pos(),
      Eigen::Quaterniond(output.state_clone->_imu->Rot().inverse()));
  common::ensurePositiveQuaternion(&T_M_I.getRotation());
  const Eigen::Vector3d v_M = output.state_clone->_imu->vel();
  OpenvinsEstimate::Ptr openvins_estimate(new OpenvinsEstimate);
  // VIO states.
  const int64_t timestamp_ns =
      time_translation_.convertOpenvinsToMaplabTimestamp(output.status.timestamp);
  openvins_estimate->timestamp_ns = timestamp_ns;
  openvins_estimate->vinode.setTimestamp(timestamp_ns);
  openvins_estimate->vinode.set_T_M_I(T_M_I);
  openvins_estimate->vinode.set_v_M_I(v_M);
  openvins_estimate->vinode.setAccBias(output.state_clone->_imu->bias_a());
  openvins_estimate->vinode.setGyroBias(output.state_clone->_imu->bias_g());

  // todo(jeffrey): check and modify the covariance.
  //    The definition to the errors are still vague.
  //    I'm not sure for now whether the errors of 6DoF velocity & 6DoF pose
  //    should be represented in the 'M' frame or in the 'I' frame (local), and 
  //    whether the error of quaternion should be viewed as a left perturbation
  //    or a right one.
  //    And we may need some transformation (maybe someting like SO3.adj()) to the
  //    JPL-quaternion's error.
  std::vector<std::shared_ptr<ov_type::Type>> pose_variables = {
    output.state_clone->_imu->p(),  // p first or q first??
    output.state_clone->_imu->q()
  };
  std::vector<std::shared_ptr<ov_type::Type>> twist_variables = {
    output.state_clone->_imu->v(),  // v first or w first??
    output.state_clone->_imu->bg()
  };
  Eigen::MatrixXd cov_pose = ov_msckf::StateHelper::get_marginal_covariance(
      output.state_clone, pose_variables);
  Eigen::MatrixXd cov_twist = ov_msckf::StateHelper::get_marginal_covariance(
      output.state_clone, twist_variables);
  
  auto openvins_params = openvins_interface_->get_params();
  cov_twist.block<3,3>(3,3) +=
      openvins_params.imu_noises.sigma_w_2 * Eigen::Matrix3d::Identity();
  openvins_estimate->vinode.setPoseCovariance(cov_pose);
  openvins_estimate->vinode.setTwistCovariance(cov_twist);

  // Camera extrinsics.
  for (size_t openvins_cam_idx = 0u; openvins_cam_idx < openvins_num_cameras_;
       ++openvins_cam_idx) {
    const size_t* maplab_cam_idx =
        maplab_to_openvins_cam_indices_mapping_.getLeft(openvins_cam_idx);
    CHECK_NOTNULL(maplab_cam_idx);

    // todo(jeffrey): retrieve the updated extrisincs from output.state_clone.
    aslam::Transformation T_B_C(
        openvins_params.T_CtoIs.at(openvins_cam_idx).block<3,1>(0,3),
        Eigen::Quaterniond(openvins_params.T_CtoIs.at(openvins_cam_idx).block<3,3>(0,0)));
    // aslam::Transformation T_B_C(
    //     openvins_params.T_CtoIs.at(openvins_cam_idx).block<3,3>(0,0),
    //     openvins_params.T_CtoIs.at(openvins_cam_idx).block<3,1>(0,3));
    common::ensurePositiveQuaternion(&T_B_C.getRotation());
    CHECK(openvins_estimate->maplab_camera_index_to_T_C_B
              .emplace(*maplab_cam_idx, T_B_C.inverse())
              .second);
  }

  // Optional localizations.
  openvins_estimate->has_T_G_M =
      extractLocalizationFromOpenvinsState(output, &openvins_estimate->T_G_M);
  localization_handler_->T_M_I_buffer_mutable()->bufferOdometryEstimate(
      openvins_estimate->vinode);
  if (openvins_estimate->has_T_G_M) {
    localization_handler_->buffer_T_G_M(openvins_estimate->T_G_M);
  }
  localization_handler_->dealWithBufferedLocalizations();

  publish_openvins_estimates_(openvins_estimate);
}
}  // namespace openvinsli
