#include "openvinsli/openvins-flow.h"

#include <memory>
#include <random>
#include <string>
#include <vector>
#include <filesystem>

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


#include "core/SimpleDenseMapping.h"      // ov_msckf
#include "core/VioManager.h"         // ov_msckf
#include "core/VioManagerOptions.h"  // ov_msckf
#include "state/Propagator.h"        // ov_msckf
#include "state/State.h"             // ov_msckf
#include "state/StateHelper.h"       // ov_msckf
#include "utils/print.h"             // ov_core
#include "utils/sensor_data.h"       // ov_core
#include "cam/CamEqui.h"
#include "cam/CamRadtan.h"

#include "hear_slam/basic/time.h"
#include "hear_slam/basic/logging.h"
#include "hear_slam/common/camera_models/camera_model_factory.h"
#include "hear_slam/vtag/vtag_mapping.h"

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

DEFINE_bool(
    openvinsli_enable_vtag, true,
    "Enable vtag or not.");

DEFINE_bool(
    openvinsli_visualize_vtag, true,
    "Visualize vtag or not.");

DEFINE_string(
    openvinsli_tag_map, "",
    "Path to the tag map file.");

DEFINE_bool(
    use_zeroed_openvins_time, false,
    "OPENVINS uses seconds (double) and maplab nanoseconds (int64_t) as "
    "timestamps. To reduce precision loss the timestamps can be zeroed using the "
    "first timestamp of maplab before conversion.");

DEFINE_bool(
    openvinsli_save_feature_images, false,
    "Whether to save feature tracking images.");

DEFINE_string(
    openvinsli_feature_images_dir, "tmp_feature_images",
    "dir to save feature tracking images.");

#define DISABLE_OPENVINS_RELOCAL

namespace openvinsli {
OpenvinsFlow::OpenvinsFlow(
    const aslam::NCamera& camera_calibration,
    const vi_map::ImuSigmas& imu_sigmas,
    const aslam::Transformation& odom_calibration) :
    time_translation_(FLAGS_use_zeroed_openvins_time) {
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

#ifndef DISABLE_OPENVINS_RELOCAL
  localization_handler_.reset(new OpenvinsLocalizationHandler(
      openvins_interface_.get(), &time_translation_, camera_calibration,
      maplab_to_openvins_cam_indices_mapping_));
#endif

  // Store external OPENVINS odometry calibration
  odom_calibration_ = odom_calibration;

  if (FLAGS_openvinsli_save_feature_images) {
    std::filesystem::path dir_path(FLAGS_openvinsli_feature_images_dir);
    if (std::filesystem::create_directory(dir_path)) {
        std::cout << "feature_images_dir created successfully." << std::endl;
    } else {
        std::cout << "feature_images_dir '" <<  FLAGS_openvinsli_feature_images_dir << "' already exists or cannot be created!!" << std::endl;
    }
  }

  global_pose_fusion_ = std::make_unique<hear_slam::GlobalPoseFusion>();
  global_pose_fusion_->setPoseUpdateCallback(
      [this](double time, const hear_slam::GlobalPoseFusion::Pose3d& pose, const Eigen::Matrix<double, 6, 6>& cov) {
        std::cout << "Fusion localization pose: p(" << pose.translation().vector().transpose() << ")  q("
                  << Eigen::Quaterniond(pose.linear().matrix()).coeffs().transpose() << ")" << std::endl;
        std::cout << "Fusion localization cov diag sqrt: " << cov.diagonal().cwiseSqrt().transpose() << std::endl;

        // publish pose
        auto fusion_res = std::make_shared<StampedGlobalPose>();
        fusion_res->timestamp_ns = time_translation_.convertOpenvinsToMaplabTimestamp(time);
        fusion_res->pose = Eigen::Isometry3d(pose.linear().matrix());
        fusion_res->pose.translation() = pose.translation().vector();
        publish_global_pose_fusion_(fusion_res);
      });

  if (FLAGS_openvinsli_enable_vtag) {
    vtag_detector_ = hear_slam::TagDetectorFactory::createDetector(hear_slam::TagType::April);
    vtag_work_queue_ = std::make_shared<hear_slam::WorkQueue<ov_core::CameraData>>(
        std::bind(&OpenvinsFlow::processTag, this, std::placeholders::_1), "vtag_work_queue", 1, 2, true);
    if (!FLAGS_openvinsli_tag_map.empty()) {
      tag_map_ = hear_slam::TagMapping::loadTagStates(FLAGS_openvinsli_tag_map);
      if (tag_map_.size() > 0) {
        for (const auto& item : tag_map_) {
          const auto& tag_id = item.first;
          const auto& tag_pose = item.second;
          Eigen::Quaterniond q(tag_pose.R);
          std::cout << "Loaded tag " << hear_slam::toStr(tag_id) << ": p = " << tag_pose.p.transpose() << ", q = " << q.coeffs().transpose() << std::endl;
        }
      }
    }
  }
}

OpenvinsFlow::~OpenvinsFlow() {
  if (vtag_work_queue_) {
    vtag_work_queue_->stop();
  }
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
        hear_slam::TimeCounter tc;
        // Do not apply the predictions but only queue them. They will be
        // applied before the next update.
        ov_core::ImuData ov_imu;
        ov_imu.timestamp =
            time_translation_.convertMaplabToOpenvinsTimestamp(imu->timestamp);
        vio::ImuData data = imu->imu_data;
        ov_imu.am << data[0], data[1], data[2];
        ov_imu.wm << data[3], data[4], data[5];
        openvins_interface_->feed_measurement_imu(ov_imu);

#ifndef DISABLE_OPENVINS_RELOCAL
        localization_handler_->T_M_I_buffer_mutable()->bufferImuMeasurement(
            *imu);
#endif

        double eplased_ms = tc.elapsed().millis();
        if (eplased_ms > 1) {
          LOGI("OpenvinsFlow.IMU: cost time %.3f ms, timestamp %.3f s", eplased_ms, ov_imu.timestamp);
        }
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
  flow->registerSubscriber<message_flow_topics::IMAGE_MEASUREMENTS>(
      kSubscriberNodeName, openvins_subscriber_options,
      [this](const vio::ImageMeasurement::ConstPtr& image) {
        size_t openvins_cam_id = 0;
        auto openvins_params = openvins_interface_->get_params();
        if (image->camera_index < 0 && openvins_params.state_options.use_rgbd) {
          // we use negative camera_index for depth images.
          openvins_cam_id = 1;  // we fix the depth cam id to be 1.
        } else {
          const size_t maplab_cam_idx = image->camera_index;
          const size_t* p_openvins_cam_index =
              maplab_to_openvins_cam_indices_mapping_.getRight(maplab_cam_idx);
          
          if (p_openvins_cam_index == nullptr) {
            // Skip this image, as the camera was marked as inactive.
            return;
          }
          openvins_cam_id = *p_openvins_cam_index;
        }

        // we only support three settings in openvins: mono, stereo, rgbd.
        assert(openvins_cam_id == 0 || openvins_cam_id == 1);
        cam_id_to_image_queue_[openvins_cam_id].push_back(image);

        // try synchronizing
        const int64_t MAX_PRECISION_NS = 10;  // Maybe 1 is enough.
        if (openvins_params.state_options.num_cameras == 2 || openvins_params.state_options.use_rgbd) {
          while (!cam_id_to_image_queue_[0].empty() && !cam_id_to_image_queue_[1].empty()) {
            hear_slam::TimeCounter tc;
            vio::ImageMeasurement::ConstPtr image0 = cam_id_to_image_queue_[0].front();
            vio::ImageMeasurement::ConstPtr image1 = cam_id_to_image_queue_[1].front();
            if (abs(image0->timestamp - image1->timestamp) > MAX_PRECISION_NS) {
              if (image0->timestamp < image1->timestamp) {
                cam_id_to_image_queue_[0].pop_front();
              } else {
                cam_id_to_image_queue_[1].pop_front();
              }
              continue;
            }

            ov_core::CameraData cam;
            cam.timestamp = time_translation_.convertMaplabToOpenvinsTimestamp(image0->timestamp);

            cam.sensor_ids.push_back(0);
            cam.images.push_back(image0->image);
            cam.masks.push_back(cv::Mat::zeros(image0->image.rows, image0->image.cols, CV_8UC1));

            cam.sensor_ids.push_back(1);
            cam.images.push_back(image1->image);
            cam.masks.push_back(cv::Mat::zeros(image1->image.rows, image1->image.cols, CV_8UC1));

            openvins_interface_->feed_measurement_camera(cam);

            if (vtag_work_queue_) {
              vtag_work_queue_->enqueue(cam);
            }

            cam_id_to_image_queue_[0].pop_front();
            cam_id_to_image_queue_[1].pop_front();

            double eplased_ms = tc.elapsed().millis();
            if (eplased_ms > 1) {
              LOGI("OpenvinsFlow.IMAGE: cost time %.3f ms, timestamp %.3f s", eplased_ms, cam.timestamp);
            }
          }
        } else {
          while (!cam_id_to_image_queue_[0].empty()) {
            hear_slam::TimeCounter tc;
            vio::ImageMeasurement::ConstPtr image0 = cam_id_to_image_queue_[0].front();
            ov_core::CameraData cam;
            cam.timestamp = time_translation_.convertMaplabToOpenvinsTimestamp(image0->timestamp);

            cam.sensor_ids.push_back(0);
            cam.images.push_back(image0->image);
            cam.masks.push_back(cv::Mat::zeros(image0->image.rows, image0->image.cols, CV_8UC1));

            openvins_interface_->feed_measurement_camera(cam);

            if (vtag_work_queue_) {
              vtag_work_queue_->enqueue(cam);
            }

            cam_id_to_image_queue_[0].pop_front();

            double eplased_ms = tc.elapsed().millis();
            if (eplased_ms > 1) {
              LOGI("OpenvinsFlow.IMAGE: cost time %.3f ms, timestamp %.3f s", eplased_ms, cam.timestamp);
            }
          }
        }
      });

//   // We need synced images (while imu is not required)
//   flow->registerSubscriber<message_flow_topics::SYNCED_NFRAMES_AND_IMU>(
//       kSubscriberNodeName, openvins_subscriber_options,
//       [this](const vio::SynchronizedNFrameImu::Ptr& nframe_imu) {
//         CHECK(nframe_imu);
//         aslam::VisualNFrame::Ptr nframe = nframe_imu->nframe;
//         CHECK(nframe);
//         LOG(INFO) << "OpenvinsFlow: Received nframe with ts "
//                   << nframe->getMinTimestampNanoseconds();

//         std::map<size_t, cv::Mat> ov_idx_to_img;
//         for (size_t i=0; i<nframe->getNumFrames(); i++) {
//           const size_t* openvins_cam_index =
//               maplab_to_openvins_cam_indices_mapping_.getRight(i);
//           if (openvins_cam_index == nullptr) {
//             // Skip this image, as the camera was marked as inactive.
//             continue;
//           }
//           const aslam::VisualFrame& frame = nframe->getFrame(i);
//           ov_idx_to_img[*openvins_cam_index] = frame.getRawImage();
//         }
//         CHECK_EQ(openvins_num_cameras_, ov_idx_to_img.size());

//         ov_core::CameraData cam;
//         cam.timestamp =
//             time_translation_.convertMaplabToOpenvinsTimestamp(
//                 nframe->getMinTimestampNanoseconds());
//         for (const auto& item : ov_idx_to_img) {
//           cam.sensor_ids.push_back(item.first);
//           cam.images.push_back(item.second);
//           int rows = item.second.rows;
//           int cols = item.second.cols;
//           cam.masks.push_back(cv::Mat::zeros(rows, cols, CV_8UC1));
//         }
//         openvins_interface_->feed_measurement_camera(cam);
//       });

  // Input localization updates.
#ifndef DISABLE_OPENVINS_RELOCAL
  flow->registerSubscriber<message_flow_topics::LOCALIZATION_RESULT>(
      kSubscriberNodeName, openvins_subscriber_options,
      std::bind(
          &OpenvinsLocalizationHandler::processLocalizationResult,
          localization_handler_.get(), std::placeholders::_1));
#endif

  // Output tag detections
  publish_tag_detections_ =
      flow->registerPublisher<message_flow_topics::TAG_DETECTIONS>();

  // Output global pose fusion
  publish_global_pose_fusion_ =
      flow->registerPublisher<message_flow_topics::GLOBAL_POSE_FUSION>();

  // Output OPENVINS estimates.
  publish_openvins_estimates_ =
      flow->registerPublisher<message_flow_topics::OPENVINS_ESTIMATES>();
  CHECK(openvins_interface_);
  openvins_interface_->setUpdateCallback(std::bind(
      &OpenvinsFlow::processAndPublishOpenvinsUpdate, this, std::placeholders::_1));

  auto publish_rgbd_local_map =
      flow->registerPublisher<message_flow_topics::RGBD_LOCAL_MAP>();
  openvins_interface_->set_rgbd_map_update_callback([=](std::shared_ptr<ov_msckf::dense_mapping::SimpleDenseMapOutput> rgbd_dense_map_output) {
    DenseMapWrapper map_wrapper;
    map_wrapper.map_data = rgbd_dense_map_output->ro_map;
    map_wrapper.timestamp_ns = time_translation_.convertOpenvinsToMaplabTimestamp(map_wrapper.map_data->time);
    auto map_wrapper_ptr = std::make_shared<const DenseMapWrapper>(map_wrapper);
    publish_rgbd_local_map(map_wrapper_ptr);
  });
}

void OpenvinsFlow::processTag(ov_core::CameraData cam) {
  auto openvins_cams = openvins_interface_->getLastOutput()->state_clone->_cam_intrinsics_cameras;
  if (openvins_cams.empty()) {
    LOG(ERROR) << "OpenvinsFlow::processTag(): No camera intrinsics found in Openvins output";
    return;
  }

  const int cam_id = 0;
  if (openvins_cams.count(cam_id) == 0) {
    LOG(ERROR) << "OpenvinsFlow::processTag(): No camera intrinsics found for camera id " << cam_id;
    return;
  }

  auto openvins_cam = openvins_cams.at(0);
  Eigen::MatrixXd openvins_cam_params = openvins_cam->get_value();

  std::unique_ptr<hear_slam::CameraModelInterface> camera;
  if (dynamic_cast<ov_core::CamRadtan*>(openvins_cam.get())) {
    camera = hear_slam::createCameraModel(
        hear_slam::CameraModelType::RADTAN4,
        openvins_cam_params);
  } else if (dynamic_cast<ov_core::CamEqui*>(openvins_cam.get())) {
    camera = hear_slam::createCameraModel(
        hear_slam::CameraModelType::EQUIDISTANT,
        openvins_cam_params);
  } else {
    const double* d = openvins_cam_params.data();
    Eigen::Matrix<double, 4, 1> intrin;
    intrin << d[0], d[1], d[2], d[3];
    camera = hear_slam::createCameraModel<hear_slam::CameraModelType::PINHOLE>(intrin);
  }

  ASSERT(cam.sensor_ids.at(0) == 0);
  cv::Mat gray;
  if (cam.images.at(0).channels() == 3) {
    cv::cvtColor(cam.images.at(0), gray, cv::COLOR_RGB2GRAY);
  } else {
    gray = cam.images.at(0);
  }

  using hear_slam::Time;
  Time start_time = Time::now();
  std::vector<hear_slam::TagDetection> detections = vtag_detector_->detect(gray, camera.get(), true, true, true);
  Time end_time = Time::now();
  LOGI("Tag-detection took %.2f ms", (end_time - start_time).millis());

  std::unique_ptr<hear_slam::Pose3dWithCov> cam_pose_and_cov;
  if (!tag_map_.empty() && !detections.empty()) {
    double reproj_rmse_thr = 1.5;  // 0.5;
    double reproj_maxerr_thr = 3.0;  // 2.0;
    int min_tags_to_loc = 2;  // 1
    bool compute_cov = false;  // true

    Time start_time = Time::now();
    cam_pose_and_cov = hear_slam::TagMapping::localizeCamera(
        *camera, detections, tag_map_, reproj_rmse_thr,
        reproj_maxerr_thr, min_tags_to_loc, compute_cov);
    Time end_time = Time::now();
    LOGI("Tag-loc took %.2f ms", (end_time - start_time).millis());

    if (cam_pose_and_cov) {
      Eigen::Isometry3d T_I_Color;
      {
        // - [1.0, 0.0, 0.0, 0.02878]
        // - [0.0, 1.0, 0.0, 0.0074]
        // - [0.0, 0.0, 1.0, 0.01602]
        Eigen::Matrix3d R_I_Color;
        R_I_Color <<  1, 0, 0,
                      0, 1, 0,
                      0, 0, 1;
        Eigen::Vector3d t_I_Color(0.02878, 0.0074, 0.01602);

        T_I_Color = Eigen::Isometry3d(R_I_Color);
        T_I_Color.translation() = t_I_Color;
      }

      hear_slam::Pose3dWithCov imu_pose_and_cov = (*cam_pose_and_cov) * (T_I_Color.inverse());
      hear_slam::GlobalPoseFusion::Pose3d pose(imu_pose_and_cov.R, imu_pose_and_cov.p);
      if (compute_cov) {
        Eigen::Matrix<double, 6, 6> cov;
        cov << imu_pose_and_cov.cov.block<3, 3>(3, 3), imu_pose_and_cov.cov.block<3, 3>(3, 0),
              imu_pose_and_cov.cov.block<3, 3>(0, 3), imu_pose_and_cov.cov.block<3, 3>(0, 0);
        global_pose_fusion_->feedGlobalPose(cam.timestamp, pose, &cov);
      } else {
        global_pose_fusion_->feedGlobalPose(cam.timestamp, pose);
      }
    }
  }

  auto stamped_detections = std::make_shared<StampedTagDetections>();
  stamped_detections->timestamp_ns = time_translation_.convertOpenvinsToMaplabTimestamp(cam.timestamp);
  stamped_detections->detections.swap(detections);
  stamped_detections->cam_id = cam.sensor_ids.at(0);
  publish_tag_detections_(stamped_detections);

  if (FLAGS_openvinsli_visualize_vtag) {
    cv::Mat display_image;
    if (!tag_map_.empty()) {
      Eigen::Isometry3d T_G_C;
      if (cam_pose_and_cov) {
        T_G_C = cam_pose_and_cov->toIsometry();
        display_image = hear_slam::TagMapping::visualizeTagReprojection(
            cam.timestamp * 1e9, gray, *camera,
            stamped_detections->detections,
            T_G_C, tag_map_);
      } else {
        display_image = hear_slam::TagDetectorInterface::visualizeTagDetections(
            cam.timestamp * 1e9, gray, stamped_detections->detections);
      }
    } else {
      display_image = hear_slam::TagDetectorInterface::visualizeTagDetections(
          cam.timestamp * 1e9, gray,
          stamped_detections->detections,
          camera.get());
    }

    cv::imshow("Tag detections", display_image);
    cv::waitKey(1);
  }
}

void OpenvinsFlow::processAndPublishOpenvinsUpdate(const ov_msckf::VioManager::Output& output) {

  if (FLAGS_openvinsli_save_feature_images) {
    // Get our image of history tracks
    cv::Mat feature_image = openvins_interface_->get_historical_viz_image(output);

    // RGB to BGR
    cv::cvtColor(feature_image, feature_image, cv::COLOR_RGB2BGR);

    if (!feature_image.empty()) {
      // the timestamp of the visulized image is actually 'prev_timestamp'.
      int64_t ts = int64_t(output.status.prev_timestamp * 1e9);
      std::string img_file = FLAGS_openvinsli_feature_images_dir + "/" + std::to_string(ts) + ".jpg";
      cv::imwrite(img_file, feature_image);
    }
  }

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

// std::cout << "processAndPublishOpenvinsUpdate: DEBUG 1" << ", output.state_clone->_timestamp=" << output.state_clone->_timestamp << ", output.state_clone->_imu->quat(): " << output.state_clone->_imu->quat().transpose() << std::endl;
  if (fabs(Eigen::Quaterniond(output.state_clone->_imu->Rot().inverse()).squaredNorm() - 1.0) > 0.001) {
    std::cout << "Bad quaternion: output.state_clone->_imu->Rot():" << std::endl << output.state_clone->_imu->Rot() << std::endl;
  }  
  aslam::Transformation T_M_I(
      output.state_clone->_imu->pos(),
      Eigen::Quaterniond(output.state_clone->_imu->Rot().inverse()));

  global_pose_fusion_->feedOdomometry(
      output.status.timestamp,
      hear_slam::GlobalPoseFusion::Pose3d(
          output.state_clone->_imu->Rot().inverse(),
          output.state_clone->_imu->pos()));

// std::cout << "processAndPublishOpenvinsUpdate: DEBUG 2" << std::endl;
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
// std::cout << "processAndPublishOpenvinsUpdate: DEBUG 3" << ", openvins_params.T_CtoIs.at(openvins_cam_idx)->block<3,3>(0,0): " << Eigen::Matrix3d(openvins_params.T_CtoIs.at(openvins_cam_idx)->block<3,3>(0,0)) << std::endl;
    if (fabs(Eigen::Quaterniond(openvins_params.T_CtoIs.at(openvins_cam_idx)->block<3,3>(0,0)).squaredNorm() - 1.0) > 0.001) {
      std::cout << "Bad quaternion: openvins_cam_idx=" << openvins_cam_idx
                << ", openvins_params.T_CtoIs.at(openvins_cam_idx)->block<3,3>(0,0):" << std::endl
                << openvins_params.T_CtoIs.at(openvins_cam_idx)->block<3,3>(0,0) << std::endl;
    }
    aslam::Transformation T_B_C(
        openvins_params.T_CtoIs.at(openvins_cam_idx)->block<3,1>(0,3),
        Eigen::Quaterniond(openvins_params.T_CtoIs.at(openvins_cam_idx)->block<3,3>(0,0)));
// std::cout << "processAndPublishOpenvinsUpdate: DEBUG 4" << std::endl;
    // aslam::Transformation T_B_C(
    //     openvins_params.T_CtoIs.at(openvins_cam_idx).block<3,3>(0,0),
    //     openvins_params.T_CtoIs.at(openvins_cam_idx).block<3,1>(0,3));
    common::ensurePositiveQuaternion(&T_B_C.getRotation());
    CHECK(openvins_estimate->maplab_camera_index_to_T_C_B
              .emplace(*maplab_cam_idx, T_B_C.inverse())
              .second);
  }

  // Optional localizations.
// std::cout << "processAndPublishOpenvinsUpdate: DEBUG 5" << std::endl;

#ifndef DISABLE_OPENVINS_RELOCAL
  openvins_estimate->has_T_G_M =
      extractLocalizationFromOpenvinsState(output, &openvins_estimate->T_G_M);
// std::cout << "processAndPublishOpenvinsUpdate: DEBUG 6" << std::endl;
  localization_handler_->T_M_I_buffer_mutable()->bufferOdometryEstimate(
      openvins_estimate->vinode);
  if (openvins_estimate->has_T_G_M) {
    localization_handler_->buffer_T_G_M(openvins_estimate->T_G_M);
  }
  localization_handler_->dealWithBufferedLocalizations();
#else
  openvins_estimate->has_T_G_M = false;
#endif

  std::swap(openvins_estimate, openvins_estimate_);
  publish_openvins_estimates_(openvins_estimate_);
}
}  // namespace openvinsli
