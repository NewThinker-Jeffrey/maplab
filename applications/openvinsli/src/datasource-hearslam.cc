#include "openvinsli/datasource-hearslam.h"

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <aslam/common/time.h>
#include <boost/bind.hpp>
#include <maplab-common/accessors.h>
#include <maplab-common/file-system-tools.h>
#include <vio-common/rostopic-settings.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#pragma GCC diagnostic pop

#include "openvinsli/ros-helpers.h"
#include "hear_slam/basic/logging.h"

DEFINE_double(
    hearslam_data_start_s, 0.0, "Start of the hearslam_data in seconds.");
DEFINE_double(hearslam_data_end_s, 0.0, "End of the hearslam_data in seconds.");
DEFINE_double(
    hearslam_data_realtime_playback_rate, 1.0,
    "Playback rate of the hearslam_data. Real-time corresponds to 1.0. "
    "This only makes sense when using offline data sources.");
DEFINE_bool(
    hearslam_data_openvinsli_zero_initial_timestamps, false,
    "If set to true, the timestamps outputted by the estimator start with 0. "
    "Not zeroing the timestamps may lead to less accurate results due to "
    "rounding errors.");
DEFINE_int32(
    hearslam_data_image_downsample_rate, 3,
    "The downsample rate for image data. Our realsense works at 30 FPS, "
    "and usually we need 10 FPS.");
DEFINE_string(hearslam_data_rs_serial_num, "", "realsense serial num for hearslam datasource");
DEFINE_string(hearslam_data_rs_name_hint, "", "realsense name_hint for hearslam datasource");
DEFINE_string(hearslam_data_rs_port_hint, "", "realsense port hint for hearslam datasource");

namespace openvinsli {

DataSourceHearslam::DataSourceHearslam(
    const std::string& dataset_path)
    : dataset_path_(dataset_path),
      last_imu_timestamp_ns_(aslam::time::getInvalidTime()) {
  const uint8_t num_cameras = 2;

  auto image_cb = [this](int image_idx, hear_slam::CameraData msg) {
    // std::cout << "play image_idx: " << image_idx << std::endl;
    hearslamImageCallback(image_idx, msg);
  };

  auto imu_cb = [this](int imu_idx, hear_slam::ImuData msg) {
    // std::cout << "play imu: " << imu_idx << std::endl;
    hearslamImuCallback(imu_idx, msg);
  };

  if (dataset_path_.empty()) {
    LOG(WARNING) << "The dataset_path is not set, so we get live data from realsense.";

    // use rs_capture
    static constexpr int image_width = 848;
    static constexpr int image_height = 480;
    static constexpr int imu_rate = 200;

    hear_slam::RsCapture::BasicSettings bs;
    bs.infra_width = image_width;
    bs.infra_height = image_height;
    LOGW(YELLOW "Sensor settings for realsense: image size_wh(%d, %d), imu_rate(%d)\n" RESET, bs.infra_width, bs.infra_height, bs.gyro_framerate);

    hear_slam::RsCapture::DevicePreference dp;
    dp.serial_num = FLAGS_hearslam_data_rs_serial_num;
    dp.name_hint = FLAGS_hearslam_data_rs_name_hint;
    dp.port_hint = FLAGS_hearslam_data_rs_port_hint;

    if (num_cameras == 2) {
      source_ = std::make_shared<hear_slam::RsCapture>(
          hear_slam::ViDatasource::VisualSensorType::STEREO,
          true, image_cb, imu_cb, bs, dp);
    } else {
      assert(num_cameras == 1);
      source_ = std::make_shared<hear_slam::RsCapture>(
          hear_slam::ViDatasource::VisualSensorType::MONO,
          true, image_cb, imu_cb, bs, dp);
    }
    recorder_ = std::make_shared<hear_slam::ViRecorder>();
    recorder_->startRecord(source_.get());
    // recorder_->enableImageWindow();
  } else {
    if (num_cameras == 2) {
      source_ = std::make_shared<hear_slam::ViPlayer>(
              dataset_path_,
              FLAGS_hearslam_data_realtime_playback_rate,
              hear_slam::ViDatasource::VisualSensorType::STEREO,
              true, image_cb, imu_cb);
    } else {
      assert(num_cameras == 1);
      source_ = std::make_shared<hear_slam::ViPlayer>(
              dataset_path_,
              FLAGS_hearslam_data_realtime_playback_rate,
              hear_slam::ViDatasource::VisualSensorType::MONO,
              true, image_cb, imu_cb);
    }
  }

  source_->registerStreamingOverCallback([this](){
    invokeEndOfDataCallbacks();
  });

  if (num_cameras > 0u) {
    last_image_timestamp_ns_.resize(num_cameras, aslam::time::getInvalidTime());
  }

  if (FLAGS_imu_to_camera_time_offset_ns != 0) {
    LOG(WARNING) << "You are applying a time offset between IMU and camera, be "
                 << "aware that this will shift the image timestamps, which "
                 << "means the published pose estimates will now correspond "
                 << "these shifted timestamps!";
  }
}

DataSourceHearslam::~DataSourceHearslam() {
  shutdown();
}

std::string DataSourceHearslam::getDatasetName() const {
  std::string path, filename;
  if (!dataset_path_.empty()) {
    common::splitPathAndFilename(dataset_path_, &path, &filename);
    LOG(INFO) << "DataSourceHearslam::getDatasetName:  filename="
              << filename << ", path=" << path;
    return filename;
  } else {
    return "realsense-live";
  }
}

void DataSourceHearslam::startStreaming() {
  CHECK(source_);
  CHECK(source_->startStreaming());
}

void DataSourceHearslam::shutdown() {
  CHECK(source_);
  if (recorder_) {
    recorder_->stopRecord();
  }
  source_->stopStreaming();
}

bool DataSourceHearslam::allDataStreamed() const {
  return !source_->isStreaming();
}

void DataSourceHearslam::hearslamImageCallback(int image_idx, hear_slam::CameraData msg) {
  if (image_idx % FLAGS_hearslam_data_image_downsample_rate != 0) {
    return;
  }

  size_t n_cam = msg.sensor_ids.size();

  for (size_t i=0; i<n_cam; i++) {
    // const size_t camera_idx = i;
    const size_t camera_idx = msg.sensor_ids[i];
    CHECK_EQ(msg.sensor_ids[i], i);

    vio::ImageMeasurement::Ptr image_measurement = 
        std::make_shared<vio::ImageMeasurement>();
    image_measurement->timestamp = msg.timestamp*1e9;
    image_measurement->camera_index = camera_idx;
    image_measurement->image = msg.images[i];

    // Apply the IMU to camera time shift.
    if (FLAGS_imu_to_camera_time_offset_ns != 0) {
      image_measurement->timestamp += FLAGS_imu_to_camera_time_offset_ns;
    }

    // Shift timestamps to start at 0.
    if (!FLAGS_hearslam_data_openvinsli_zero_initial_timestamps ||
        shiftByFirstTimestamp(&(image_measurement->timestamp))) {
      // Check for strictly increasing image timestamps.
      CHECK_LT(camera_idx, last_image_timestamp_ns_.size());
      if (aslam::time::isValidTime(last_image_timestamp_ns_[camera_idx]) &&
          last_image_timestamp_ns_[camera_idx] >=
              image_measurement->timestamp) {
        LOG(WARNING) << "[OPENVINSLI-DataSource] Image message (cam "
                      << camera_idx << ") is not strictly "
                      << "increasing! Current timestamp: "
                      << image_measurement->timestamp
                      << "ns vs last timestamp: "
                      << last_image_timestamp_ns_[camera_idx] << "ns.";
      } else {
        last_image_timestamp_ns_[camera_idx] = image_measurement->timestamp;

        VLOG(3) << "Publish Image measurement...";
        invokeImageCallbacks(image_measurement);
      }
    }
  }
}

void DataSourceHearslam::hearslamImuCallback(int imu_idx, hear_slam::ImuData msg) {
  vio::ImuData data;
  data << msg.am.x(), msg.am.y(), msg.am.z(),
          msg.wm.x(), msg.wm.y(), msg.wm.z();
  vio::ImuMeasurement::Ptr imu_measurement =
      std::make_shared<vio::ImuMeasurement>(msg.timestamp*1e9, data);

  // Shift timestamps to start at 0.
  if (!FLAGS_hearslam_data_openvinsli_zero_initial_timestamps ||
      shiftByFirstTimestamp(&(imu_measurement->timestamp))) {
    // Check for strictly increasing imu timestamps.
    if (aslam::time::isValidTime(last_imu_timestamp_ns_) &&
        last_imu_timestamp_ns_ >= imu_measurement->timestamp) {
      LOG(WARNING) << "[OPENVINSLI-DataSource] IMU message is not strictly "
                    << "increasing! Current timestamp: "
                    << imu_measurement->timestamp
                    << "ns vs last timestamp: " << last_imu_timestamp_ns_
                    << "ns.";
    } else {
      last_imu_timestamp_ns_ = imu_measurement->timestamp;

      VLOG(3) << "Publish IMU measurement...";
      invokeImuCallbacks(imu_measurement);
    }
  }
}

}  // namespace openvinsli
