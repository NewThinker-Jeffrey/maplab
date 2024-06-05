#include "openvinsli/datasource-hearslam.h"

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>

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
#include "hear_slam/basic/thread_pool.h"

DEFINE_double(
    hearslam_data_start_s, 0.0, "Start of the hearslam_data in seconds.");
DEFINE_double(hearslam_data_end_s, 0.0, "End of the hearslam_data in seconds.");
DEFINE_double(
    hearslam_data_realtime_playback_rate, 1.0,
    "Playback rate of the hearslam_data. Real-time corresponds to 1.0. "
    "This only makes sense when using offline data sources.");
DEFINE_bool(
    hearslam_data_auto_record_live, true,
    "If set to true, the live data will be auto recorded.");
DEFINE_bool(
    hearslam_data_openvinsli_zero_initial_timestamps, false,
    "If set to true, the timestamps outputted by the estimator start with 0. "
    "Not zeroing the timestamps may lead to less accurate results due to "
    "rounding errors.");
DEFINE_int32(
    hearslam_data_image_downsample_rate, 3,
    "The downsample rate for image data. Our realsense works at 30 FPS, "
    "and usually we need 10 FPS.");
DEFINE_int32(
    hearslam_data_rgbd_align_interval, 15,
    "How often (in frames) to align depth and color images (only used in rgbd mode)");
DEFINE_string(hearslam_data_rs_serial_num, "", "realsense serial num for hearslam datasource");
DEFINE_string(hearslam_data_rs_name_hint, "", "realsense name_hint for hearslam datasource");
DEFINE_string(hearslam_data_rs_port_hint, "", "realsense port hint for hearslam datasource");
DEFINE_double(hearslam_data_infra_exposure_time_us, -1.0, "<0 for auto exposure");
DEFINE_double(hearslam_data_color_exposure_time_us, -1.0, "<0 for auto exposure");
DEFINE_bool(
    hearslam_data_visualize_depth, false,
    "Whether to visualize depth images.");
DEFINE_bool(
    hearslam_data_save_visualized_depth, false,
    "Whether to save visualized_depth images.");
DEFINE_string(
    hearslam_data_visualized_depth_images_dir, "tmp_visualized_depth_images",
    "dir to save visualized_depth images.");

namespace {

uint8_t* log_depth_table = nullptr;

cv::Mat visualizeDepthImage(const cv::Mat& depth_img) {
  ASSERT(depth_img.type() == CV_16U);

  if (!log_depth_table) {
    // build log_depth_table
    log_depth_table = new uint8_t[65536];
    const double inv_log2__x__16 = 16.0 / std::log(2.0);
    auto log_value = [](int depth_i) {
      double depth = 0.001 * depth_i;  // in meters
      static const double base_depth = 0.3;
      double delta_depth = (depth - base_depth);
      delta_depth = std::max(delta_depth, 0.0);
      // return log(static_cast<double>(1000.0 * delta_depth + 1.0));
      return log(double(delta_depth/5.0 + 1.0));
      // return log(double(depth+1.0));
    };
    double min_value = log_value(0);
    // double max_value = log_value(65535);
    double max_value = log_value(5000);
    double ratio = 255.0 / (max_value - min_value);
    for (int i = 0; i < 65536; i++) {
      int v = (log_value(i) - min_value) * ratio;
      v = std::min(v, 255);
      v = std::max(v, 0);
      log_depth_table[i] = 255 - v;
    }

    // 0 for unavailable.
    log_depth_table[0] = 0;
  }

  cv::Mat log_depth_img(depth_img.size(), CV_8UC3);
  auto* ptr_log = log_depth_img.ptr<uint8_t>();
  const auto* ptr = depth_img.ptr<uint16_t>();
  for (size_t i = 0; i < depth_img.rows * depth_img.cols; i++) {
    ptr_log[i*3] = log_depth_table[ptr[i]];
    ptr_log[i*3 + 1] = 0;
    ptr_log[i*3 + 2] = (ptr_log[i*3] != 0) * (255 - ptr_log[i*3]);
  }

  return log_depth_img;
}

}

namespace openvinsli {

DataSourceHearslam::DataSourceHearslam(
    const std::string& dataset_path, bool rgbd)
    : dataset_path_(dataset_path),
      rgbd_(rgbd),
      last_depth_timestamp_ns_(aslam::time::getInvalidTime()),
      last_imu_timestamp_ns_(aslam::time::getInvalidTime()) {
  uint8_t num_cameras = 2;
  if (rgbd_) {
    num_cameras = 1;
  }

  auto image_cb = [this](int image_idx, hear_slam::CameraData msg) {
    // std::cout << "play image_idx: " << image_idx << std::endl;
    hearslamImageCallback(image_idx, msg);
  };

  auto imu_cb = [this](int imu_idx, hear_slam::ImuData msg) {
    // std::cout << "play imu: " << imu_idx << std::endl;
    hearslamImuCallback(imu_idx, msg);
  };

  if (FLAGS_hearslam_data_save_visualized_depth) {
    std::filesystem::path dir_path(FLAGS_hearslam_data_visualized_depth_images_dir);
    if (std::filesystem::create_directory(dir_path)) {
        std::cout << "visualized_depth_images_dir created successfully." << std::endl;
    } else {
        std::cout << "visualized_depth_images_dir '" <<  FLAGS_hearslam_data_visualized_depth_images_dir << "' already exists or cannot be created!!" << std::endl;
    }
  }

  if (dataset_path_.empty()) {
    LOG(WARNING) << "The dataset_path is not set, so we get live data from realsense.";

    // use rs_capture
    static constexpr int image_width = 848;
    static constexpr int image_height = 480;
    static constexpr int imu_rate = 200;

    hear_slam::RsCapture::BasicSettings bs;
    bs.image_buffer_size = -1;  // buffer all
    bs.infra_framerate = 30;
    bs.infra_width = image_width;
    bs.infra_height = image_height;
    bs.infra_exposure_time_us = FLAGS_hearslam_data_infra_exposure_time_us;
    bs.color_exposure_time_us = FLAGS_hearslam_data_color_exposure_time_us;

    LOGW(YELLOW "Sensor settings for realsense: image size_wh(%d, %d), imu_rate(%d)\n" RESET, bs.infra_width, bs.infra_height, bs.gyro_framerate);

    hear_slam::RsCapture::DevicePreference dp;
    dp.serial_num = FLAGS_hearslam_data_rs_serial_num;
    dp.name_hint = FLAGS_hearslam_data_rs_name_hint;
    dp.port_hint = FLAGS_hearslam_data_rs_port_hint;

    if (rgbd_) {
      CHECK(num_cameras == 1);
      CHECK(FLAGS_hearslam_data_rgbd_align_interval % FLAGS_hearslam_data_image_downsample_rate == 0);
      bs.depth_framerate = 30;
      bs.color_framerate = 30;
      bs.depth_width = image_width;
      bs.depth_height = image_height;
      bs.color_width = image_width;
      bs.color_height = image_height;
      bs.rgbd_align_interval = FLAGS_hearslam_data_rgbd_align_interval;
      source_ = std::make_shared<hear_slam::RsCapture>(
          hear_slam::ViDatasource::VisualSensorType::RGBD,
          true, image_cb, imu_cb, bs, dp);
    } else if (num_cameras == 2) {
      source_ = std::make_shared<hear_slam::RsCapture>(
          hear_slam::ViDatasource::VisualSensorType::STEREO,
          true, image_cb, imu_cb, bs, dp);
    } else {
      assert(num_cameras == 1);
      source_ = std::make_shared<hear_slam::RsCapture>(
          hear_slam::ViDatasource::VisualSensorType::MONO,
          true, image_cb, imu_cb, bs, dp);
    }
    if (FLAGS_hearslam_data_auto_record_live) {
      recorder_ = std::make_shared<hear_slam::ViRecorder>();
      recorder_->startRecord(source_.get());
      // recorder_->enableImageWindow();
    }
  } else {
    if (rgbd_) {
      assert(num_cameras == 1);
      source_ = std::make_shared<hear_slam::ViPlayer>(
              dataset_path_,
              FLAGS_hearslam_data_realtime_playback_rate,
              hear_slam::ViDatasource::VisualSensorType::RGBD,
              true, image_cb, imu_cb);
    } else if (num_cameras == 2) {
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

hear_slam::ViPlayer* DataSourceHearslam::getPlayer() const {
  if (!source_) {
    return nullptr;
  } else {
    // Return non-null only when we're using ViPlayer as the source_.
    return dynamic_cast<hear_slam::ViPlayer*>(source_.get());
  }
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
    CHECK_EQ(msg.sensor_ids[i], i);
    // int camera_idx = i;
    int camera_idx = msg.sensor_ids[i];
    if (rgbd_ && i == 1) {
      camera_idx = -1;
    }

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

      if (camera_idx < 0) {
        assert(rgbd_ && i == 1);
        if (aslam::time::isValidTime(last_depth_timestamp_ns_) &&
            last_depth_timestamp_ns_ >= image_measurement->timestamp) {
          LOG(WARNING) << "[OPENVINSLI-DataSource] Depth Image message is not strictly "
                        << "increasing! Current timestamp: "
                        << image_measurement->timestamp
                        << "ns vs last timestamp: " << last_depth_timestamp_ns_
                        << "ns.";
        } else {
          last_depth_timestamp_ns_ = image_measurement->timestamp;
          VLOG(3) << "Publish Depth Image measurement...";
          invokeImageCallbacks(image_measurement);

          if (FLAGS_hearslam_data_save_visualized_depth || FLAGS_hearslam_data_visualize_depth) {
            // Get our image of history tracks
            hear_slam::ThreadPool::getNamed("depth_viz")->schedule([image_measurement](){
              cv::Mat visualized_depth_image = visualizeDepthImage(image_measurement->image);
              // // RGB to BGR
              // cv::cvtColor(visualized_depth_image, visualized_depth_image, cv::COLOR_RGB2BGR);

              if (!visualized_depth_image.empty()) {
                int64_t ts = image_measurement->timestamp;
                std::string img_file = FLAGS_hearslam_data_visualized_depth_images_dir + "/" + std::to_string(ts) + ".jpg";
                if (FLAGS_hearslam_data_save_visualized_depth) {
                  cv::imwrite(img_file, visualized_depth_image);
                }
                if (FLAGS_hearslam_data_visualize_depth) {
                  cv::imshow("depth", visualized_depth_image);
                  cv::waitKey(1);
                }
              }
            });
          }          
        }
      } else {
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
