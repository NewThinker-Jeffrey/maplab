#ifndef OPENVINSLI_DATASOURCE_HEARSLAM_H_
#define OPENVINSLI_DATASOURCE_HEARSLAM_H_

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <aslam/common/time.h>
#include <vio-common/rostopic-settings.h>
#include <vio-common/vio-types.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#pragma GCC diagnostic pop

#include "openvinsli/datasource.h"
#include "hear_slam/utils/datasource/vi_source.h"
#include "hear_slam/utils/datasource/vi_recorder.h"
#include "hear_slam/utils/datasource/vi_player.h"
#include "hear_slam/utils/datasource/rs/rs_capture.h"

DECLARE_int64(imu_to_camera_time_offset_ns);

namespace openvinsli {

class DataSourceHearslam : public DataSource {
 public:
  DataSourceHearslam(
      const std::string& dataset_path, bool rgbd = false);
  virtual ~DataSourceHearslam();

  virtual void startStreaming();
  virtual void shutdown();
  virtual bool allDataStreamed() const;
  virtual std::string getDatasetName() const;

  hear_slam::ViPlayer* getPlayer() const;

 private:
  void hearslamImageCallback(int image_idx, hear_slam::CameraData msg);
  void hearslamImuCallback(int imu_idx, hear_slam::ImuData msg);

 private:
  std::string dataset_path_;
  std::shared_ptr<hear_slam::ViDatasource> source_;
  std::shared_ptr<hear_slam::ViRecorder> recorder_;

  int64_t last_imu_timestamp_ns_;
  int64_t last_depth_timestamp_ns_;
  std::vector<int64_t> last_image_timestamp_ns_;

  bool rgbd_;
};

}  // namespace openvinsli

#endif  // OPENVINSLI_DATASOURCE_HEARSLAM_H_
