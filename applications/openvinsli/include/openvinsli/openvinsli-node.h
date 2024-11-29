#ifndef OPENVINSLI_OPENVINSLI_NODE_H_
#define OPENVINSLI_OPENVINSLI_NODE_H_
#include <memory>

#include <atomic>
#include <string>

#include <message-flow/message-flow.h>
#include <sensors/imu.h>
#include <sensors/wheel-odometry-sensor.h>

#include "openvinsli/data-publisher-flow.h"
#include "openvinsli/datasource-flow.h"
#include "openvinsli/feature-tracking-flow.h"
#include "openvinsli/imu-camera-synchronizer-flow.h"
#include "openvinsli/localizer-flow.h"
#include "openvinsli/map-builder-flow.h"
#include "openvinsli/openvins-flow.h"
#include "openvinsli/viewer.h"       


#include "mininav2d/mininav2d-flow.h"

namespace openvinsli {
class OpenvinsliNode final {
 public:
  OpenvinsliNode(
      const vi_map::SensorManager& sensor_manager,
      const vi_map::ImuSigmas& openvins_imu_sigmas,
      const std::string& save_map_folder,
      const summary_map::LocalizationSummaryMap* const localization_map,
      message_flow::MessageFlow* flow);
  ~OpenvinsliNode();

  void start();
  void shutdown();

  // Save the map to disk. Optionally keyframe, optimize and summarize the map.
  void saveMapAndOptionallyOptimize(
      const std::string& path, const bool overwrite_existing_map,
      const bool process_to_localization_map);

  std::atomic<bool>& isDataSourceExhausted();

 private:
  std::unique_ptr<DataSourceFlow> datasource_flow_;
  std::unique_ptr<OpenvinsFlow> openvins_flow_;
  std::unique_ptr<LocalizerFlow> localizer_flow_;
  std::unique_ptr<ImuCameraSynchronizerFlow> synchronizer_flow_;
  std::unique_ptr<FeatureTrackingFlow> tracker_flow_;
  std::unique_ptr<MapBuilderFlow> map_builder_flow_;
  std::unique_ptr<mininav2d::Nav2dFlow> nav2d_flow_;
  std::unique_ptr<DataPublisherFlow> data_publisher_flow_;

  // gl visualization
  std::unique_ptr<OpenvinsliViewer> gl_viewer_;
  std::unique_ptr<std::thread> vis_thread_;
  std::atomic<bool> stop_viz_request_;
  double last_visualization_timestamp_ = 0;

  // Set to true once the data-source has played back all its data. Will never
  // be true for infinite data-sources (live-data).
  std::atomic<bool> is_datasource_exhausted_;
};
}  // namespace openvinsli
#endif  // OPENVINSLI_OPENVINSLI_NODE_H_
