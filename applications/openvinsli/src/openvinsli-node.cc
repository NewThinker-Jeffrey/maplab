#include "openvinsli/openvinsli-node.h"

#include <string>

#include <aslam/cameras/ncamera.h>
#include <localization-summary-map/localization-summary-map.h>
#include <message-flow/message-flow.h>
#include <message-flow/message-topic-registration.h>
#include <vi-map/sensor-utils.h>
#include <vio-common/vio-types.h>

#include "openvinsli/datasource-flow.h"
#include "openvinsli/feature-tracking-flow.h"
#include "openvinsli/imu-camera-synchronizer-flow.h"
#include "openvinsli/localizer-flow.h"
#include "openvinsli/openvins-flow.h"
#include "openvinsli/mini-nav2d-flow.h"
#include "openvinsli/datasource-hearslam.h"

DEFINE_bool(
    openvinsli_run_map_builder, true,
    "When set to false, the map builder will be deactivated and no map will be "
    "built. Openvins+Localization will still run as usual.");

DEFINE_bool(
    openvinsli_run_nav, true,
    "Run nav");

DEFINE_string(openvinsli_nav_config, "", "nav_config: traj and target points for navigation");

DEFINE_string(openvinsli_nav_cmd_to_play, "", "the file storing the nav cmds to play (for offline play mode)");

DEFINE_string(openvinsli_nav_cmd_savefile, "", "the file to store the online nav cmds");

DEFINE_double(openvinsli_gl_viz_rate, 40.0, "gl visualization rate");

DECLARE_double(vio_max_localization_frequency_hz);

namespace openvinsli {
OpenvinsliNode::OpenvinsliNode(
    const vi_map::SensorManager& sensor_manager,
    const vi_map::ImuSigmas& openvins_imu_sigmas,
    const std::string& save_map_folder,
    const summary_map::LocalizationSummaryMap* const localization_map,
    message_flow::MessageFlow* flow)
    : is_datasource_exhausted_(false) {
  // localization_summary_map is optional and can be a nullptr.
  CHECK_NOTNULL(flow);

  aslam::NCamera::Ptr camera_system = getSelectedNCamera(sensor_manager);
  CHECK(camera_system) << "No NCamera found in the sensor manager.";
  vi_map::Imu::Ptr maplab_imu_sensor = getSelectedImu(sensor_manager);
  CHECK(maplab_imu_sensor) << "No IMU found in the sensor manager.";
  vi_map::WheelOdometry::Ptr maplab_wheel_odometry_sensor =
      getSelectedWheelOdometrySensor(sensor_manager);
  LOG_IF(INFO, maplab_wheel_odometry_sensor == nullptr)
      << "No Wheel odometry Odometry sensor found in the sensor manager.";

  if (maplab_wheel_odometry_sensor != nullptr) {
    const aslam::Transformation& T_B_S =
        sensor_manager.getSensor_T_B_S(maplab_wheel_odometry_sensor->getId());
    openvins_flow_.reset(new OpenvinsFlow(*camera_system, openvins_imu_sigmas, T_B_S));
    openvins_flow_->attachToMessageFlow(flow);
  } else {
    // TODO(ben): remove identity transformation if no rel 6dof sensor available
    openvins_flow_.reset(new OpenvinsFlow(
        *camera_system, openvins_imu_sigmas, aslam::Transformation()));
    openvins_flow_->attachToMessageFlow(flow);
  }

  auto openvins_params = openvins_flow_->openvinsInterface()->get_params();
  vio_common::RosTopicSettings topic_settings(sensor_manager);
  datasource_flow_.reset(new DataSourceFlow(topic_settings, openvins_params.use_rgbd));
  datasource_flow_->attachToMessageFlow(flow);

  const bool localization_enabled = localization_map != nullptr;
  if (FLAGS_openvinsli_run_map_builder || localization_enabled) {
    // If there's no localization and no map should be built, no maplab feature
    // tracking is needed.
    if (localization_enabled) {
      constexpr bool kVisualizeLocalization = true;
      localizer_flow_.reset(
          new LocalizerFlow(*localization_map, kVisualizeLocalization));
      localizer_flow_->attachToMessageFlow(flow);
    }

    // Launch the synchronizer after the localizer because creating the
    // localization database can take some time. This can cause the
    // synchronizer's detection of missing image or IMU measurements to fire
    // early.
    synchronizer_flow_.reset(new ImuCameraSynchronizerFlow(camera_system));
    synchronizer_flow_->attachToMessageFlow(flow);

    double max_feattrack_frequency_hz = -1.0;
    if (!FLAGS_openvinsli_run_map_builder) {
      // only localization is enabled.
      // we don't need high feattrack frequency for localization.
      max_feattrack_frequency_hz = FLAGS_vio_max_localization_frequency_hz;
    }
    tracker_flow_.reset(
        new FeatureTrackingFlow(camera_system, *maplab_imu_sensor, max_feattrack_frequency_hz));
    tracker_flow_->attachToMessageFlow(flow);
  }

  gl_viewer_.reset(new OpenvinsliViewer(openvins_flow_->openvinsInterface()));
  {
    DataSourceHearslam* hearslam_ds = dynamic_cast<DataSourceHearslam*>(datasource_flow_->getDataSource());
    if (hearslam_ds) {
      hear_slam::ViPlayer* vi_player = hearslam_ds->getPlayer();
      if (vi_player) {
        gl_viewer_->setViPlayer(vi_player);
      }
    }
  }
  stop_viz_request_ = false;
  vis_thread_.reset(new std::thread([this] {
    pthread_setname_np(pthread_self(), "ov_visualize");
    if (gl_viewer_) {
      gl_viewer_->init();
    }

    // use a high rate to ensure the vis_output to update in time (which is also needed in visualize_odometry()).
    ros::Rate  loop_rate(FLAGS_openvinsli_gl_viz_rate);
    while (ros::ok() && !stop_viz_request_) {
      auto simple_output = openvins_flow_->openvinsInterface()->getLastOutput(false, false);
      // if (simple_output->status.timestamp <= 0 || last_visualization_timestamp_ == simple_output->status.timestamp && simple_output->status.initialized) {
      //   continue;
      // }
      if (simple_output->status.timestamp <= 0) {
        continue;
      }

      auto vis_output = openvins_flow_->openvinsInterface()->getLastOutput(true, true);
      // last_visualization_timestamp_ = vis_output->state_clone->_timestamp;
      last_visualization_timestamp_ = vis_output->status.timestamp;
      if (gl_viewer_) {
        gl_viewer_->show(vis_output);
      }
      openvins_flow_->openvinsInterface()->clear_older_tracking_cache(vis_output->status.prev_timestamp);  // clear tracking cache at the prev timestamp after images are published

      loop_rate.sleep();
    }
  }));


  if (FLAGS_openvinsli_run_nav) {
    nav2d_flow_.reset(new Nav2dFlow());
    if (!FLAGS_openvinsli_nav_cmd_to_play.empty()) {
      LOG(WARNING) << "nav2d_flow_: We're running in offline play mode!";
      nav2d_flow_->beginPlayNavCmds(FLAGS_openvinsli_nav_cmd_to_play);
    } else if (!FLAGS_openvinsli_nav_cmd_savefile.empty()) {
      nav2d_flow_->beginSaveNavCmds(FLAGS_openvinsli_nav_cmd_savefile);
    }

    nav2d_flow_->attachToMessageFlow(flow);

    if (!FLAGS_openvinsli_nav_config.empty()) {
      if (common::fileExists(FLAGS_openvinsli_nav_config)) {
        nav2d_flow_->deserialize(FLAGS_openvinsli_nav_config);
      } else {
        LOG(WARNING) << "BadNavConfig: the nav config file specified by --openvinsli_nav_config ("
                     << FLAGS_openvinsli_nav_config << ") doesn't exist!";
      }
    }

    gl_viewer_->setNav(nav2d_flow_.get());
    message_flow::DeliveryOptions subscriber_options;
    subscriber_options.exclusivity_group_id =
        kExclusivityGroupIdOpenvinsSensorSubscribers;
    flow->registerSubscriber<message_flow_topics::NAV2D_CMD>(
        "Nav2dVisualization", subscriber_options,
        [this](const openvinsli::Nav2dCmd::ConstPtr& nav_cmd) {
          if (gl_viewer_) {
            gl_viewer_->setNavCmd(nav_cmd);
          }
        });
  }

  data_publisher_flow_.reset(new DataPublisherFlow);
  data_publisher_flow_->attachToMessageFlow(flow);

  if (FLAGS_openvinsli_run_map_builder) {
    map_builder_flow_.reset(
        new MapBuilderFlow(sensor_manager, save_map_folder));
    map_builder_flow_->attachToMessageFlow(flow);
  }

  // Subscribe to end of days signal from the datasource.
  datasource_flow_->registerEndOfDataCallback(
      [&]() { is_datasource_exhausted_.store(true); });
}

OpenvinsliNode::~OpenvinsliNode() {
  shutdown();
}

void OpenvinsliNode::saveMapAndOptionallyOptimize(
    const std::string& path, const bool overwrite_existing_map,
    bool process_to_localization_map) {
  if (map_builder_flow_) {
    map_builder_flow_->saveMapAndOptionallyOptimize(
        path, overwrite_existing_map, process_to_localization_map);
  }
}

void OpenvinsliNode::start() {
  CHECK(!is_datasource_exhausted_.load())
      << "Cannot start localization node after the "
      << "end-of-days signal was received!";
  datasource_flow_->startStreaming();
  VLOG(1) << "Starting data source...";
}

void OpenvinsliNode::shutdown() {
  datasource_flow_->shutdown();
  VLOG(1) << "Closing data source...";

  if (vis_thread_) {
    stop_viz_request_ = true;
    vis_thread_->join();
    vis_thread_.reset();
  }
}

std::atomic<bool>& OpenvinsliNode::isDataSourceExhausted() {
  return is_datasource_exhausted_;
}

}  // namespace openvinsli
