#include <memory>

#include <aslam/cameras/ncamera.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <localization-summary-map/localization-summary-map-creation.h>
#include <localization-summary-map/localization-summary-map.h>
#include <maplab-common/sigint-breaker.h>
#include <maplab-common/threading-helpers.h>
#include <maplab-ros-common/gflags-interface.h>
#include <message-flow/message-dispatcher-fifo.h>
#include <message-flow/message-flow.h>
#include <ros/ros.h>
#include <sensors/imu.h>
#include <sensors/sensor-types.h>
#include <signal.h>
#include <vi-map/sensor-utils.h>
#include <vi-map/vi-map-serialization.h>

#include "openvinsli/openvinsli-node.h"
#include <pthread.h>  // thread priority
#include <sched.h>  // thread priority

DEFINE_string(
    vio_localization_map_folder, "",
    "Path to a localization summary map or a full VI-map used for "
    "localization.");
DEFINE_string(sensor_calibration_file, "", "Path to sensor calibration yaml.");

DEFINE_string(
    external_imu_parameters_openvins, "",
    "Optional, path to the IMU configuration yaml for OPENVINS. If none is "
    "provided the maplab values will be used for OPENVINS as well.");
DEFINE_string(
    save_map_folder, "", "Save map to folder; if empty nothing is saved.");
DEFINE_bool(
    overwrite_existing_map, false,
    "If set to true, an existing map will be overwritten on save. Otherwise, a "
    "number will be appended to save_map_folder to obtain an available "
    "folder.");
DEFINE_bool(
    optimize_map_to_localization_map, false,
    "Optimize and process the map into a localization map before "
    "saving it.");
DECLARE_double(openvinsli_image_resize_factor);
DECLARE_bool(data_player_paused);


std::shared_ptr<openvinsli::OpenvinsliNode> openvins_localization_node = nullptr;
__sighandler_t old_sigint_handler = nullptr;
void shutdownSigintHandler(int sig) {
  std::cout << "[APP STATUS] Stop Requested ... " << std::endl;
  if (openvins_localization_node) {
    openvins_localization_node->shutdown();
  }
  // Keep ros alive to finish the map visualization.

  // ros::shutdown();

  // if (old_sigint_handler) {
  //   old_sigint_handler(sig);
  // }
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  const bool realtime_flow = true;

  ros::init(argc, argv, "openvinsli");
  ros::NodeHandle nh, nh_private("~");

  ros_common::parseGflagsFromRosParams(argv[0], nh_private);

  // Override the default sigint handler.
  // This must be set after the first NodeHandle is created.
  old_sigint_handler = signal(SIGINT, shutdownSigintHandler);

  // Load sensors.
  CHECK(!FLAGS_sensor_calibration_file.empty())
      << "[OPENVINSLI] No sensor calibration file was provided!";
  vi_map::SensorManager sensor_manager;
  if (!sensor_manager.deserializeFromFile(FLAGS_sensor_calibration_file)) {
    LOG(FATAL) << "[OPENVINSLI] Failed to read the sensor calibration from '"
               << FLAGS_sensor_calibration_file << "'!";
  }

  CHECK(vi_map::getSelectedImu(sensor_manager))
      << "[OPENVINSLI] The sensor calibration does not contain an IMU!";

  aslam::NCamera::Ptr mapping_ncamera =
      vi_map::getSelectedNCamera(sensor_manager);
  CHECK(mapping_ncamera)
      << "[OPENVINSLI] The sensor calibration does not contain a NCamera!";

  if (fabs(FLAGS_openvinsli_image_resize_factor - 1.0) > 1e-6) {
    for (size_t i = 0; i < mapping_ncamera->getNumCameras(); i++) {
      // The intrinsics of the camera can just be multiplied with the resize
      // factor. Distortion parameters are agnostic to the image size.
      aslam::Camera::Ptr camera = mapping_ncamera->getCameraShared(i);
      camera->setParameters(
          camera->getParameters() * FLAGS_openvinsli_image_resize_factor);
      camera->setImageWidth(
          round(camera->imageWidth() * FLAGS_openvinsli_image_resize_factor));
      camera->setImageHeight(
          round(camera->imageHeight() * FLAGS_openvinsli_image_resize_factor));
      camera->setDescription(
          camera->getDescription() + " - resized " +
          std::to_string(FLAGS_openvinsli_image_resize_factor));

      // The parameters have changed so we need to generate a new sensor id.
      aslam::SensorId camera_id;
      generateId(&camera_id);
      camera->setId(camera_id);
    }
  }

  // Optionally load localization map.
  std::unique_ptr<summary_map::LocalizationSummaryMap> localization_map;
  if (!FLAGS_vio_localization_map_folder.empty()) {
    localization_map.reset(new summary_map::LocalizationSummaryMap);
    if (!localization_map->loadFromFolder(FLAGS_vio_localization_map_folder)) {
      LOG(WARNING)
          << "[OPENVINSLI] Could not load a localization summary map from "
          << FLAGS_vio_localization_map_folder
          << ". Will try to load it as a full VI map.";
      vi_map::VIMap vi_map;
      CHECK(vi_map::serialization::loadMapFromFolder(
          FLAGS_vio_localization_map_folder, &vi_map))
          << "[OPENVINSLI] Loading a VI map failed. Either provide a valid "
             "localization map "
          << "or leave the map folder flag empty.";

      localization_map.reset(new summary_map::LocalizationSummaryMap);
      summary_map::createLocalizationSummaryMapForWellConstrainedLandmarks(
          vi_map, localization_map.get());
      // Make sure the localization map is not empty.
      CHECK_GT(localization_map->GLandmarkPosition().cols(), 0);
    }
  }

  // Optionally, load external values for the OPENVINS sigmas; otherwise also use
  // the maplab values for OPENVINS.
  vi_map::ImuSigmas openvins_imu_sigmas;
  if (!FLAGS_external_imu_parameters_openvins.empty()) {
    CHECK(openvins_imu_sigmas.loadFromYaml(FLAGS_external_imu_parameters_openvins))
        << "[OPENVINSLI] Could not load IMU parameters for OPENVINS from: \'"
        << FLAGS_external_imu_parameters_openvins << "\'";
    CHECK(openvins_imu_sigmas.isValid());
  } else {
    const vi_map::Imu::Ptr imu = vi_map::getSelectedImu(sensor_manager);
    CHECK(imu) << "[OPENVINSLI] No imu was found in the sensor calibration!";
    openvins_imu_sigmas = imu->getImuSigmas();
  }

  // Construct the application.
  ros::AsyncSpinner ros_spinner(common::getNumHardwareThreads());
  std::unique_ptr<message_flow::MessageFlow> flow(
      message_flow::MessageFlow::create<message_flow::MessageDispatcherFifo>(
          1, // common::getNumHardwareThreads(),
          "maplab_msg_flow",
          realtime_flow));

  if (FLAGS_map_builder_save_image_as_resources &&
      FLAGS_save_map_folder.empty()) {
    LOG(FATAL) << "If you would like to save the resources, "
               << "please also set a map folder with: --save_map_folder";
  }

  // If a map will be saved (i.e., if the save map folder is not empty), append
  // a number to the name until a name is found that is free.
  std::string save_map_folder = FLAGS_save_map_folder;
  if (!FLAGS_save_map_folder.empty()) {
    size_t counter = 0u;
    while (common::fileExists(save_map_folder) ||
           (!FLAGS_overwrite_existing_map &&
            common::pathExists(save_map_folder))) {
      save_map_folder = FLAGS_save_map_folder + "_" + std::to_string(counter++);
    }
  }

  openvins_localization_node = std::make_shared<openvinsli::OpenvinsliNode>(
      sensor_manager, openvins_imu_sigmas, save_map_folder, localization_map.get(),
      flow.get());

  // Start the pipeline. The ROS spinner will handle SIGINT for us and abort
  // the application on CTRL+C.
  std::cout << "[APP STATUS] Start ros_spinner ..." << std::endl;
  ros_spinner.start();
  std::cout << "[APP STATUS] Start openvinsli node ..." << std::endl;
  openvins_localization_node->start();

  std::atomic<bool>& end_of_days_signal_received =
      openvins_localization_node->isDataSourceExhausted();
  int main_loop = 0;
  while (ros::ok() && !end_of_days_signal_received.load()) {
    if (!FLAGS_data_player_paused) {
      VLOG_EVERY_N(1, 10) << "\n" << flow->printDeliveryQueueStatistics();
      if (main_loop++ % 10 == 0) {
        std::cout << "[APP STATUS] main_loop = " << main_loop << std::endl;
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  std::cout << "[APP STATUS] Going to shutdown openvinsli node ..." << std::endl;

  openvins_localization_node->shutdown();

  std::cout << "[APP STATUS] Going to shutdown message flow ..." << std::endl;

  flow->shutdown();

  std::cout << "[APP STATUS] Waiting message flow idle ..." << std::endl;

  flow->waitUntilIdle();

  std::cout << "[APP STATUS] Saving and optimizing (optionally) the vimap ..." << std::endl;

  if (!save_map_folder.empty()) {
    openvins_localization_node->saveMapAndOptionallyOptimize(
        save_map_folder, FLAGS_overwrite_existing_map,
        FLAGS_optimize_map_to_localization_map);
  }

  std::cout << "[APP STATUS] All done." << std::endl;

  return 0;
}
