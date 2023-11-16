#include "openvinsli/openvins-factory.h"

#include <Eigen/Core>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/distortion.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/file-system-tools.h>
#include <message-flow/message-flow.h>
#include <vio-common/vio-types.h>

#include "core/VioManager.h"         // ov_msckf
#include "core/VioManagerOptions.h"  // ov_msckf
#include "state/Propagator.h"        // ov_msckf
#include "state/State.h"             // ov_msckf
#include "state/StateHelper.h"       // ov_msckf
#include "utils/print.h"             // ov_core
#include "utils/sensor_data.h"       // ov_core


// DEFINE_bool(
//     openvins_enable_frame_visualization, true,
//     "Set to false to disable the Openvins GUI.");
// DEFINE_double(
//     openvinsli_position_noise_density, 0.01,
//     "Position prediction noise density [m/sqrt(s)].");

// DEFINE_string(
//     openvins_image_mask_path, "",
//     "Path to image mask to be applied to the OPENVINS. No features are extracted "
//     "on the masked areas. Currently only supports a single camera.");

DEFINE_string(
    openvins_config_path, "",
    "Path to estimator_config.yaml for ov_msckf. "
    "By default we use ${OPENVINS_CONFIG_DIR}/openvins_default_estimator_config.yaml");

namespace openvinsli {
namespace {

ov_msckf::VioManagerOptions loadOpenvinsConfig() {  // Load the config
  std::string config_path = FLAGS_openvins_config_path;
  if (config_path.empty()) {
    PRINT_WARNING(YELLOW "You haven't specify --openvins_config_path, and we'll use "
                  "'${OPENVINS_CONFIG_DIR}/openvins_default_estimator_config.yaml' "
                  "by default.\n" RESET);
    const char* openvins_config_template_path = getenv("OPENVINS_CONFIG_DIR");
    CHECK_NE(openvins_config_template_path, static_cast<char*>(NULL))
        << "OPENVINS_CONFIG_DIR environment variable is not set.\n"
        << "Source the Maplab environment from your workspace:\n"
        << "source devel/setup.bash";
    config_path = std::string(openvins_config_template_path) + "/openvins_default_estimator_config.yaml";
  }
  PRINT_INFO("Openvins:config_path='%s'\n", config_path.c_str());
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);

  // Verbosity
  std::string verbosity = "DEBUG";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);  

  ov_msckf::VioManagerOptions params;
  params.print_and_load(parser);
  params.use_multi_threading_subs = true;
  // Ensure we read in all parameters required
  if (!parser->successful()) {
    PRINT_ERROR(RED "unable to parse all parameters, please fix\n" RESET);
    std::exit(EXIT_FAILURE);
  }
  return params;
}

void setOpenvinsImuSigmas(
    const vi_map::ImuSigmas& imu_sigmas,
    ov_msckf::VioManagerOptions* openvins_config) {
  CHECK_NOTNULL(openvins_config);
  CHECK(imu_sigmas.isValid());
  openvins_config->set_imu_noises(
      imu_sigmas.gyro_noise_density,
      imu_sigmas.gyro_bias_random_walk_noise_density,
      imu_sigmas.acc_noise_density,
      imu_sigmas.acc_bias_random_walk_noise_density);
}

void convertAslamToOpenvinsCamera(
    const aslam::NCamera& aslam_cameras,
    ov_msckf::VioManagerOptions* openvins_config) {
  CHECK_NOTNULL(openvins_config);

  const size_t num_cameras = aslam_cameras.numCameras();
  CHECK_EQ(openvins_config->state_options.num_cameras, num_cameras);
  for (size_t cam_idx = 0u; cam_idx < num_cameras; ++cam_idx) {
    // Translate cameras intrinsic parameters.
    const aslam::Camera& acam = aslam_cameras.getCamera(cam_idx);
    CHECK_EQ(acam.getType(), aslam::Camera::Type::kPinhole)
        << "Only pinhole models supported.";

    std::string dist_model = "radtan";
    Eigen::Matrix<double, 4, 1> dist;
    // Translate cameras distortion parameters.
    switch (acam.getDistortion().getType()) {
      case aslam::Distortion::Type::kNoDistortion:
        // The no-distortion case is emulated using a RADTAN distortion with
        // parameters that do not distort anything.
        dist_model = "radtan";
        dist = Eigen::Matrix<double, 4, 1>::Zero();
        break;
      case aslam::Distortion::Type::kRadTan:
        dist_model = "radtan";
        dist = acam.getDistortion().getParameters();
        break;
      case aslam::Distortion::Type::kEquidistant:
        dist_model = "equidistant";
        dist = acam.getDistortion().getParameters();
        break;
      // case aslam::Distortion::Type::kFisheye:
      //   dist_model = "fov";
      //   dist = acam.getDistortion().getParameters();
      //   break;
      default:
        LOG(FATAL) << "Unsupported distortion.";
        break;
    }
    const aslam::PinholeCamera* pinholecam = dynamic_cast<const aslam::PinholeCamera*>(&acam);
    CHECK_NOTNULL(pinholecam);
    Eigen::Matrix<double, 8, 1> cam_calib;
    cam_calib << pinholecam->fu(), pinholecam->fv(), pinholecam->cu(), pinholecam->cv(),
                 dist[0], dist[1], dist[2], dist[3];
    openvins_config->set_camera_intrinsics(
        cam_idx, dist_model, acam.imageWidth(), acam.imageHeight(), cam_calib);

    // Translate extrinsics.
    const aslam::Transformation T_C_B = aslam_cameras.get_T_C_B(cam_idx);
    openvins_config->set_camera_extrinsics(
        cam_idx, T_C_B.inverse().getTransformationMatrix());
  }
}
}  // namespace

ov_msckf::VioManager* constructAndConfigureOpenvins(
    const aslam::NCamera& camera_calibration,
    const vi_map::ImuSigmas& imu_config) {
  ov_msckf::VioManagerOptions params = loadOpenvinsConfig();
  setOpenvinsImuSigmas(imu_config, &params);
  convertAslamToOpenvinsCamera(camera_calibration, &params);
  return new ov_msckf::VioManager(params);
}
}  // namespace openvinsli
