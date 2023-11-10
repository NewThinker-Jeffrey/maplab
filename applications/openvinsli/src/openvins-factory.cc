#include "openvinsli/openvins-factory.h"

#include <Eigen/Core>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/distortion.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/file-system-tools.h>
#include <message-flow/message-flow.h>
#include <openvins/FilterConfiguration.hpp>
#include <openvins/OpenvinsInterfaceBuilder.hpp>
#include <vio-common/vio-types.h>

DEFINE_bool(
    openvins_enable_frame_visualization, true,
    "Set to false to disable the Openvins GUI.");
DEFINE_double(
    openvinsli_position_noise_density, 0.01,
    "Position prediction noise density [m/sqrt(s)].");

DEFINE_string(
    openvins_image_mask_path, "",
    "Path to image mask to be applied to the OPENVINS. No features are extracted "
    "on the masked areas. Currently only supports a single camera.");

namespace openvinsli {
namespace {
template <int kNumCameras>
struct OpenvinsBuilder {
  // kEnableMapLocalization: switches the localization mode of OPENVINS.
  static constexpr bool kEnableMapLocalization = true;
  // kNPoses: 0-off, 1: estimate baseframe, 2: (1) + sensor offset
  static constexpr int kNPoses = 1;
  // Maximal number of considered features in the filter state.
  static constexpr int kMaxNumFeatures = 25;
  // Total number of pyramid levels considered.
  static constexpr int kPyramidLevels = 4;
  // Edge length of the patches (in pixel). Must be a multiple of 2!
  static constexpr int kFeaturePatchSizePx = 6;
  openvins::OpenvinsInterface* operator()(
      const openvins::FilterConfiguration& filter_config,
      const openvins::CameraCalibrationVector& camera_calibrations) {
    return openvins::createOpenvinsInterface<
        kNumCameras, kEnableMapLocalization, kNPoses, kMaxNumFeatures,
        kPyramidLevels, kFeaturePatchSizePx>(
        filter_config, camera_calibrations);
  }
};

void convertAslamToOpenvinsCamera(
    const aslam::NCamera& aslam_cameras,
    openvins::CameraCalibrationVector* openvins_cameras) {
  CHECK_NOTNULL(openvins_cameras)->clear();

  const size_t num_cameras = aslam_cameras.numCameras();
  for (size_t cam_idx = 0u; cam_idx < num_cameras; ++cam_idx) {
    openvins::CameraCalibration openvins_camera;

    // Translate cameras intrinsic parameters.
    const aslam::Camera& aslam_camera = aslam_cameras.getCamera(cam_idx);
    CHECK_EQ(aslam_camera.getType(), aslam::Camera::Type::kPinhole)
        << "Only pinhole models supported.";
    openvins_camera.K_ = static_cast<const aslam::PinholeCamera&>(aslam_camera)
                          .getCameraMatrix();

    // Translate cameras distortion parameters.
    switch (aslam_camera.getDistortion().getType()) {
      case aslam::Distortion::Type::kNoDistortion:
        // The no-distortion case is emulated using a RADTAN distortion with
        // parameters that do not distort anything.
        openvins_camera.distortionModel_ = openvins::DistortionModel::RADTAN;
        openvins_camera.distortionParams_ = Eigen::Matrix<double, 5, 1>::Zero();
        break;
      case aslam::Distortion::Type::kRadTan:
        openvins_camera.distortionModel_ = openvins::DistortionModel::RADTAN;
        // Openvins uses 5 parameters (k1,k2,p1,p2,k3) we only use 4 (k1,k2,p1,p2)
        // therefore we pad with one zero.
        openvins_camera.distortionParams_.resize(5);
        openvins_camera.distortionParams_.head<4>() =
            aslam_camera.getDistortion().getParameters();
        openvins_camera.distortionParams_(4) = 0.0;
        break;
      case aslam::Distortion::Type::kEquidistant:
        openvins_camera.distortionModel_ = openvins::DistortionModel::EQUIDIST;
        openvins_camera.distortionParams_ =
            aslam_camera.getDistortion().getParameters();
        break;
      case aslam::Distortion::Type::kFisheye:
        openvins_camera.distortionModel_ = openvins::DistortionModel::FOV;
        openvins_camera.distortionParams_ =
            aslam_camera.getDistortion().getParameters();
        break;
      default:
        LOG(FATAL) << "Unsupported distortion.";
        break;
    }
    openvins_camera.hasIntrinsics_ = true;

    // Translate extrinsics.
    const aslam::Transformation T_C_B = aslam_cameras.get_T_C_B(cam_idx);
    const Eigen::Vector3d MrMC = T_C_B.inverse().getPosition();
    const kindr::RotationQuaternionPD qCM(
        T_C_B.getRotation().toImplementation());
    openvins_camera.setCameraExtrinsics(MrMC, qCM);
    openvins_cameras->emplace_back(openvins_camera);
  }
  CHECK_EQ(openvins_cameras->size(), num_cameras);
}

void initFilterConfigurationFromGFLags(
    openvins::FilterConfiguration* openvins_config) {
  CHECK_NOTNULL(openvins_config);
  openvins_config->setDoFrameVisualization(FLAGS_openvins_enable_frame_visualization);
  if (!FLAGS_openvins_image_mask_path.empty()) {
    openvins_config->setImageMaskPath(FLAGS_openvins_image_mask_path);
  }
}

void setOpenvinsImuSigmas(
    const vi_map::ImuSigmas& imu_sigmas,
    openvins::FilterConfiguration* openvins_config) {
  CHECK_NOTNULL(openvins_config);
  CHECK(imu_sigmas.isValid());

  const double position_noise_density_cov =
      FLAGS_openvinsli_position_noise_density *
      FLAGS_openvinsli_position_noise_density;
  openvins_config->setPositionCovarianceX(position_noise_density_cov);
  openvins_config->setPositionCovarianceY(position_noise_density_cov);
  openvins_config->setPositionCovarianceZ(position_noise_density_cov);

  const double acc_noise_density_cov =
      imu_sigmas.acc_noise_density * imu_sigmas.acc_noise_density;
  openvins_config->setAccCovarianceX(acc_noise_density_cov);
  openvins_config->setAccCovarianceY(acc_noise_density_cov);
  openvins_config->setAccCovarianceZ(acc_noise_density_cov);

  const double acc_bias_random_walk_noise_density_cov =
      imu_sigmas.acc_bias_random_walk_noise_density *
      imu_sigmas.acc_bias_random_walk_noise_density;
  openvins_config->setAccBiasCovarianceX(acc_bias_random_walk_noise_density_cov);
  openvins_config->setAccBiasCovarianceY(acc_bias_random_walk_noise_density_cov);
  openvins_config->setAccBiasCovarianceZ(acc_bias_random_walk_noise_density_cov);

  const double gyro_noise_density_cov =
      imu_sigmas.gyro_noise_density * imu_sigmas.gyro_noise_density;
  openvins_config->setGyroCovarianceX(gyro_noise_density_cov);
  openvins_config->setGyroCovarianceY(gyro_noise_density_cov);
  openvins_config->setGyroCovarianceZ(gyro_noise_density_cov);

  const double gyro_bias_random_walk_noise_density_cov =
      imu_sigmas.gyro_bias_random_walk_noise_density *
      imu_sigmas.gyro_bias_random_walk_noise_density;
  openvins_config->setGyroBiasCovarianceX(gyro_bias_random_walk_noise_density_cov);
  openvins_config->setGyroBiasCovarianceY(gyro_bias_random_walk_noise_density_cov);
  openvins_config->setGyroBiasCovarianceZ(gyro_bias_random_walk_noise_density_cov);
}

std::string getOpenvinsConfigurationTemplateFile() {
  const char* openvins_config_template_path = getenv("OPENVINS_CONFIG_DIR");
  CHECK_NE(openvins_config_template_path, static_cast<char*>(NULL))
      << "OPENVINS_CONFIG_DIR environment variable is not set.\n"
      << "Source the Maplab environment from your workspace:\n"
      << "source devel/setup.bash";
  std::string openvins_config_template_file(openvins_config_template_path);
  openvins_config_template_file += "/openvins_default_config.info";
  CHECK(common::fileExists(openvins_config_template_file))
      << "OPENVINS configuration template file does not exist: "
      << openvins_config_template_file;
  return openvins_config_template_file;
}
}  // namespace

openvins::OpenvinsInterface* constructAndConfigureOpenvins(
    const aslam::NCamera& camera_calibration,
    const vi_map::ImuSigmas& imu_config) {
  // Translate the camera calibration to OPENVINS format.
  size_t num_cameras = camera_calibration.getNumCameras();
  openvins::CameraCalibrationVector openvins_calibrations;
  convertAslamToOpenvinsCamera(camera_calibration, &openvins_calibrations);

  // Load default OPENVINS parameters from file and adapt where necessary.
  const std::string openvins_config_template_file =
      getOpenvinsConfigurationTemplateFile();
  VLOG(1) << "Loading OPENVINS configuration template: "
          << openvins_config_template_file;

  openvins::FilterConfiguration openvins_configuration(openvins_config_template_file);
  initFilterConfigurationFromGFLags(&openvins_configuration);
  setOpenvinsImuSigmas(imu_config, &openvins_configuration);

  openvins::OpenvinsInterface* openvins = nullptr;
  switch (num_cameras) {
    case 1u:
      openvins = OpenvinsBuilder<1>()(openvins_configuration, openvins_calibrations);
      break;
    case 2u:
      openvins = OpenvinsBuilder<2>()(openvins_configuration, openvins_calibrations);
      break;
    default:
      LOG(WARNING) << "OPENVINS support is only compiled for up to 2 cameras. "
                   << "Please adapt the code if you need more.";
  }

  return openvins;
}
}  // namespace openvinsli
