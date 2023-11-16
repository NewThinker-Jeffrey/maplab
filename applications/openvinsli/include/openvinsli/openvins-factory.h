#ifndef OPENVINSLI_OPENVINS_FACTORY_H_
#define OPENVINSLI_OPENVINS_FACTORY_H_

#include <aslam/cameras/ncamera.h>
#include <sensors/imu.h>

namespace ov_msckf {
class VioManager;
}  // namespace

namespace openvinsli {
ov_msckf::VioManager* constructAndConfigureOpenvins(
    const aslam::NCamera& camera_calibration,
    const vi_map::ImuSigmas& imu_sigmas);
}  // namespace openvinsli
#endif  // OPENVINSLI_OPENVINS_FACTORY_H_

