#ifndef OPENVINSLI_OPENVINS_FACTORY_H_
#define OPENVINSLI_OPENVINS_FACTORY_H_

#include <aslam/cameras/ncamera.h>
#include <openvins/OpenvinsInterfaceBuilder.hpp>
#include <sensors/imu.h>

namespace openvinsli {
openvins::OpenvinsInterface* constructAndConfigureOpenvins(
    const aslam::NCamera& camera_calibration,
    const vi_map::ImuSigmas& imu_sigmas);
}  // namespace openvinsli
#endif  // OPENVINSLI_OPENVINS_FACTORY_H_
