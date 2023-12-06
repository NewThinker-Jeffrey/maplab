#ifndef OPENVINSLI_FLOW_TOPICS_H_
#define OPENVINSLI_FLOW_TOPICS_H_
#include <message-flow/message-topic-registration.h>
#include <vi-map/vi-map.h>
#include <vio-common/map-update.h>
#include <vio-common/vio-types.h>

#include "openvinsli/mini-nav2d-msg.h"
#include "openvinsli/openvins-estimate.h"
#include "openvinsli/vi-map-with-mutex.h"

// TODO(schneith): All message should be ConstPtr.

// Raw sensor data.
MESSAGE_FLOW_TOPIC(IMAGE_MEASUREMENTS, vio::ImageMeasurement::Ptr);
MESSAGE_FLOW_TOPIC(IMU_MEASUREMENTS, vio::ImuMeasurement::Ptr);
MESSAGE_FLOW_TOPIC(ODOMETRY_MEASUREMENTS, vio::OdometryMeasurement::Ptr);

// Synchronized images from multiple cameras and IMU data.
MESSAGE_FLOW_TOPIC(SYNCED_NFRAMES_AND_IMU, vio::SynchronizedNFrameImu::Ptr);

MESSAGE_FLOW_TOPIC(
    TRACKED_NFRAMES_AND_IMU, vio::SynchronizedNFrameImu::ConstPtr);

// Output of the localizer.
MESSAGE_FLOW_TOPIC(LOCALIZATION_RESULT, vio::LocalizationResult::ConstPtr);

// Raw estimate of the VINS.
MESSAGE_FLOW_TOPIC(MAP_UPDATES, vio::MapUpdate::ConstPtr);

// Raw estimate output of OPENVINS.
MESSAGE_FLOW_TOPIC(OPENVINS_ESTIMATES, openvinsli::OpenvinsEstimate::ConstPtr);

// Nav2d output
MESSAGE_FLOW_TOPIC(NAV2D_CMD, openvinsli::Nav2dCmd::ConstPtr);

// Resulting map.
MESSAGE_FLOW_TOPIC(RAW_VIMAP, openvinsli::VIMapWithMutex::ConstPtr);

// All data input subscribers are put in an exclusivity group such that the
// delivery ordering for all messages (cam, imu, localization) are
// corresponding to the publishing order and no sensor can be left behind.
constexpr int kExclusivityGroupIdOpenvinsSensorSubscribers = 0;

#endif  // OPENVINSLI_FLOW_TOPICS_H_
