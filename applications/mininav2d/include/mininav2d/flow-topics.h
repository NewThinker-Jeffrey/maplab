#ifndef MININAV2D_FLOW_TOPICS_H_
#define MININAV2D_FLOW_TOPICS_H_
#include <message-flow/message-topic-registration.h>
#include "mininav2d/mininav2d-msg.h"
#include "hear_slam/common/pose/pose.h"

namespace mininav2d {
struct StampedGlobalPose {
  using Pose3d = hear_slam::Pose3d;
  int64_t timestamp_ns;
  Pose3d odom_pose;
  Pose3d global_pose;
  bool global_pose_valid;
  using ConstPtr = std::shared_ptr<const StampedGlobalPose>;
};
}  // namespace mininav2d

MESSAGE_FLOW_TOPIC(
    GLOBAL_POSE_FUSION, mininav2d::StampedGlobalPose::ConstPtr);

// Nav2d output
MESSAGE_FLOW_TOPIC(NAV2D_CMD, mininav2d::Nav2dCmd::ConstPtr);

// All data input subscribers are put in an exclusivity group such that the
// delivery ordering for all messages (cam, imu, localization) are
// corresponding to the publishing order and no sensor can be left behind.
constexpr int kExclusivityGroupIdMiniNav2dSensorSubscribers = 0;

#endif  // MININAV2D_FLOW_TOPICS_H_
