#ifndef OPENVINSLI_MINI_NAV2D_MSG_H_
#define OPENVINSLI_MINI_NAV2D_MSG_H_

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <maplab-common/macros.h>
#include <vio-common/vio-types.h>

namespace openvinsli {
struct Nav2dCmd {
  MAPLAB_POINTER_TYPEDEFS(Nav2dCmd);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int64_t timestamp_ns;

  Eigen::Vector3d cur_pose2d;

  std::vector<Eigen::Vector3d> next_pathpoints;

  bool is_last_pathpoint;
};
}  // namespace openvinsli
#endif  // OPENVINSLI_MINI_NAV2D_MSG_H_
