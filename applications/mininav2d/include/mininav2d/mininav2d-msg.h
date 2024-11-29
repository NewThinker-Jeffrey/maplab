#ifndef MININAV2D_MINI_NAV2D_MSG_H_
#define MININAV2D_MINI_NAV2D_MSG_H_

#include <Eigen/Core>

namespace mininav2d {
struct Nav2dCmd {
  // MAPLAB_POINTER_TYPEDEFS(Nav2dCmd);
  using ConstPtr = std::shared_ptr<const Nav2dCmd>;
  using Ptr = std::shared_ptr<Nav2dCmd>;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int64_t timestamp_ns;

  Eigen::Vector3d cur_pose2d;

  std::vector<Eigen::Vector3d> next_pathpoints;

  Eigen::Vector3d waypoint;

  bool is_last_pathpoint;

  static Nav2dCmd fromStr(const std::string& str) {
    Nav2dCmd cmd;
    std::istringstream is(str);
    size_t next_pathpoints_size;
    is >> cmd.timestamp_ns >> cmd.is_last_pathpoint >> next_pathpoints_size;
    is >> cmd.cur_pose2d.x() >> cmd.cur_pose2d.y() >> cmd.cur_pose2d.z();
    is >> cmd.waypoint.x() >> cmd.waypoint.y() >> cmd.waypoint.z();
    for (int i = 0; i < next_pathpoints_size; ++i) {
      Eigen::Vector3d pathpoint;
      is >> pathpoint.x() >> pathpoint.y() >> pathpoint.z();
      cmd.next_pathpoints.push_back(pathpoint);
    }
    return cmd;
  }

  std::string toStr() const {
    std::ostringstream oss;
    oss << timestamp_ns << " " << is_last_pathpoint << " "
        << next_pathpoints.size() << " " << cur_pose2d.x() << " "
        << cur_pose2d.y() << " " << cur_pose2d.z() << " "
        << waypoint.x() << " " << waypoint.y() << " " << waypoint.z();
    for (size_t i = 0; i < next_pathpoints.size(); ++i) {
      oss << " " << next_pathpoints[i].x() << " " << next_pathpoints[i].y()
          << " " << next_pathpoints[i].z();
    }
    return oss.str();
  }
};
}  // namespace mininav2d
#endif  // MININAV2D_MINI_NAV2D_MSG_H_
