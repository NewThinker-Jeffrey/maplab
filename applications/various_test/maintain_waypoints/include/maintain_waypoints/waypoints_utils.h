#ifndef MAINTAIN_WAYPOINTS_WAYPOINTS_UTILS_H_
#define MAINTAIN_WAYPOINTS_WAYPOINTS_UTILS_H_

#include <Eigen/Core>
#include <string>
#include <vector>

namespace waypoints {

bool readWaypointsFromFile(
    std::string fn, std::vector<Eigen::Vector3d>* waypoints);

bool writeWaypointsToFile(
    std::string fn, const std::vector<Eigen::Vector3d>& waypoints);

}  // namespace waypoints

#endif  // MAINTAIN_WAYPOINTS_WAYPOINTS_UTILS_H_
