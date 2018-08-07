#include "maintain_waypoints/waypoints_utils.h"

#include <fstream>
#include <sstream>

#include "glog/logging.h"

namespace waypoints {

bool readWaypointsFromFile(
    std::string fn, std::vector<Eigen::Vector3d>* waypoints) {
  std::ifstream fs;
  fs.open(fn);
  int n;
  if (!(fs >> n)) {
    LOG(ERROR) << "read size error";
    fs.close();
    return false;
  }

  std::vector<Eigen::Vector3d> tmp_waypoints;
  tmp_waypoints.clear();
  double x, y, theta;
  for (int i = 0; i < n; i++) {
    if ((fs >> x) && (fs >> y) && (fs >> theta)) {
      tmp_waypoints.push_back(Eigen::Vector3d(x, y, theta));
    } else {
      LOG(ERROR) << "read waypoint error " << i;
      fs.close();
      return false;
    }
  }
  waypoints->swap(tmp_waypoints);
  fs.close();
  return true;
}

bool writeWaypointsToFile(
    std::string fn, const std::vector<Eigen::Vector3d>& waypoints) {
  std::ofstream fs;
  fs.open(fn);
  fs << waypoints.size() << std::endl;
  for (int i = 0; i < waypoints.size(); i++) {
    fs << waypoints[i][0] << '\t' << waypoints[i][1] << '\t' << waypoints[i][2]
       << std::endl;
  }
  fs.close();
  return true;
}

}  // namespace waypoints
