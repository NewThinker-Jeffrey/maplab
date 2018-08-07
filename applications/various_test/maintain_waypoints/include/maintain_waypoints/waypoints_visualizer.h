#ifndef MAINTAIN_WAYPOINTS_WAYPOINTS_VISUALIZER_H_
#define MAINTAIN_WAYPOINTS_WAYPOINTS_VISUALIZER_H_

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace waypoints {

class WaypointsVisualizer {
 public:
  WaypointsVisualizer(
      ros::NodeHandle* node, std::string topic, std::string frame_id = "/map",
      std::string m_namespace = "");
  ~WaypointsVisualizer();

  void setRadius(double radius);

  void setShape(uint32_t shape);

  void addWaypoints(const std::vector<Eigen::Vector3d>& waypoints);

  void clearWaypoints();

  void visualize();

  bool waitForSubscriber();

 private:
  void addMarker(int idx);
  ros::Publisher m_marker_pub;
  std::string m_topic;
  std::string m_frame_id;
  std::string m_namespace;
  std::vector<Eigen::Vector3d> m_waypoints;
  visualization_msgs::MarkerArray m_marker_array;
  double m_radius;
  uint32_t m_shape;
};
}  // namespace waypoints

#endif  // MAINTAIN_WAYPOINTS_WAYPOINTS_VISUALIZER_H_
