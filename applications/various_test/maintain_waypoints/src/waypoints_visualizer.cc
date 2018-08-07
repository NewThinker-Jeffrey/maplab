#include "maintain_waypoints/waypoints_visualizer.h"

namespace waypoints {

namespace {

// draw waypoints with different colors.
visualization_msgs::Marker::_color_type getColorForIdx(int idx) {
  visualization_msgs::Marker::_color_type color;
  int mod4 = idx % 4;
  switch (mod4) {
    case 0:
      color.r = 1.0f;
      color.g = 1.0f;
      color.b = 1.0f;
      color.a = 1.0;
      break;
    case 1:
      color.r = 1.0f;
      color.g = 0.0f;
      color.b = 0.0f;
      color.a = 1.0;
      break;
    case 2:
      color.r = 0.0f;
      color.g = 1.0f;
      color.b = 0.0f;
      color.a = 1.0;
      break;
    case 3:
      color.r = 0.0f;
      color.g = 0.0f;
      color.b = 1.0f;
      color.a = 1.0;
      break;
  }
  return color;
}
}  // namespace

WaypointsVisualizer::WaypointsVisualizer(
    ros::NodeHandle* node, std::string topic, std::string frame_id,
    std::string ns)
    : m_topic(topic),
      m_namespace(ns),
      m_frame_id(frame_id),
      m_radius(0.1),
      m_shape(visualization_msgs::Marker::SPHERE) {
  if (m_namespace == "") {
    m_namespace = m_topic;
  }
  m_marker_pub = node->advertise<visualization_msgs::MarkerArray>(topic, 1);
}

WaypointsVisualizer::~WaypointsVisualizer() {}

void WaypointsVisualizer::addWaypoints(
    const std::vector<Eigen::Vector3d>& waypoints) {
  int size = m_waypoints.size() + waypoints.size();
  m_waypoints.reserve(size);
  for (auto waypoint : waypoints) {
    int idx = m_waypoints.size();
    m_waypoints.push_back(waypoint);
    addMarker(idx);
  }
}

void WaypointsVisualizer::clearWaypoints() {
  m_waypoints.clear();
  m_marker_array.markers.clear();
}

void WaypointsVisualizer::visualize() {
  m_marker_pub.publish(m_marker_array);
}

bool WaypointsVisualizer::waitForSubscriber() {
  while (m_marker_pub.getNumSubscribers() < 1) {
    if (!ros::ok()) {
      ROS_WARN("WaypointsVisualizer::waitForSubscriber: ros quitted");
      return false;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  ROS_INFO_ONCE("WaypointsVisualizer::waitForSubscriber: find subscriber");
  return true;
}

void WaypointsVisualizer::addMarker(int idx) {
  visualization_msgs::Marker marker;

  // Set the frame ID and timestamp.  See the TF tutorials for information on
  // these.
  marker.header.frame_id = m_frame_id;
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique
  // ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = m_namespace;
  marker.id = idx;

  // Set the marker type.  Initially this is CUBE, and cycles between that and
  // SPHERE, ARROW, and CYLINDER
  marker.type = m_shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3
  // (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the
  // frame/time specified in the header
  marker.pose.position.x = m_waypoints[idx][0];
  marker.pose.position.y = m_waypoints[idx][1];
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = m_radius;
  marker.scale.y = m_radius;
  marker.scale.z = m_radius;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color = getColorForIdx(idx);
  marker.lifetime = ros::Duration();
  m_marker_array.markers.push_back(marker);
}

void WaypointsVisualizer::setRadius(double radius) {
  m_radius = radius;
}

void WaypointsVisualizer::setShape(uint32_t shape) {
  m_shape = shape;
}

}  // namespace waypoints
