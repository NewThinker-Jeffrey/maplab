#include "maintain_waypoints/waypoints_remap.h"
#include "maintain_waypoints/waypoints_utils.h"
#include "maintain_waypoints/waypoints_visualizer.h"

#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

DEFINE_string(
    waypoints_file, "",
    "Path to the text file in which infomation about waypoints are stored");

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "visualize_waypt");
  ros::NodeHandle n;
  ros::Rate r(1);

  waypoints::WaypointsVisualizer visualizer(&n, "test_waypt");
  visualizer.setRadius(0.1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  std::vector<Eigen::Vector3d> waypts;
  if (FLAGS_waypoints_file != "") {
    if (waypoints::readWaypointsFromFile(FLAGS_waypoints_file, &waypts)) {
      std::cout << "read " << waypts.size() << " waypoints:" << std::endl;
      for (auto waypoint : waypts) {
        std::cout << waypoint[0] << "," << waypoint[1] << "," << waypoint[2]
                  << std::endl;
      }
    } else {
      std::cerr << "failed to read waypoints from " << FLAGS_waypoints_file
                << std::endl;
      return 1;
    }
  } else {
    std::cout << "FLAGS_waypoints_file hasn't been specified" << std::endl;
    std::cout << "Useage: " << argv[0] << " --waypoints_file <your-file>"
              << std::endl;
    return 1;
  }

  visualizer.waitForSubscriber();
  while (ros::ok()) {
    visualizer.setShape(shape);
    visualizer.clearWaypoints();
    visualizer.addWaypoints(waypts);
    visualizer.visualize();

    // Cycle between different shapes
    switch (shape) {
      case visualization_msgs::Marker::CUBE:
        shape = visualization_msgs::Marker::SPHERE;
        break;
      case visualization_msgs::Marker::SPHERE:
        shape = visualization_msgs::Marker::ARROW;
        break;
      case visualization_msgs::Marker::ARROW:
        shape = visualization_msgs::Marker::CYLINDER;
        break;
      case visualization_msgs::Marker::CYLINDER:
        shape = visualization_msgs::Marker::CUBE;
        break;
    }

    r.sleep();
  }
}
