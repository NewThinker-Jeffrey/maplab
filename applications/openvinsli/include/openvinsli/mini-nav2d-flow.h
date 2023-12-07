#ifndef OPENVINSLI_MINI_NAV2D_FLOW_H_
#define OPENVINSLI_MINI_NAV2D_FLOW_H_

#include <functional>
#include <memory>
#include <vector>

#include <maplab-common/bidirectional-map.h>
#include <message-flow/message-flow.h>
#include <sensors/imu.h>
#include <vio-common/vio-types.h>

#include "openvinsli/openvins-estimate.h"
#include "openvinsli/openvins-factory.h"
#include "openvinsli/openvins-health-monitor.h"
#include "openvinsli/openvins-maplab-timetranslation.h"
#include "openvinsli/feature-tracking.h"

#include "openvinsli/mini-nav2d-msg.h"

// ros interface
#define EANBLE_ROS_NAV_INTERFACE
#ifdef EANBLE_ROS_NAV_INTERFACE
#include <ros/ros.h>
#include "openvinsli/RosNavRequest.h"
#include "openvinsli/RosNav2dCmd.h"
#endif

namespace openvinsli {


class Nav2dFlow {

 public:
  enum class NavState {
    IDLE = 0,
    PATH_RECORDING = 1,
    PATH_PLANNING = 2,
    NAVIGATING = 3
  };

  static std::string stateStr(NavState state) {
    switch(state) {
      case NavState::IDLE: return "IDLE";
      case NavState::PATH_RECORDING: return "PATH_RECORDING";
      case NavState::PATH_PLANNING: return "PATH_PLANNING";
      case NavState::NAVIGATING: return "NAVIGATING";
    }
    return "UNKOWN";
  }

  explicit Nav2dFlow();
  ~Nav2dFlow();

  void attachToMessageFlow(message_flow::MessageFlow* flow);

  //bool loadNavConfig(const std::string& nav_config_file); // deserialize
  
  bool startPathRecording();

  bool finishPathRecording(const std::string& savefile="");

  bool addTargetPoint(const std::string& target_name="");

  bool navigateTo(size_t target_idx);

  bool navigateTo(const std::string& target_name);

  bool stopNav();

  bool serialize(const std::string& nav_config_file);

  bool deserialize(const std::string& nav_config_file);

 public:  // for display

  struct NavInfoForDisplay {
    // will convert all compact 2d information into 3d.

    std::vector<Eigen::Vector3d> traj;  // set z=0 for all traj points.

    std::vector<Eigen::Isometry3d> nav_targets;

    std::vector<std::string> target_names;

    std::vector<Eigen::Vector3d> current_path;
    // Eigen::Vector3d current_pathpoint;

    std::string state;
  };

  std::shared_ptr<NavInfoForDisplay> getCurNavInfoForDisplay();

 private:

  void nav_worker();

  void processInput(const OpenvinsEstimate::ConstPtr& estimate);

  void tryAddingTrajPoint(const Eigen::Vector3d& traj_point);

  // find neareast
  size_t findNearstTrajPoint(const Eigen::Vector3d& current_pose_2d);

  // find p2p traj:
  std::vector<Eigen::Vector3d>  findPoint2PointTraj(size_t start_traj_point_idx, size_t target_point_idx);

 private:

  //// Do we need an occupancy mapping thread?
  // std::shared_ptr<std::thread> occupancy_mapping_thread_;  // subscribe depth data、 odometry.

  std::function<void(Nav2dCmd::ConstPtr)> publish_nav_;

  std::shared_ptr<std::thread> nav_thread_;

  std::mutex mutex_queue_;

  std::condition_variable cond_queue_;

  bool stop_request_;

  std::deque<OpenvinsEstimate::ConstPtr> vio_estimates_;



  // current nav task
  std::mutex mutex_nav_;

  // nav info
  std::vector<Eigen::Vector3d> traj_2d_;
  std::vector<size_t> target_points_;
  std::vector<std::string> target_point_names_;

  NavState state_;
  std::vector<Eigen::Vector3d> current_path_;
  size_t current_pathpoint_idx_;
  Eigen::Vector3d current_pose_2d_;
  size_t current_target_idx_;

  // std::deque<Eigen::Vector3d> traj_2d_recroding_;  // used for recording


#ifdef EANBLE_ROS_NAV_INTERFACE
 private:
  // ros interface
  ros::NodeHandle node_handle_;
  ros::ServiceServer ros_nav_srv_;
  ros::Publisher ros_pub_nav_cmd_;
  void initRosInterface();
  bool dealWithRosRequest(openvinsli::RosNavRequest::Request &request, openvinsli::RosNavRequest::Response &response);
  void convertAndPublishNavCmd(const Nav2dCmd& cmd);
#endif
};
}  // namespace openvinsli
#endif  // OPENVINSLI_MINI_NAV2D_FLOW_H_
