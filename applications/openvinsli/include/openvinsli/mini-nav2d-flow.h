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
#include "openvinsli/flow-topics.h"

#include "hear_slam/utils/yaml_helper.h"
#include "hear_slam/unstable/global_pose_fusion/pose_buffer.h"

// ros interface
#define EANBLE_ROS_NAV_INTERFACE
#ifdef EANBLE_ROS_NAV_INTERFACE
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
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

  // Save the nav_cmds to a file
  void beginSaveNavCmds(const std::string& filename);

  // For offline play mode
  void beginPlayNavCmds(const std::string& filename);

  //bool loadNavConfig(const std::string& nav_config_file); // deserialize
  
  bool startPathRecording();

  void setPathRecordFile(const std::string& filename);

  bool finishPathRecording(const std::string& savefile="");

  bool addWaypoint(const std::string& waypoint_name="");

  bool navigateToWaypoint(size_t waypoint_idx);

  bool navigateToWaypoint(const std::string& waypoint_name);

  bool navigateToObject(const std::string& object_name);

  bool stopNav();

  bool serialize(const std::string& nav_config_file);

  bool deserialize(const std::string& nav_config_file);

 public:  // for display

  struct NavInfoForDisplay {
    // will convert all compact 2d information into 3d.

    std::vector<Eigen::Vector3d> traj;  // set z=0 for all traj points.

    std::vector<StampedGlobalPose::Pose3d> nav_waypoints;

    std::vector<std::string> waypoint_names;

    std::vector<Eigen::Vector3d> current_path;
    // Eigen::Vector3d current_pathpoint;

    std::unique_ptr<StampedGlobalPose::Pose3d> T_O_G;  // global in odom.

    std::string state;
  };

  std::shared_ptr<NavInfoForDisplay> getCurNavInfoForDisplay();

 private:

  void nav_worker();

  // void processInput(const OpenvinsEstimate::ConstPtr& estimate);
  void processInput(const StampedGlobalPose::ConstPtr& estimate);

  void tryAddingTrajPoint(const Eigen::Vector3d& traj_point);

  // find neareast
  int findNearstTrajPoint(const Eigen::Vector3d& current_pose_2d, double* best_distance);

  // find p2p traj:
  std::vector<Eigen::Vector3d>  findToWaypointTraj(const Eigen::Vector3d& current_pose_2d, size_t waypoint_idx);

  struct NavToObjectParams;
  std::vector<Eigen::Vector3d> findToObjectTraj(const Eigen::Vector3d& current_pose_2d, const NavToObjectParams& nav_params);

  static double getPathLength(const Eigen::Vector3d& current_pose_2d, const std::vector<Eigen::Vector3d>& path);

  static std::vector<Eigen::Vector3d> filterPath(const std::vector<Eigen::Vector3d>& path);

  bool checkArrival() const;

  Nav2dCmd::Ptr runWaypointNav(int64_t timestamp_ns=-1);

  Nav2dCmd::Ptr runObjectNav(int64_t timestamp_ns=-1);

  void saveNavCmd(const Nav2dCmd& cmd);

  std::unique_ptr<StampedGlobalPose::Pose3d> getOdomPoseAtTime(int64_t timestamp_ns);

 private:

  //// Do we need an occupancy mapping thread?
  // std::shared_ptr<std::thread> occupancy_mapping_thread_;  // subscribe depth data、 odometry.

  std::function<void(Nav2dCmd::ConstPtr)> publish_nav_;

  std::shared_ptr<std::thread> nav_thread_;

  std::mutex mutex_queue_;

  std::condition_variable cond_queue_;

  bool stop_request_;

  // std::deque<OpenvinsEstimate::ConstPtr> vio_estimates_;
  std::deque<StampedGlobalPose::ConstPtr> vio_estimates_;

  // current nav task
  std::mutex mutex_nav_;

  // nav info
  std::vector<Eigen::Vector3d> traj_2d_;
  std::vector<size_t> waypoints_;
  std::vector<std::string> waypoint_names_;

  NavState state_;
  enum class NavType : uint8_t {TO_WAYPOINT, TO_OBJECT};
  NavType current_nav_type_;
  std::vector<Eigen::Vector3d> current_path_;
  size_t current_pathpoint_idx_;
  Eigen::Vector3d current_global_pose_2d_;
  Eigen::Vector3d current_odom_pose_2d_;

  // for TO_WAYPOINT
  size_t current_waypoint_idx_;

  // for TO_OBJECT
  struct NavToObjectParams {
    int ref_waypoint_idx = -1;
    std::string ref_waypoint_name;
    double max_forward_distance = 0.3;
    NavToObjectParams() {}

    using Config = hear_slam::YamlConfig;
    NavToObjectParams(const Config& config) : NavToObjectParams() {
      CONFIG_LOAD_I(config, ref_waypoint_name);
      CONFIG_LOAD_I(config, max_forward_distance);
    }
  };
  std::unique_ptr<NavToObjectParams> current_nav_to_object_params_;

  // std::deque<Eigen::Vector3d> traj_2d_recroding_;  // used for recording
  std::string path_record_file_;

 private:

  // saving online nav cmds
  std::shared_ptr<std::ofstream> nav_cmd_file_;

  int32_t new_nav_cmds_since_last_flush_ = 0;

  // For offline play
  std::vector<Nav2dCmd> nav_cmds_to_play_;

  Nav2dCmd::Ptr last_played_nav_cmd_;

  int32_t nav_cmd_play_idx_ = 0;

  std::unique_ptr<StampedGlobalPose::Pose3d> T_G_O_;

  std::unique_ptr<StampedGlobalPose::Pose3d> last_odom_pose_;

  using PoseBuffer = hear_slam::PoseBuffer;
  PoseBuffer odom_pose_buffer_;

  class PathTracking;
  std::unique_ptr<PathTracking> path_tracking_;

#ifdef EANBLE_ROS_NAV_INTERFACE
 private:
  // ros interface
  ros::NodeHandle node_handle_;
  ros::ServiceServer ros_nav_srv_;
  ros::Publisher ros_pub_nav_cmd_;
  ros::Publisher ros_pub_nav_cmd_viz_;
  ros::Publisher ros_pub_locomotion_cmd_;
  ros::Publisher ros_pub_nav_target_;

  ros::Subscriber sub_local_object_pose_;  // for grasping task (nav to object)
  StampedGlobalPose::Pose3d object_cam_extrinsics_;
  int64_t object_observation_timestamp_ns_ = -1;
  StampedGlobalPose::Pose3d local_object_pose_;
  std::unique_ptr<StampedGlobalPose::Pose3d> object_in_odom_frame_;
  std::mutex mutex_object_nav_;

  uint32_t ros_nav_cmd_seq_ = 0;
  uint32_t ros_locomotion_cmd_seq_ = 0;
  void initRosInterface();
  bool dealWithRosRequest(RosNavRequest::Request &request, RosNavRequest::Response &response);
  void localObjectPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void convertAndPublishNavCmd(const Nav2dCmd& cmd);
  void publishLocomotionCmd(int64_t time_ns, const Eigen::Vector3d& speed_2d);
  void publishGlobalNavInfoViz();

  // only used to stamp the ros messages in publishGlobalNavInfoViz()
  int64_t last_vio_estimate_timestamp_ns_ = -1;
#endif
};
}  // namespace openvinsli
#endif  // OPENVINSLI_MINI_NAV2D_FLOW_H_
