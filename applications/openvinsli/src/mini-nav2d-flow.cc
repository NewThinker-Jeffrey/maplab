#include "openvinsli/mini-nav2d-flow.h"

#include <memory>
#include <random>
#include <string>
#include <vector>
#include <algorithm>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <aslam/common/pose-types.h>
#include <aslam/common/time.h>
#include <aslam/common/unique-id.h>
#include <gflags/gflags.h>
#include <maplab-common/fixed-size-queue.h>
#include <maplab-common/geometry.h>
#include <maplab-common/string-tools.h>
#include <message-flow/message-flow.h>
#include <vio-common/pose-lookup-buffer.h>
#include <vio-common/vio-types.h>

#include "openvinsli/flow-topics.h"
#include "openvinsli/openvins-factory.h"
#include "openvinsli/openvins-health-monitor.h"
#include "openvinsli/openvins-localization-handler.h"
#include "openvinsli/openvins-maplab-timetranslation.h"

#include "core/VioManager.h"         // ov_msckf
#include "core/VioManagerOptions.h"  // ov_msckf
#include "state/Propagator.h"        // ov_msckf
#include "state/State.h"             // ov_msckf
#include "state/StateHelper.h"       // ov_msckf
#include "utils/print.h"             // ov_core
#include "utils/sensor_data.h"       // ov_core

#include "hear_slam/basic/string_helper.h"
#include "hear_slam/basic/logging.h"

// ros and rviz interface
#ifdef EANBLE_ROS_NAV_INTERFACE
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization/common-rviz-visualization.h>
#include <visualization/rviz-visualization-sink.h>
#endif


#define PUBLISH_NAV_CMD_IN_ODOM_FRAME  // otherwise publish that in global frame

DEFINE_double(
    nav_record_path_point_dist, 0.5,
    "The average distance between two path points when recording path. (0.5)");

DEFINE_double(
    nav_plan_pathloop_thr, 0.6,
    "");

DEFINE_double(
    nav_safe_dist_to_path, -1.0,
    "If the current position is too far from the path, it might not be safe "
    "to start a navigation. This flag specifies the safe distance (from the current "
    "position to the nearest pathpoint). Negative value means an infinite safe distance.");

DEFINE_double(
    nav_swith_to_next_point_dist_thr, 0.25,
    "");

DEFINE_double(
    nav_fastforward_dist_thr, 0.75,
    "");


DEFINE_double(
    nav_arrival_dist_thr, 0.15,  // metre
    "");

DEFINE_double(
    nav_arrival_angle_thr, 0.05,  // rad
    "");


namespace openvinsli {

namespace {

double getYawFromQuaternion(const Eigen::Quaterniond& q, const Eigen::Vector3d& local_front) {
  Eigen::Vector3d front = q * local_front;
  // todo:make sure front.y() and front.x() fixed.
  return  atan2(front.y(), front.x());
}


Eigen::Vector3d transformPoseFrom3dTo2d(const StampedGlobalPose::Pose3d& pose_3d,
                                        // For our sensor, Z is forward.
                                        const Eigen::Vector3d& local_front = Eigen::Vector3d::UnitZ()) {
  Eigen::Vector3d pose_2d = pose_3d.translation();
  Eigen::Quaterniond q(pose_3d.linear().matrix());
  q.normalize();
  pose_2d[2] = getYawFromQuaternion(q, local_front);
  return pose_2d;
}

Eigen::Vector3d transformPoseFrom3dTo2d(const Eigen::Isometry3d& pose_3d,
                                        // For our sensor, Z is forward.
                                        const Eigen::Vector3d& local_front = Eigen::Vector3d::UnitZ()) {
  Eigen::Vector3d pose_2d = pose_3d.translation();
  Eigen::Quaterniond q(pose_3d.linear());
  q.normalize();
  pose_2d[2] = getYawFromQuaternion(q, local_front);
  return pose_2d;
}

Eigen::Matrix2d getRotation2d(const double yaw) {
  Eigen::Matrix2d rot;
  rot << cos(yaw), -sin(yaw),
         sin(yaw), cos(yaw);
  return rot;
}

Eigen::Vector3d inversePose2d(const Eigen::Vector3d& pose_2d) {
  Eigen::Vector3d inv;
  inv[2] = -pose_2d[2];
  inv.head<2>() = getRotation2d(inv[2]) * (- (pose_2d.head<2>()));
  return inv;
}

Eigen::Vector3d composePose2d(const Eigen::Vector3d& pose_1, const Eigen::Vector3d& pose_2) {
  Eigen::Vector3d ret;
  ret[2] = pose_1[2] + pose_2[2];
  if (ret[2] > M_PI) {
    ret[2] -= 2 * M_PI;
  } else if (ret[2] < -M_PI) {
    ret[2] += 2 * M_PI;
  }
  ret.head<2>() = getRotation2d(pose_1[2]) * pose_2.head<2>() + pose_1.head<2>();
  return ret;
}

Eigen::Vector3d transformPoseFrom3dTo2d(const aslam::Transformation& pose_3d, const Eigen::Vector3d& local_front = Eigen::Vector3d::UnitZ()) {
  Eigen::Vector3d pose_2d = pose_3d.getPosition();
  pose_2d[2] = getYawFromQuaternion(pose_3d.getRotation().toImplementation(), local_front);
  return pose_2d;
}

// Eigen::Vector3d getPose2dFromVioEstimate(OpenvinsEstimate::ConstPtr vio_estimate) {
//   CHECK(vio_estimate->has_T_G_M);
//   aslam::Transformation T_G_I = vio_estimate->T_G_M * vio_estimate->vinode.get_T_M_I();
//   return transformPoseFrom3dTo2d(T_G_I);
// }

// FrontZ can recover the real 3d pose of the camera, but not friendly for visualization.
StampedGlobalPose::Pose3d transformPoseFrom2dTo3d_FrontZ(const Eigen::Vector3d& pose_2d) {
  StampedGlobalPose::Pose3d pose_3d = StampedGlobalPose::Pose3d::Identity();

  Eigen::Vector3d Z(cos(pose_2d[2]), sin(pose_2d[2]), 0);
  Z.normalize();
  Eigen::Vector3d Y(0, 0, -1);
  Eigen::Vector3d X = Y.cross(Z);
  X.normalize();

  Eigen::Matrix3d rot;
  rot << X,Y,Z;

  Eigen::Vector3d translation = pose_2d;
  translation.z() = 0;

  pose_3d.linear() = rot;
  pose_3d.translation() = translation;
  return pose_3d;
}

// FrontX is more friendly for visualization.
StampedGlobalPose::Pose3d transformPoseFrom2dTo3d_FrontX(const Eigen::Vector3d& pose_2d) {
  StampedGlobalPose::Pose3d pose_3d = StampedGlobalPose::Pose3d::Identity();

  Eigen::Vector3d X(cos(pose_2d[2]), sin(pose_2d[2]), 0);
  X.normalize();
  Eigen::Vector3d Z(0, 0, 1);
  Eigen::Vector3d Y = Z.cross(X);
  Y.normalize();

  Eigen::Matrix3d rot;
  rot << X,Y,Z;

  Eigen::Vector3d translation = pose_2d;
  translation.z() = 0;

  pose_3d.linear() = rot;
  pose_3d.translation() = translation;
  return pose_3d;
}

Nav2dCmd::Ptr transformNav2dCmd(const Nav2dCmd& nav_cmd, const StampedGlobalPose::Pose3d& transform) {
  Nav2dCmd::Ptr ret_ptr;
  ret_ptr.reset(new Nav2dCmd());
  Nav2dCmd& ret = *ret_ptr;
  ret.timestamp_ns = nav_cmd.timestamp_ns;
  ret.is_last_pathpoint = nav_cmd.is_last_pathpoint;

  Eigen::Vector3d transform_2d = transformPoseFrom3dTo2d(transform, Eigen::Vector3d::UnitX());
  ret.cur_pose2d = composePose2d(transform_2d, nav_cmd.cur_pose2d);
  ret.waypoint = composePose2d(transform_2d, nav_cmd.waypoint);
  ret.next_pathpoints.clear();
  ret.next_pathpoints.reserve(nav_cmd.next_pathpoints.size());
  for (const auto& next_pathpoint : nav_cmd.next_pathpoints) {
    ret.next_pathpoints.push_back(composePose2d(transform_2d, next_pathpoint));
  }

  // ret.cur_pose2d = transformPoseFrom3dTo2d(transform * transformPoseFrom2dTo3d_FrontZ(nav_cmd.cur_pose2d));
  // ret.next_pathpoints.clear();
  // ret.next_pathpoints.reserve(nav_cmd.next_pathpoints.size());
  // for (const auto& next_pathpoint : nav_cmd.next_pathpoints) {    
  //   ret.next_pathpoints.push_back(
  //     transformPoseFrom3dTo2d(transform * transformPoseFrom2dTo3d_FrontZ(next_pathpoint)));
  // }

  return ret_ptr;
}

}  // namespace


Nav2dFlow::Nav2dFlow() : state_(NavState::IDLE), path_record_file_("nav.yaml") {

#ifdef EANBLE_ROS_NAV_INTERFACE
  initRosInterface();
#endif

  // get the object camera extrinsics (for grasping task)
  auto object_nav_config = hear_slam::rootCfg().get("object_nav");
  Eigen::Matrix4d object_cam_extrinsics = Eigen::Matrix4d::Identity();

  // CONFIG_UPDT_I(object_nav_config, object_cam_extrinsics);
  object_cam_extrinsics = hear_slam::yamlToMatrix(object_nav_config.mutableYaml()["object_cam_extrinsics"]);
      // This is a workround since maplab and hear_slam has different implementations for the conversion from yaml to Eigen matrix.
  std::cout << "object_cam_extrinsics: " << std::endl << object_cam_extrinsics << std::endl;

  object_cam_extrinsics_.linear() = object_cam_extrinsics.block<3, 3>(0, 0);
  object_cam_extrinsics_.translation() = object_cam_extrinsics.block<3, 1>(0, 3);

  stop_request_ = false;
  nav_thread_ = std::make_shared<std::thread>(std::bind(&Nav2dFlow::nav_worker, this));
}

Nav2dFlow::~Nav2dFlow() {
  if (nav_thread_ && nav_thread_->joinable()) {
    {
      std::unique_lock<std::mutex> lock(mutex_queue_);
      stop_request_ = true;
      cond_queue_.notify_one();
    }
    nav_thread_->join();
  }
  nav_thread_.reset();
}

void Nav2dFlow::attachToMessageFlow(message_flow::MessageFlow* flow) {
  CHECK_NOTNULL(flow);
  static constexpr char kSubscriberNodeName[] = "Nav2dFlow";

  publish_nav_ =
      flow->registerPublisher<message_flow_topics::NAV2D_CMD>();

  // //// subscribe odometry
  // flow->registerSubscriber<message_flow_topics::OPENVINS_ESTIMATES>(
  //     kSubscriberNodeName, message_flow::DeliveryOptions(),
  //     [this](const OpenvinsEstimate::ConstPtr& estimate) {
  //       CHECK(estimate);
  //       // process the estimate in a special thread?
  //       std::unique_lock<std::mutex> lock(mutex_queue_);
  //       vio_estimates_.push_back(estimate);
  //       cond_queue_.notify_one();
  //     });

  // subscribe global pose fusion.
  flow->registerSubscriber<message_flow_topics::GLOBAL_POSE_FUSION>(
      kSubscriberNodeName, message_flow::DeliveryOptions(),
      [this](const StampedGlobalPose::ConstPtr& global_pose) {
        CHECK(global_pose != nullptr);
        // process the global_pose in a special thread.
        std::unique_lock<std::mutex> lock(mutex_queue_);
        vio_estimates_.push_back(global_pose);
        cond_queue_.notify_one();
      });

  // maybe we also need an occupancy-mapping-flow?

  //// todo? update occupancy grid map with depth image data.
  ////       also need to sync depth and odometry (without reloc) data.
  // flow->registerSubscriber<message_flow_topics::DEPTH_IMAGE>(
  //     kSubscriberNodeName, message_flow::DeliveryOptions(),
  //     [this](const DepthImage::ConstPtr& depth) {
  //       CHECK(depth);          
  //     });
}


bool Nav2dFlow::startPathRecording() {
  std::unique_lock<std::mutex> lock(mutex_nav_);
  if (state_ == NavState::IDLE) {
    state_ = NavState::PATH_RECORDING;
    traj_2d_.clear();
    waypoints_.clear();
    waypoint_names_.clear();
    LOG(INFO) << "Nav2dFlow: startPathRecording() OK!";
    return true;
  }

  LOG(WARNING) << "Nav2dFlow: startPathRecording() failed since we're in IDLE state!";
  return false;
}

void Nav2dFlow::setPathRecordFile(const std::string& filename) {
  path_record_file_ = filename;
}

bool Nav2dFlow::finishPathRecording(const std::string& tmp_savefile) {
  std::string savefile = tmp_savefile;
  if (savefile.empty()) {
    savefile = path_record_file_;
  }
  std::unique_lock<std::mutex> lock(mutex_nav_);
  if (state_ == NavState::PATH_RECORDING) {
    serialize(savefile);
    // traj_2d_.clear();
    // waypoints_.clear();
    // waypoint_names_.clear();
    LOG(INFO) << "Nav2dFlow: finishPathRecording() OK!";
    state_ = NavState::IDLE;
    return true;
  }

  LOG(WARNING) << "Nav2dFlow: finishPathRecording() failed since we're in a wrong state!";
  return false;
}

bool Nav2dFlow::addWaypoint(const std::string& waypoint_name) {
  std::unique_lock<std::mutex> lock(mutex_nav_);
  if (state_ == NavState::PATH_RECORDING) {
    int new_waypoint_idx = waypoints_.size();
    waypoints_.push_back(traj_2d_.size() - 1);
    std::string name = waypoint_name;
    if (name.empty()) {
      name = "waypoint_" + std::to_string(new_waypoint_idx);
    }
    waypoint_names_.push_back(name);
    LOG(INFO) << "Nav2dFlow: addWaypoint() OK!";
    return true;
  }

  LOG(WARNING) << "Nav2dFlow: addWaypoint() failed since we're in a wrong state!";
  return false;
}

bool Nav2dFlow::navigateToWaypoint(size_t waypoint_idx) {
  std::unique_lock<std::mutex> lock(mutex_nav_);
  if (state_ == NavState::IDLE) {
    state_ = NavState::PATH_PLANNING;
    current_nav_type_ = NavType::TO_WAYPOINT;
    current_waypoint_idx_ = waypoint_idx;
    LOG(INFO) << "Nav2dFlow: navigateToWaypoint() OK!";
    return true;
  }

  LOG(WARNING) << "Nav2dFlow: navigateToWaypoint() failed since we're  NOT in IDLE state!";
  return false;
}

bool Nav2dFlow::navigateToWaypoint(const std::string& waypoint_name) {
  for (size_t i=0; i<waypoint_names_.size(); i++) {
    if (waypoint_names_[i] == waypoint_name) {
      return navigateToWaypoint(i);
    }
  }

  LOG(WARNING) << "Nav2dFlow: navigateToWaypoint() failed since we can't find a waypoint named \"" << waypoint_name << "\"";
  return false;
}

bool Nav2dFlow::navigateToObject(const std::string& object_name) {
  if (object_name.empty()) {
    LOG(WARNING) << "Nav2dFlow: navigateToObject() failed since we get an empty object name/type!!";
    return false;
  }

  std::string config_key = "object_nav.objects." + object_name;
  if (!hear_slam::rootCfg().has(config_key)) {
    LOG(WARNING) << "Nav2dFlow: navigateToObject() failed since the object is not defined!! name/type: " << object_name;
    return false;
  }

  auto nav_to_object_params = std::make_unique<NavToObjectParams>(hear_slam::rootCfg().get(config_key));
  for (size_t i=0; i<waypoint_names_.size(); i++) {
    if (waypoint_names_[i] == nav_to_object_params->ref_waypoint_name) {
      nav_to_object_params->ref_waypoint_idx = i;
      break;
    }
  }

  std::unique_lock<std::mutex> lock(mutex_nav_);
  if (state_ == NavState::IDLE) {
    state_ = NavState::PATH_PLANNING;
    current_nav_type_ = NavType::TO_OBJECT;
    current_nav_to_object_params_ = std::move(nav_to_object_params);
    LOG(INFO) << "Nav2dFlow: navigateToObject() OK!";
    return true;
  }

  LOG(WARNING) << "Nav2dFlow: navigateToObject() failed since we're NOT in IDLE state!";
  return false;
}

bool Nav2dFlow::stopNav() {
  std::unique_lock<std::mutex> lock(mutex_nav_);
  if (state_ == NavState::NAVIGATING) {
    LOG(INFO) << "Nav2dFlow: stopNav() OK!";
    state_ = NavState::IDLE;
    return true;
  }

  LOG(WARNING) << "Nav2dFlow: stopNav() failed since we're in a wrong state!";
  return false;
}

bool Nav2dFlow::deserialize(const std::string& nav_config_file) {
  auto p_obj = hear_slam::loadYaml(nav_config_file);
  hear_slam::YamlObj& obj = *p_obj;
  auto waypoints = obj["waypoints"];
  waypoints_.resize(waypoints.size());
  waypoint_names_.resize(waypoints.size());
  for (size_t i=0; i < waypoints.size(); i++) {
    auto waypoint = waypoints[i];
    waypoint_names_[i] = waypoint["name"].as<std::string>();
    waypoints_[i] = waypoint["traj_pidx"].as<int>();
  }

  auto traj_points = obj["traj_points"];
  traj_2d_.resize(traj_points.size());
  for (size_t i=0; i < traj_points.size(); i++) {
    auto traj_point = traj_points[i];
    traj_2d_[i][0] = traj_point[0].as<double>();
    traj_2d_[i][1] = traj_point[1].as<double>();
    traj_2d_[i][2] = traj_point[2].as<double>();
  }

  return true;
}

bool Nav2dFlow::serialize(const std::string& nav_config_file) {
  hear_slam::YamlObj obj;
  for (size_t i=0; i<waypoints_.size(); i++) {
    hear_slam::YamlObj cur_waypoint;
    cur_waypoint["name"] = waypoint_names_[i];
    cur_waypoint["traj_pidx"] = waypoints_[i];
    obj["waypoints"].push_back(cur_waypoint);
  }

  for (size_t i=0; i<traj_2d_.size(); i++) {
    hear_slam::YamlObj cur_point;
    cur_point.push_back(traj_2d_[i][0]);
    cur_point.push_back(traj_2d_[i][1]);
    cur_point.push_back(traj_2d_[i][2]);
    obj["traj_points"].push_back(cur_point);
  }

  hear_slam::saveYaml(nav_config_file, obj);
  return true;
}


void Nav2dFlow::nav_worker() {
  while (1) {
    // get new odometry measurement.
    // OpenvinsEstimate::ConstPtr vio_estimate = nullptr;
    StampedGlobalPose::ConstPtr vio_estimate = nullptr;
    {
      std::unique_lock<std::mutex> lock(mutex_queue_);
      cond_queue_.wait(lock, [this](){
        return ! vio_estimates_.empty() || stop_request_;
      });

      if (stop_request_) {
        break;
      }

      // vio_estimate = vio_estimates_.front();
      // vio_estimates_.pop_front();

      // get the latest esitmate
      vio_estimate = vio_estimates_.back();
      if (vio_estimates_.size() > 1) {
        LOG(WARNING) << "Nav2dFlow: abandon " << vio_estimates_.size() - 1 << " poses.";
      }
      vio_estimates_.clear();
    }

    CHECK(vio_estimate);
    processInput(vio_estimate);
  }

  // std::cout << "end of nav_worker()." << std::endl;
}

// void Nav2dFlow::processInput(const OpenvinsEstimate::ConstPtr& vio_estimate) {
void Nav2dFlow::processInput(const StampedGlobalPose::ConstPtr& vio_estimate) {
  // Deal with vio_estimate.

  // TODO: Check the input to determine if it is valid.
  //   NavToWaypoint requires global pose, while NavToObject only needs local poses.
  bool cur_global_pose_valid = true;

  std::unique_lock<std::mutex> lock(mutex_nav_);

  if (!last_odom_pose_) {
    last_odom_pose_ = std::make_unique<StampedGlobalPose::Pose3d>();
  }
  *last_odom_pose_ = vio_estimate->odom_pose;

#ifdef EANBLE_ROS_NAV_INTERFACE
  last_vio_estimate_timestamp_ns_ = vio_estimate->timestamp_ns;
#endif

  if (cur_global_pose_valid) {
    if (!T_G_O_) {
      T_G_O_ = std::make_unique<StampedGlobalPose::Pose3d>();
    }
    *T_G_O_ = vio_estimate->odom_pose.inverse() * vio_estimate->global_pose;
#ifdef EANBLE_ROS_NAV_INTERFACE
    publishGlobalNavInfoViz();
#endif
  }

  // We only play back nav_cmds if global pose is valid.
  if (cur_global_pose_valid && !nav_cmds_to_play_.empty()) {
    // If we're in offline mode, we just play back the recorded nav_cmds.

    while (nav_cmd_play_idx_ < nav_cmds_to_play_.size()) {
      const auto& cur_nav_cmd = nav_cmds_to_play_[nav_cmd_play_idx_];
      const int64_t time_ns_precision = 10;
      if (cur_nav_cmd.timestamp_ns <= vio_estimate->timestamp_ns + time_ns_precision) {
        Nav2dCmd::Ptr nav_cmd = aligned_shared<Nav2dCmd>(cur_nav_cmd);
        state_ = NavState::NAVIGATING;

#ifdef PUBLISH_NAV_CMD_IN_ODOM_FRAME
        Nav2dCmd::Ptr nav_cmd_in_odom_frame = transformNav2dCmd(*nav_cmd, T_G_O_->inverse());
        Nav2dCmd::Ptr nav_cmd_to_publish = nav_cmd_in_odom_frame;
#else
        Nav2dCmd::Ptr nav_cmd_to_publish = nav_cmd;
#endif


#ifdef EANBLE_ROS_NAV_INTERFACE
        convertAndPublishNavCmd(*nav_cmd_to_publish);
#endif
        publish_nav_(nav_cmd_to_publish);
        nav_cmd_play_idx_ ++;
        last_played_nav_cmd_ = nav_cmd;
      } else {
        break;
      }
    }
  
    // If we've lost nav cmds for 300ms in offline play mode, set the state to IDLE.
    if (last_played_nav_cmd_ && vio_estimate->timestamp_ns - last_played_nav_cmd_->timestamp_ns > 300000000) {
      state_ = NavState::IDLE;
    }

    return;
  }

  if (cur_global_pose_valid) {
    current_global_pose_2d_ = transformPoseFrom3dTo2d(vio_estimate->global_pose);
  }
  current_odom_pose_2d_ = transformPoseFrom3dTo2d(vio_estimate->odom_pose);

  if (NavState::PATH_RECORDING == state_) {
    tryAddingTrajPoint(current_global_pose_2d_);
  } else if (NavState::NAVIGATING == state_) {
    // For now we skip the following check since it will be done by the downstream controller.
    // // Check whether the robot has reached the waypoint point.
    // if (checkArrival()) {
    //   LOG(WARNING) << "Nav2dFlow:  Finished navigation.";
    //   state_ = NavState::IDLE;
    //   return;
    // }
    
    if (current_nav_type_ == NavType::TO_WAYPOINT) {
      if (!cur_global_pose_valid) {
        LOG(WARNING) << "Nav2dFlow:  Global pose is not valid!!  Skip updating nav command!";
        return;
      }

      Nav2dCmd::Ptr nav_cmd = runWaypointNav(vio_estimate->timestamp_ns);
#ifdef PUBLISH_NAV_CMD_IN_ODOM_FRAME
      Nav2dCmd::Ptr nav_cmd_in_odom_frame = transformNav2dCmd(*nav_cmd, T_G_O_->inverse());
      Nav2dCmd::Ptr nav_cmd_to_publish = nav_cmd_in_odom_frame;
#else
      Nav2dCmd::Ptr nav_cmd_to_publish = nav_cmd;
#endif

#ifdef EANBLE_ROS_NAV_INTERFACE
      convertAndPublishNavCmd(*nav_cmd_to_publish);
#endif
      publish_nav_(nav_cmd_to_publish);
      saveNavCmd(*nav_cmd);  // Save the nav_cmd for offline playback.
    } else {
      ASSERT(current_nav_type_ == NavType::TO_OBJECT);
      // For object navigation, the output nav_cmd of runObjectNav() is already in
      // the odom frame.
      Nav2dCmd::Ptr nav_cmd = runObjectNav(vio_estimate->timestamp_ns);
#ifdef EANBLE_ROS_NAV_INTERFACE
      convertAndPublishNavCmd(*nav_cmd);
#endif
      publish_nav_(nav_cmd);

      // cmd saved for offline playback must be in the global frame
      if (cur_global_pose_valid) {
        saveNavCmd(*transformNav2dCmd(*nav_cmd, *T_G_O_));
      }
    }
  } else if (NavState::PATH_PLANNING == state_) {
    // find the path
    if (current_nav_type_ == NavType::TO_WAYPOINT) {
      if (!cur_global_pose_valid) {
        LOG(WARNING) << "Nav2dFlow:  Global pose is not valid!!  Skip path planning and wait for next pose update.";
        return;
      }
      current_path_ = findToWaypointTraj(current_global_pose_2d_, current_waypoint_idx_);
      if (current_path_.size() > 0) {
        current_pathpoint_idx_ = 0;
        state_ = NavState::NAVIGATING;
      } else {
        LOG(WARNING) << "Nav2dFlow:  No path found to waypoint. Wait for next pose update.";
        // state_ = NavState::IDLE;
      }
    } else {
      ASSERT(current_nav_type_ == NavType::TO_OBJECT);
      current_path_ = findToObjectTraj(current_global_pose_2d_, *current_nav_to_object_params_);
      if (current_path_.size() > 0) {
        current_pathpoint_idx_ = 0;
        state_ = NavState::NAVIGATING;
      } else {
        LOG(WARNING) << "Nav2dFlow:  No path found to object. Wait for next pose update.";
        // state_ = NavState::IDLE;
      }
    }
  } else if (NavState::IDLE == state_) {
    
  }
}


void Nav2dFlow::tryAddingTrajPoint(const Eigen::Vector3d& traj_point) {

  size_t n = traj_2d_.size();
  if (n < 2) {
    traj_2d_.push_back(traj_point);
    return;
  }

  if (!waypoints_.empty()) {
    // ensure our waypoint points won't be removed
    size_t newest_waypoint_pidx = waypoints_.back();
    if (n < newest_waypoint_pidx + 2) {
      traj_2d_.push_back(traj_point);
      return;
    }
  }


  Eigen::Vector3d diff_latest = traj_2d_[n-1] - traj_2d_[n-2];
  diff_latest.z() = 0;
  double dist_latest = diff_latest.norm();
  if (dist_latest < FLAGS_nav_record_path_point_dist) {
    // replace the last with our new traj_point
    traj_2d_[n-1] = traj_point;
  } else {
    traj_2d_.push_back(traj_point);
  }
}

std::vector<Eigen::Vector3d> Nav2dFlow::filterPath(const std::vector<Eigen::Vector3d>& path) {
  std::vector<Eigen::Vector3d> filtered_path;
  if (path.size() < 3) {
    return filtered_path;
  }
  filtered_path.reserve(path.size());
  filtered_path.push_back(path[0]);

  for (int i = 1; i < path.size() - 1; i++) {
    const Eigen::Vector3d& cur_point = path[i];

    // Check whether there is a path loop.
    // Note:
    //   - the first path point is ensured not to be removed.
    //   - 'j+1 < filtered_path.size()' prevents nonsense loops between neibour points.
    for (int j=0; j+1<filtered_path.size(); j++) {
      const Eigen::Vector3d& old_point = filtered_path[j];
      Eigen::Vector3d diff = cur_point - old_point;
      diff.z() = 0.0;
      double dist = diff.norm();
      if (dist < FLAGS_nav_plan_pathloop_thr) {
        filtered_path.erase(filtered_path.begin() + j + 1, filtered_path.end());
        break;
      }
    }
    filtered_path.push_back(cur_point);
  }

  // Always keep the last point.
  filtered_path.push_back(path.back());

  CHECK_EQ(filtered_path.front(), path.front());
  CHECK_EQ(filtered_path.back(), path.back());

  return filtered_path;
}

double Nav2dFlow::getPathLength(const Eigen::Vector3d& current_pose_2d, const std::vector<Eigen::Vector3d>& path) {
  double length = 0.0;
  for (size_t i=0; i<path.size(); i++) {
    Eigen::Vector3d diff;
    if (i == 0) {
      diff = path[i] - current_pose_2d;
    } else {
      diff = path[i] - path[i-1];
    }
    diff.z() = 0.0;
    length += diff.norm();
  }
  return length;
}

int Nav2dFlow::findNearstTrajPoint(const Eigen::Vector3d& current_pose_2d, double* pbest_distance) {
  int best_i = 0;
  double& best_distance = *pbest_distance;
  best_distance = std::numeric_limits<double>::max();
  for (size_t i=0; i<traj_2d_.size(); i++) {
    Eigen::Vector3d diff = traj_2d_[i] - current_pose_2d;
    diff.z() = 0;
    double distance = diff.norm();
    if (distance < best_distance) {
      best_distance = distance;
      best_i = i;
    }
  }
  return best_i;
}

std::vector<Eigen::Vector3d> Nav2dFlow::findToObjectTraj(
    const Eigen::Vector3d& current_pose_2d, const NavToObjectParams& nav_params) {
  StampedGlobalPose::Pose3d object_in_odom_frame;
  {
    std::unique_lock<std::mutex> lock(mutex_object_nav_);
    if (object_in_odom_frame_) {
      object_in_odom_frame = *object_in_odom_frame_;
    } else {
      LOG(WARNING) << "Nav2dFlow: Can't find traj to the object since "
                   << "the pose of the object is unkonwn!!"; 
      return std::vector<Eigen::Vector3d>();
    }
  }

  Eigen::Vector3d object_in_odom_frame_2d = transformPoseFrom3dTo2d(object_in_odom_frame, Eigen::Vector3d::UnitX());
  Eigen::Vector2d object_xy = object_in_odom_frame_2d.head<2>();
  double max_forward_distance = nav_params.max_forward_distance;
  int ref_traj_point_idx = waypoints_[nav_params.ref_waypoint_idx];
  Eigen::Vector3d ref_traj_point = traj_2d_[ref_traj_point_idx];
  Eigen::Vector2d ref_xy = ref_traj_point.head<2>();

  Eigen::Vector3d target;
  target[2] = ref_traj_point[2];
  Eigen::Matrix2d rot2d = getRotation2d(ref_traj_point[2]);

  Eigen::Vector2d object_xy_in_ref = rot2d.transpose() * (object_xy - ref_xy);
  Eigen::Vector2d target_xy_in_ref = object_xy_in_ref;
  // target_xy_in_ref[0] = std::min(object_xy_in_ref[0], max_forward_distance);
  target_xy_in_ref[0] = max_forward_distance;
  Eigen::Vector2d target_xy = rot2d * target_xy_in_ref + ref_xy;
  target.head<2>() = target_xy;

  return std::vector<Eigen::Vector3d>({target});
}

std::vector<Eigen::Vector3d>  Nav2dFlow::findToWaypointTraj(
    const Eigen::Vector3d& current_pose_2d, size_t waypoint_idx) {
  std::vector<Eigen::Vector3d> path;

  double nearest_distance = std::numeric_limits<double>::max();
  int nearest_traj_point_idx = findNearstTrajPoint(current_pose_2d, &nearest_distance);
  if (FLAGS_nav_safe_dist_to_path > 0 && nearest_distance > FLAGS_nav_safe_dist_to_path) {
    // Return empty path if the current position is too far from the path.
    return path;
  }

  double dist_thr = std::max(FLAGS_nav_fastforward_dist_thr, nearest_distance + 0.1);
  int end_traj_point_idx = waypoints_[waypoint_idx];
  std::vector<Eigen::Vector3d> left_path;
  std::vector<Eigen::Vector3d> right_path;

  // left path
  int left_start_traj_point_idx = -1;
  for (int left_i=end_traj_point_idx; left_i>=0; left_i--) {
    Eigen::Vector3d diff = current_pose_2d - traj_2d_[left_i];
    diff.z() = 0;
    double dist = diff.norm();
    if (dist < dist_thr) {
      left_start_traj_point_idx = left_i;
      break;
    }
  }
  if (left_start_traj_point_idx >= 0) {
    left_path = std::vector<Eigen::Vector3d>(
        traj_2d_.begin() + left_start_traj_point_idx,
        traj_2d_.begin() + end_traj_point_idx + 1);
    left_path = filterPath(left_path);
  }
  
  // right path
  int right_start_traj_point_idx = -1;
  for (int right_i=end_traj_point_idx; right_i<traj_2d_.size(); right_i++) {
    Eigen::Vector3d diff = current_pose_2d - traj_2d_[right_i];
    diff.z() = 0;
    double dist = diff.norm();
    if (dist < dist_thr) {
      right_start_traj_point_idx = right_i;
      break;
    }
  }
  if (right_start_traj_point_idx >= 0) {
    right_path = std::vector<Eigen::Vector3d>(
        traj_2d_.begin() + end_traj_point_idx,
        traj_2d_.begin() + right_start_traj_point_idx + 1);
    std::reverse(right_path.begin(), right_path.end());
    right_path = filterPath(right_path);
  }

  // return path
  if (left_path.empty()) {
    return right_path;
  } else if (right_path.empty()) {
    return left_path;
  }

  if (getPathLength(current_pose_2d, left_path) <
      getPathLength(current_pose_2d, right_path)) {
    return left_path;
  } else {
    return right_path;
  }
}

bool Nav2dFlow::checkArrival() const {
  if (current_pathpoint_idx_ + 1 < current_path_.size()) {
    return false;
  }

  CHECK_EQ(current_pathpoint_idx_, current_path_.size() - 1);
  Eigen::Vector3d waypoint_pose_2d = current_path_[current_pathpoint_idx_];
  Eigen::Vector3d diff = current_global_pose_2d_ - waypoint_pose_2d;

  double theta_diff = diff.z();
  if (theta_diff > M_PI) {
    theta_diff -= 2.0 * M_PI;
  } else if (theta_diff < -M_PI) {
    theta_diff += 2.0 * M_PI;
  }
  theta_diff = std::abs(theta_diff);

  diff.z() = 0;
  double dist = diff.norm();

  if (dist <= FLAGS_nav_arrival_dist_thr && theta_diff <= FLAGS_nav_arrival_angle_thr) {
    return true;
  } else {
    return false;
  }
}

Nav2dCmd::Ptr Nav2dFlow::runObjectNav(int64_t timestamp_ns) {
  Nav2dCmd::Ptr nav_cmd = aligned_shared<Nav2dCmd>();
  nav_cmd->timestamp_ns = timestamp_ns;
  nav_cmd->cur_pose2d = current_odom_pose_2d_;

  // Replan path every time
  auto new_path = findToObjectTraj(current_global_pose_2d_, *current_nav_to_object_params_);
  if (new_path.size() > 0) {
    std::swap(current_path_, new_path);
    current_pathpoint_idx_ = 0;
  } else {
    LOG(WARNING) << "Nav2dFlow:  No new path found to object! keep the previes path.";
    // state_ = NavState::IDLE;
  }

  nav_cmd->waypoint = current_path_.back();
  nav_cmd->next_pathpoints = current_path_;
  nav_cmd->is_last_pathpoint = (nav_cmd->next_pathpoints.size() == 1);
  return nav_cmd;
}

Nav2dCmd::Ptr Nav2dFlow::runWaypointNav(int64_t timestamp_ns) {
  Nav2dCmd::Ptr nav_cmd = aligned_shared<Nav2dCmd>();
  nav_cmd->timestamp_ns = timestamp_ns;
  nav_cmd->cur_pose2d = current_global_pose_2d_;
  nav_cmd->waypoint = current_path_.back();

  if (current_pathpoint_idx_ + 1 < current_path_.size()) {
    // Check whether we need to change current_pathpoint_idx_.
    int fastforward_step = 0;  // default to zero

    // fastforward as many steps as possible.
    for (size_t step = 1; current_pathpoint_idx_+step < current_path_.size(); step++) {
      Eigen::Vector3d next_pathp = current_path_[current_pathpoint_idx_+step];
      Eigen::Vector3d diff = next_pathp - current_global_pose_2d_;
      diff.z() = 0;
      double dist = diff.norm();
      if (dist < FLAGS_nav_fastforward_dist_thr) {
        fastforward_step = step;
      }
    }

    if (fastforward_step > 0) {
      current_pathpoint_idx_ += fastforward_step;
    } else {
      Eigen::Vector3d cur_pathp = current_path_[current_pathpoint_idx_];
      Eigen::Vector3d diff = cur_pathp - current_global_pose_2d_;
      diff.z() = 0;
      double dist = diff.norm();

      if (dist < FLAGS_nav_swith_to_next_point_dist_thr) {
        current_pathpoint_idx_ += 1;
      }
    }
    
    for (size_t i=0; i < 10 && current_pathpoint_idx_+i < current_path_.size(); i++) {
      nav_cmd->next_pathpoints.push_back(current_path_[current_pathpoint_idx_+i]);
    }
    nav_cmd->is_last_pathpoint = !(current_pathpoint_idx_ + 1 < current_path_.size());
  } else {
    nav_cmd->next_pathpoints.push_back(current_path_[current_pathpoint_idx_]);
    // nav_cmd->is_last_pathpoint = !(current_pathpoint_idx_ + 1 < current_path_.size());
    nav_cmd->is_last_pathpoint = true;
  }

  bool re_orient_intermediate_pathpoints = true;
  if (re_orient_intermediate_pathpoints) {
    // Re-orient intermediate pathpoints.
    if (nav_cmd->next_pathpoints.size() > 1) {
      for (size_t i=0; i<nav_cmd->next_pathpoints.size() - 1; i++) {
        Eigen::Vector3d diff = nav_cmd->next_pathpoints[i+1] - nav_cmd->next_pathpoints[i];
        diff.z() = 0;
        nav_cmd->next_pathpoints[i].z() = std::atan2(diff.y(), diff.x());
      }
    }
  }

  return nav_cmd;
}

std::shared_ptr<Nav2dFlow::NavInfoForDisplay>
Nav2dFlow::getCurNavInfoForDisplay() {
  std::unique_lock<std::mutex> lock(mutex_nav_);
  if (!T_G_O_) {
    return nullptr;
  }

  std::shared_ptr<NavInfoForDisplay> info_ptr(new NavInfoForDisplay());
  NavInfoForDisplay& info = *info_ptr;

  // Eigen::Vector3d T_O_G_2d = transformPoseFrom3dTo2d(*T_G_O_);
  StampedGlobalPose::Pose3d T_O_G = T_G_O_->inverse();
  info.T_O_G = std::make_unique<StampedGlobalPose::Pose3d>(T_O_G);

  info.traj = traj_2d_;
  for (size_t i=0; i<info.traj.size(); i++) {
    info.traj[i].z() = 0;  // set z=0 for all traj points.

    // convert to odom frame
    info.traj[i] = T_O_G * info.traj[i];
    // info.traj[i].z() = 0;
  }

  for (size_t i=0; i<waypoints_.size(); i++) {
    info.nav_waypoints.push_back(T_O_G * transformPoseFrom2dTo3d_FrontX(traj_2d_[waypoints_[i]]));
  }

  info.waypoint_names = waypoint_names_;
  info.current_path = current_path_;
  for (size_t i=0; i<info.current_path.size(); i++) {
    info.current_path[i].z() = 0;

    // convert to odom frame
    info.current_path[i] = T_O_G * info.current_path[i];
    // info.current_path[i].z() = 0;
  }
  // info.current_pathpoint = current_path_[current_pathpoint_idx_];
  // info.current_pathpoint.z() = 0;

  info.state = stateStr(state_);
  return info_ptr;
}

void Nav2dFlow::beginSaveNavCmds(const std::string& filename) {
  // std::shared_ptr<std::ofstream>;
  nav_cmd_file_ = std::make_shared<std::ofstream>(filename);
}

void Nav2dFlow::beginPlayNavCmds(const std::string& filename) {
  std::vector<Nav2dCmd> nav_cmds_to_play;
  std::ifstream nav_cmd_file(filename);
  if (!nav_cmd_file.is_open()) {
    LOG(ERROR) << "Could not open nav command file " << filename.c_str();
    return;
  }
  std::string line;
  while (std::getline(nav_cmd_file, line)) {
    Nav2dCmd cmd = Nav2dCmd::fromStr(line);
    nav_cmds_to_play.push_back(cmd);
  }
  nav_cmd_file.close();

  std::swap(nav_cmds_to_play, nav_cmds_to_play_);
  nav_cmd_play_idx_ = 0;
}

void Nav2dFlow::saveNavCmd(const Nav2dCmd& cmd) {
  if (nav_cmd_file_ && nav_cmd_file_->is_open()) {
    *nav_cmd_file_ << cmd.toStr() << std::endl;
    new_nav_cmds_since_last_flush_ ++;
    if (new_nav_cmds_since_last_flush_ > 20) {
      nav_cmd_file_->flush();
      new_nav_cmds_since_last_flush_ = 0;
    }
  }
}

#ifdef EANBLE_ROS_NAV_INTERFACE

namespace {
inline ros::Time createRosTimestamp(int64_t timestamp_nanoseconds) {
  static constexpr uint32_t kNanosecondsPerSecond = 1e9;
  const uint64_t timestamp_u64 = static_cast<uint64_t>(timestamp_nanoseconds);
  const uint32_t ros_timestamp_sec = timestamp_u64 / kNanosecondsPerSecond;
  const uint32_t ros_timestamp_nsec =
      timestamp_u64 - (ros_timestamp_sec * kNanosecondsPerSecond);
  return ros::Time(ros_timestamp_sec, ros_timestamp_nsec);
}
}


void Nav2dFlow::initRosInterface() {
  // std::function<bool(RosNavRequest::Request&, RosNavRequest::Response&)>
  //     srv_callback =
  //         std::bind(&Nav2dFlow::dealWithRosRequest, this, std::placeholders::_1, std::placeholders::_2);
	// ros_nav_srv_ = node_handle_.advertiseService("NavRequest", srv_callback);
	ros_nav_srv_ = node_handle_.advertiseService("NavRequest", &Nav2dFlow::dealWithRosRequest, this);

  ros_pub_nav_cmd_ =
      node_handle_.advertise<RosNav2dCmd>("nav2d_cmd", 1);

  ros_pub_nav_cmd_viz_ = node_handle_.advertise<geometry_msgs::PoseArray>(
      "nav2d_cmd_viz", 1);

  // node_handle_.subscribe("nav2d_waypoint", &Nav2dFlow::nav2dTargetCallback, this);
  // boost::function<void(const sensor_msgs::ImuConstPtr&)> imu_callback =
  //     boost::bind(&DataSourceRostopic::imuMeasurementCallback, this, _1);

  //     ros::Subscriber sub_local_object_pose_
  sub_local_object_pose_ = node_handle_.subscribe("local_object_pose", 1, &Nav2dFlow::localObjectPoseCallback, this);
}

void Nav2dFlow::localObjectPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  LOG(INFO) << "Nav2dFlow: Received local object pose: time " << msg->header.stamp.toSec() << "; position "
            << msg->pose.position.x << ", " << msg->pose.position.y << ", " << msg->pose.position.z;

  int64_t timestamp_ns = msg->header.stamp.toSec() * 1e9;
  Eigen::Quaterniond local_object_q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
  Eigen::Vector3d local_object_t(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  StampedGlobalPose::Pose3d local_object_pose(local_object_q.toRotationMatrix(), local_object_t);
  local_object_pose = object_cam_extrinsics_ * local_object_pose;

  auto odom_pose = getOdomPoseAtTime(timestamp_ns);
  if (!odom_pose) {
    return;
  }

  auto object_in_odom_frame = std::make_unique<StampedGlobalPose::Pose3d>(*odom_pose * local_object_pose);

  std::unique_lock<std::mutex> lock(mutex_object_nav_);
  object_in_odom_frame_ = std::move(object_in_odom_frame);
}

std::unique_ptr<StampedGlobalPose::Pose3d> Nav2dFlow::getOdomPoseAtTime(int64_t timestamp_ns) {
  // TODO: Store a buffer of poses and interpolate between them.
  std::unique_lock<std::mutex> lock(mutex_nav_);
  if (last_odom_pose_) {
    return nullptr;
  } else {
    return std::make_unique<StampedGlobalPose::Pose3d>(*last_odom_pose_);
  }
}

bool Nav2dFlow::dealWithRosRequest(RosNavRequest::Request &request, RosNavRequest::Response &response) {
  std::string cmd = request.cmd;
  if (cmd == "startPathRecording") {
    response.ack = startPathRecording();
  } else if (cmd == "addWaypoint") {
    std::string waypoint_name = request.arg;
    response.ack = addWaypoint(waypoint_name);
  } else if (cmd == "finishPathRecording") {
    response.ack = finishPathRecording();
  } else if (cmd == "navigateToWaypoint") {
    std::string waypoint_name = request.arg;
    response.ack = navigateToWaypoint(waypoint_name);
  } else if (cmd == "navigateToObject") {
    std::string object_name = request.arg;
    response.ack = navigateToObject(object_name);
  } else if (cmd == "stopNav") {
    response.ack = stopNav();
  } else {
    response.ack = false;
    response.error_info = "Unknown cmd!";
  }
  return true;
}

void Nav2dFlow::convertAndPublishNavCmd(const Nav2dCmd& cmd) {
  if (ros_pub_nav_cmd_.getNumSubscribers() == 0 && ros_pub_nav_cmd_viz_.getNumSubscribers() == 0) {
    return;
  }

  RosNav2dCmd roscmd;
  ros::Time timestamp_ros = createRosTimestamp(cmd.timestamp_ns);
  roscmd.header.seq = ros_nav_cmd_seq_++;
  roscmd.header.stamp = timestamp_ros;  
  roscmd.header.frame_id = "NAV";
  roscmd.cur_pose2d.x = cmd.cur_pose2d.x();
  roscmd.cur_pose2d.y = cmd.cur_pose2d.y();
  roscmd.cur_pose2d.theta = cmd.cur_pose2d.z();
  roscmd.waypoint.x = cmd.waypoint.x();
  roscmd.waypoint.y = cmd.waypoint.y();
  roscmd.waypoint.theta = cmd.waypoint.z();

  for (const Eigen::Vector3d& next_pathpoint : cmd.next_pathpoints) {
    geometry_msgs::Pose2D p;
    p.x = next_pathpoint.x();
    p.y = next_pathpoint.y();
    p.theta = next_pathpoint.z();
    roscmd.next_pathpoints.push_back(p);
  }
  roscmd.is_last_pathpoint = cmd.is_last_pathpoint;
  ros_pub_nav_cmd_.publish(roscmd);

  // publish pose array for rviz visualization
  geometry_msgs::PoseArray nav_cmd_viz_msg;
  nav_cmd_viz_msg.header.frame_id = FLAGS_tf_mission_frame;
  nav_cmd_viz_msg.header.stamp = createRosTimestamp(cmd.timestamp_ns);
  for (const Eigen::Vector3d& next_point_2d : cmd.next_pathpoints) {
    const auto& next_point_3d = transformPoseFrom2dTo3d_FrontX(next_point_2d);
    Eigen::Quaterniond next_point_q(next_point_3d.linear().matrix());
    Eigen::Vector3d next_point_p(next_point_3d.translation());

    geometry_msgs::Pose next_point_msg;
    tf::pointEigenToMsg(next_point_p, next_point_msg.position);
    tf::quaternionEigenToMsg(next_point_q, next_point_msg.orientation);
    nav_cmd_viz_msg.poses.emplace_back(next_point_msg);
  }
  ros_pub_nav_cmd_viz_.publish(nav_cmd_viz_msg);

  // publish current waypoint point to tf
  const auto& waypoint_3d = transformPoseFrom2dTo3d_FrontX(cmd.waypoint);
  Eigen::Quaterniond waypoint_q(waypoint_3d.linear().matrix());
  Eigen::Vector3d waypoint_p(waypoint_3d.translation());
  const aslam::Transformation waypoint_pose(waypoint_p, waypoint_q);
  visualization::publishTF(
      waypoint_pose, FLAGS_tf_mission_frame, "NAV_CUR_TARGET", timestamp_ros);
}

void Nav2dFlow::publishGlobalNavInfoViz() {
  // publish pose of nav-waypoint points to tf

  if (last_vio_estimate_timestamp_ns_ > 0) {
    ros::Time timestamp_ros = createRosTimestamp(last_vio_estimate_timestamp_ns_);
#ifdef PUBLISH_NAV_CMD_IN_ODOM_FRAME
    std::string parent_frame = FLAGS_tf_mission_frame;
    Eigen::Vector3d T_O_G_2d = transformPoseFrom3dTo2d(T_G_O_->inverse(), Eigen::Vector3d::UnitX());
#else
    std::string parent_frame = FLAGS_tf_map_frame;
#endif
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      size_t point_idx = waypoints_[i];
      Eigen::Vector3d waypoint_2d = traj_2d_[point_idx];
#ifdef PUBLISH_NAV_CMD_IN_ODOM_FRAME
      waypoint_2d = composePose2d(T_O_G_2d, waypoint_2d);
#endif
      const auto& waypoint_3d = transformPoseFrom2dTo3d_FrontX(waypoint_2d);
      Eigen::Quaterniond waypoint_q(waypoint_3d.linear().matrix());
      Eigen::Vector3d waypoint_p(waypoint_3d.translation());
      const aslam::Transformation waypoint_pose(waypoint_p, waypoint_q);
      visualization::publishTF(
          waypoint_pose, parent_frame, "NAV_" + waypoint_names_[i], timestamp_ros);
    }
  }
}

#endif


}  // namespace openvinsli
