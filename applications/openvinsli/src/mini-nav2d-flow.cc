#include "openvinsli/mini-nav2d-flow.h"

#include <memory>
#include <random>
#include <string>
#include <vector>

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

#include "hear_slam/common/yaml_helper.h"

DEFINE_string(
    openvinsli_nav_savefile, "nav.yaml",
    "Path to save the recorded traj and target points. (nav.yaml)");

namespace openvinsli {

namespace {

double getYawFromQuaternion(const Eigen::Quaterniond& q) {
  // For our sensor, Z is forward.
  Eigen::Vector3d front = Eigen::Vector3d::UnitZ();
  front = q * front;
  // todo:make sure front.y() and front.x() fixed.
  return  atan2(front.y(), front.x());
}

Eigen::Vector3d transformPoseFrom3dTo2d(const Eigen::Isometry3d& pose_3d) {
  Eigen::Vector3d pose_2d = pose_3d.translation();
  Eigen::Quaterniond q(pose_3d.linear());
  q.normalize();
  pose_2d[2] = getYawFromQuaternion(q);
  return pose_2d;
}

Eigen::Vector3d transformPoseFrom3dTo2d(const aslam::Transformation& pose_3d) {
  Eigen::Vector3d pose_2d = pose_3d.getPosition();
  pose_2d[2] = getYawFromQuaternion(pose_3d.getRotation().toImplementation());
  return pose_2d;
}

Eigen::Vector3d getPose2dFromVioEstimate(OpenvinsEstimate::ConstPtr vio_estimate) {
  CHECK(vio_estimate->has_T_G_M);
  aslam::Transformation T_G_I = vio_estimate->T_G_M * vio_estimate->vinode.get_T_M_I();
  return transformPoseFrom3dTo2d(T_G_I);
}

Eigen::Isometry3d transformPoseFrom2dTo3d(const Eigen::Vector3d& pose_2d) {
  Eigen::Isometry3d pose_3d = Eigen::Isometry3d::Identity();

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

}  // namespace


Nav2dFlow::Nav2dFlow() : state_(NavState::IDLE) {

#ifdef EANBLE_ROS_NAV_INTERFACE
  initRosInterface();
#endif

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

  //// subscribe odometry
  flow->registerSubscriber<message_flow_topics::OPENVINS_ESTIMATES>(
      kSubscriberNodeName, message_flow::DeliveryOptions(),
      [this](const OpenvinsEstimate::ConstPtr& estimate) {
        CHECK(estimate);
        // process the estimate in a special thread?
        std::unique_lock<std::mutex> lock(mutex_queue_);
        vio_estimates_.push_back(estimate);
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
    target_points_.clear();
    target_point_names_.clear();
    LOG(INFO) << "Nav2dFlow: startPathRecording() OK!";
    return true;
  }

  LOG(WARNING) << "Nav2dFlow: startPathRecording() failed since we're in IDLE state!";
  return false;
}

bool Nav2dFlow::finishPathRecording(const std::string& tmp_savefile) {
  std::string savefile = tmp_savefile;
  if (savefile.empty()) {
    savefile = FLAGS_openvinsli_nav_savefile;
  }
  std::unique_lock<std::mutex> lock(mutex_nav_);
  if (state_ == NavState::PATH_RECORDING) {
    serialize(savefile);
    // traj_2d_.clear();
    // target_points_.clear();
    // target_point_names_.clear();
    LOG(INFO) << "Nav2dFlow: finishPathRecording() OK!";
    state_ = NavState::IDLE;
    return true;
  }

  LOG(WARNING) << "Nav2dFlow: finishPathRecording() failed since we're in a wrong state!";
  return false;
}

bool Nav2dFlow::addTargetPoint(const std::string& target_name) {
  std::unique_lock<std::mutex> lock(mutex_nav_);
  if (state_ == NavState::PATH_RECORDING) {
    int new_target_idx = target_points_.size();
    target_points_.push_back(traj_2d_.size() - 1);
    std::string name = target_name;
    if (name.empty()) {
      name = "target_" + std::to_string(new_target_idx);
    }
    target_point_names_.push_back(name);
    LOG(INFO) << "Nav2dFlow: addTargetPoint() OK!";
    return true;
  }

  LOG(WARNING) << "Nav2dFlow: addTargetPoint() failed since we're in a wrong state!";
  return false;
}

bool Nav2dFlow::navigateTo(size_t target_idx) {
  std::unique_lock<std::mutex> lock(mutex_nav_);
  if (state_ == NavState::IDLE) {
    state_ = NavState::PATH_PLANNING;
    current_target_idx_ = target_idx;
    LOG(INFO) << "Nav2dFlow: navigateTo() OK!";
    return true;
  }

  LOG(WARNING) << "Nav2dFlow: navigateTo() failed since we're in IDLE state!";
  return false;
}

bool Nav2dFlow::navigateTo(const std::string& target_name) {
  for (size_t i=0; i<target_point_names_.size(); i++) {
    if (target_point_names_[i] == target_name) {
      return navigateTo(i);
    }
  }

  LOG(WARNING) << "Nav2dFlow: navigateTo() failed since we can't find a target named \"" << target_name << "\"";
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
  hear_slam::YamlObjPtr p_obj = hear_slam::loadYaml(nav_config_file);
  hear_slam::YamlObj& obj = *p_obj;
  auto target_points = obj["target_points"];
  target_points_.resize(target_points.size());
  target_point_names_.resize(target_points.size());
  for (size_t i=0; i < target_points.size(); i++) {
    auto target_point = target_points[i];
    target_point_names_[i] = target_point["name"].as<std::string>();
    target_points_[i] = target_point["traj_pidx"].as<int>();
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
  for (size_t i=0; i<target_points_.size(); i++) {
    hear_slam::YamlObj cur_target;
    cur_target["name"] = target_point_names_[i];
    cur_target["traj_pidx"] = target_points_[i];
    obj["target_points"].push_back(cur_target);
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
    OpenvinsEstimate::ConstPtr vio_estimate = nullptr;
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

void Nav2dFlow::processInput(const OpenvinsEstimate::ConstPtr& vio_estimate) {
  // Deal with vio_estimate.
  std::unique_lock<std::mutex> lock(mutex_nav_);

  if (!nav_cmds_to_play_.empty()) {
    // If we're in offline mode, we just play back the recorded nav_cmds.

    while (nav_cmd_play_idx_ < nav_cmds_to_play_.size()) {
      const auto& cur_nav_cmd = nav_cmds_to_play_[nav_cmd_play_idx_];
      const int64_t time_ns_precision = 10;
      if (cur_nav_cmd.timestamp_ns <= vio_estimate->timestamp_ns + time_ns_precision) {
        Nav2dCmd::Ptr nav_cmd = aligned_shared<Nav2dCmd>(cur_nav_cmd);
        state_ = NavState::NAVIGATING;

#ifdef EANBLE_ROS_NAV_INTERFACE
        convertAndPublishNavCmd(*nav_cmd);
#endif
        publish_nav_(nav_cmd);
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
  
  if (!vio_estimate->has_T_G_M) {
    if (state_ != NavState::IDLE) {
      LOG(WARNING) << "Nav2dFlow:  The robot has not been localized yet "
                      "or the localization has been lost! Change state from "
                    << stateStr(state_) << " to IDLE";
      state_ = NavState::IDLE;
    }
    return;
  }

  current_pose_2d_ = getPose2dFromVioEstimate(vio_estimate);

  if (NavState::PATH_RECORDING == state_) {
    tryAddingTrajPoint(current_pose_2d_);
  } else if (NavState::NAVIGATING == state_) {

    Nav2dCmd::Ptr nav_cmd = aligned_shared<Nav2dCmd>();
    nav_cmd->timestamp_ns = vio_estimate->timestamp_ns;
    nav_cmd->cur_pose2d = current_pose_2d_;

    // Check whether we have reached the target and if not, check whether we need to change current_pathpoint_idx_
    Eigen::Vector3d cur_pathp = current_path_[current_pathpoint_idx_];
    Eigen::Vector3d diff = cur_pathp - current_pose_2d_;
    diff.z() = 0;
    double dist = diff.norm();
    if (current_pathpoint_idx_ + 1 < current_path_.size()) {
      if (dist < 0.25) {
        current_pathpoint_idx_ += 1;
      }
      
      // nav_cmd->cur_pathpoint = current_path_[current_pathpoint_idx_];
      for (size_t i=0; i < 10 && current_pathpoint_idx_+i < current_path_.size(); i++) {
        nav_cmd->next_pathpoints.push_back(current_path_[current_pathpoint_idx_+i]);
      }      
      nav_cmd->is_last_pathpoint = !(current_pathpoint_idx_ + 1 < current_path_.size());
    } else {
      // For the last point, also check the orientation
      double theta_diff = cur_pathp.z() - current_pose_2d_.z();
      if (theta_diff > M_PI) {
        theta_diff -= 2.0 * M_PI;
      } else if (theta_diff < -M_PI) {
        theta_diff += 2.0 * M_PI;
      }

      // For now we skip the following check since it will be done by the downstream controller.

      // // Check whether the robot has reached the target point.
      // if (dist < 0.15 && theta_diff < 0.05) {
      //   LOG(WARNING) << "Nav2dFlow:  Finished navigation.";
      //   state_ = NavState::IDLE;
      //   return;
      // }


      // nav_cmd->cur_pathpoint = current_path_[current_pathpoint_idx_];
      nav_cmd->next_pathpoints.push_back(current_path_[current_pathpoint_idx_]);
      // nav_cmd->is_last_pathpoint = !(current_pathpoint_idx_ + 1 < current_path_.size());
      nav_cmd->is_last_pathpoint = true;
    }

#ifdef EANBLE_ROS_NAV_INTERFACE
    convertAndPublishNavCmd(*nav_cmd);
#endif
    publish_nav_(nav_cmd);
    saveNavCmd(*nav_cmd);
  } else if (NavState::PATH_PLANNING == state_) {
    // find the nearest traj point
    size_t nearest_traj_point_idx = findNearstTrajPoint(current_pose_2d_);
    // then find the path
    current_path_ = findPoint2PointTraj(nearest_traj_point_idx, current_target_idx_);
    current_pathpoint_idx_ = 0;
    state_ = NavState::NAVIGATING;
  } else if (NavState::IDLE == state_) {
    
  }
}


void Nav2dFlow::tryAddingTrajPoint(const Eigen::Vector3d& traj_point) {

  size_t n = traj_2d_.size();
  if (n < 2) {
    traj_2d_.push_back(traj_point);
    return;
  }

  if (!target_points_.empty()) {
    // ensure our target points won't be removed
    size_t newest_target_pidx = target_points_.back();
    if (n < newest_target_pidx + 2) {
      traj_2d_.push_back(traj_point);
      return;
    }
  }


  Eigen::Vector3d diff_latest = traj_2d_[n-1] - traj_2d_[n-2];
  diff_latest.z() = 0;
  double dist_latest = diff_latest.norm();
  if (dist_latest < 0.5) {
    // replace the last with our new traj_point
    traj_2d_[n-1] = traj_point;
  } else {
    traj_2d_.push_back(traj_point);
  }
}

size_t Nav2dFlow::findNearstTrajPoint(const Eigen::Vector3d& current_pose_2d) {
  size_t best_i = 0;
  double best_distance = -1;
  for (size_t i=0; i<traj_2d_.size(); i++) {
    Eigen::Vector3d diff = traj_2d_[i] - current_pose_2d;
    diff.z() = 0;
    double distance = diff.norm();
    if (best_distance < -0.001 || distance < best_distance) {
      best_distance = distance;
      best_i = i;
    }
  }
  return best_i;
}

std::vector<Eigen::Vector3d>  Nav2dFlow::findPoint2PointTraj(size_t start_traj_point_idx, size_t target_point_idx) {
  size_t end_traj_point_idx = target_points_[target_point_idx];
  int idx_delta = 1;
  size_t max_n = 0;
  if (end_traj_point_idx < start_traj_point_idx) {
    idx_delta = -1;
    max_n = start_traj_point_idx - end_traj_point_idx;
  } else {
    idx_delta = 1;
    max_n = end_traj_point_idx - start_traj_point_idx;
  }
  std::vector<Eigen::Vector3d> path;
  path.reserve(max_n);

  for (size_t pidx = start_traj_point_idx; pidx != end_traj_point_idx + idx_delta; pidx += idx_delta) {
    Eigen::Vector3d curp = traj_2d_[pidx];
    if (path.size() < 2) {
      path.push_back(curp);
      continue;
    }

    // check loop:
    //   - begin from j=1 so that the first path point is ensured not to be removed).
    //   - 'j+3 < path.size()' prevents loops between neibour points.
    for (size_t j=1; j+3<path.size(); j++) {
      Eigen::Vector3d diff0 = curp - path[j];
      diff0.z() = 0;

      // Eigen::Vector3d diff1 = curp - path[j-1];
      // diff1.z() = 0;
      // if (diff0.norm() < 0.25 && diff1.norm() < 0.6) {
      //   // loop found.
      //   path.erase(path.begin() + j, path.end());
      //   break;
      // }

      double diff0_norm = diff0.norm();
      if (diff0_norm < 0.6) {
        // loop found.
        Eigen::Vector3d diff1 = curp - path[j-1];
        diff1.z() = 0;
        double diff1_norm = diff1.norm();
        if (diff0_norm < 0.3 && diff1_norm < 0.7) {
          path.erase(path.begin() + j, path.end());
        } else {
          path.erase(path.begin() + j + 1, path.end());
        }
        break;
      }
    }

    // add new traj_point
    path.push_back(curp);
  }
  return path;
}


std::shared_ptr<Nav2dFlow::NavInfoForDisplay>
Nav2dFlow::getCurNavInfoForDisplay() {
  std::unique_lock<std::mutex> lock(mutex_nav_);
  std::shared_ptr<NavInfoForDisplay> info_ptr(new NavInfoForDisplay());
  NavInfoForDisplay& info = *info_ptr;

  info.traj = traj_2d_;
  for (size_t i=0; i<info.traj.size(); i++) {
    info.traj[i].z() = 0;  // set z=0 for all traj points.
  }
  for (size_t i=0; i<target_points_.size(); i++) {
    info.nav_targets.push_back(transformPoseFrom2dTo3d(traj_2d_[target_points_[i]]));
  }
  info.target_names = target_point_names_;
  info.current_path = current_path_;
  for (size_t i=0; i<info.current_path.size(); i++) {
    info.current_path[i].z() = 0;
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
}

bool Nav2dFlow::dealWithRosRequest(RosNavRequest::Request &request, RosNavRequest::Response &response) {
  std::string cmd = request.cmd;
  if (cmd == "startPathRecording") {
    response.ack = startPathRecording();
  } else if (cmd == "addTargetPoint") {
    std::string target_name = request.arg;
    response.ack = addTargetPoint(target_name);
  } else if (cmd == "finishPathRecording") {
    response.ack = finishPathRecording();
  } else if (cmd == "navigateTo") {
    std::string target_name = request.arg;
    response.ack = navigateTo(target_name);
  } else if (cmd == "stopNav") {
    response.ack = stopNav();
  } else {
    response.ack = false;
    response.error_info = "Unknown cmd!";
  }
  return true;
}

void Nav2dFlow::convertAndPublishNavCmd(const Nav2dCmd& cmd) {
  if (ros_pub_nav_cmd_.getNumSubscribers() == 0) {
    return;
  }

  RosNav2dCmd roscmd;
  roscmd.header.seq = ros_nav_cmd_seq_++;
  roscmd.header.stamp = createRosTimestamp(cmd.timestamp_ns);  
  roscmd.header.frame_id = "NAV";
  roscmd.cur_pose2d.x = cmd.cur_pose2d.x();
  roscmd.cur_pose2d.y = cmd.cur_pose2d.y();
  roscmd.cur_pose2d.theta = cmd.cur_pose2d.z();

  for (const Eigen::Vector3d& next_pathpoint : cmd.next_pathpoints) {
    geometry_msgs::Pose2D p;
    p.x = next_pathpoint.x();
    p.y = next_pathpoint.y();
    p.theta = next_pathpoint.z();
    roscmd.next_pathpoints.push_back(p);
  }
  roscmd.is_last_pathpoint = cmd.is_last_pathpoint;
  ros_pub_nav_cmd_.publish(roscmd);
}
#endif


}  // namespace openvinsli
