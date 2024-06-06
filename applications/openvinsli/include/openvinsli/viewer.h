/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2022 Patrick Geneva
 * Copyright (C) 2018-2022 Guoquan Huang
 * Copyright (C) 2018-2022 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_MSCKF_VIEWER_H
#define OV_MSCKF_VIEWER_H

#include "core/VioManager.h"
#include <mutex>

#include "openvinsli/mini-nav2d-flow.h"


#if ENABLE_PANGOLIN
#include <pangolin/pangolin.h>
#include "slam_viz/pangolin_helper_types.h"

#include "hear_slam/common/datasource/vi_player.h"

namespace openvinsli {

class OpenvinsliViewer {
public:
  OpenvinsliViewer(ov_msckf::VioManager* interal_app);
  void init();
  void show(std::shared_ptr<ov_msckf::VioManager::Output> task);

  void setNav(Nav2dFlow* nav) {
    std::unique_lock<std::mutex> lock(mutex_);
    nav_ = nav;
  }

  void setNavCmd(const openvinsli::Nav2dCmd::ConstPtr& nav_cmd) {
    std::unique_lock<std::mutex> lock(mutex_);
    cur_nav_cmd_ = nav_cmd;
  }

  void setViPlayer(hear_slam::ViPlayer* vi_player) {
    vi_player_ = vi_player;
  }

private:

  void classifyPoints(std::shared_ptr<ov_msckf::VioManager::Output> task);

  void drawRobotAndMap(std::shared_ptr<ov_msckf::VioManager::Output> task,
                       bool draw_rgbd=false,
                       std::shared_ptr<Nav2dFlow::NavInfoForDisplay> nav_info=nullptr);

  /// Core application of the filter system
  ov_msckf::VioManager* _interal_app;

  std::shared_ptr<pangolin::OpenGlRenderState> s_cam1, s_cam2;
  std::deque <Eigen::Vector3f> _traj;
  std::map<size_t, Eigen::Vector3d> _map_points;
  slam_viz::pangolin_helper::PointdPtrSet _slam_points, _msckf_points, _old_points, _active_points;
  Eigen::Isometry3f _imu_pose, _predicted_imu_pose;

  std::mutex mutex_;
  Nav2dFlow* nav_;
  Nav2dCmd::ConstPtr cur_nav_cmd_;

  hear_slam::ViPlayer* vi_player_;

  // widgets
  std::shared_ptr<pangolin::Var<std::string>> target_point_name_var_;
  std::shared_ptr<pangolin::Var<bool>> show_nav_var_;
  std::shared_ptr<pangolin::Var<bool>> show_rgbd_map_var_;
  std::shared_ptr<pangolin::Var<bool>> show_reloc_var_;
  std::shared_ptr<pangolin::Var<bool>> keep_old_map_points_var_;
  std::shared_ptr<pangolin::Var<bool>> infinite_traj_length_var_;
};


} // namespace openvinsli

#else

namespace openvinsli {

class OpenvinsliViewer {
public:
  OpenvinsliViewer(ov_msckf::VioManager* interal_app) {
    std::cout << "OpenvinsliViewer::OpenvinsliViewer():  No Pangolin!" << std::endl;
  }
  void init() {}
  void show(std::shared_ptr<ov_msckf::VioManager::Output> task) {}
  void setNav(Nav2dFlow* nav) {}
  void setNavCmd(const openvinsli::Nav2dCmd::ConstPtr& nav_cmd) {}
  void setViPlayer(hear_slam::ViPlayer* vi_player) {}
};

} // namespace openvinsli

#endif


#endif // OV_MSCKF_VIEWER_H
