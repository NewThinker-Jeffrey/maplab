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

#include <gflags/gflags.h>
DEFINE_bool(
    data_player_paused, false,
    "Whether the date player paused.");



#if ENABLE_PANGOLIN

#include "openvinsli/viewer.h"
#include <pangolin/pangolin.h>
#include <pangolin/display/default_font.h>

#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <glog/logging.h>


#include "slam_viz/pangolin_helper.h"
#include "state/Propagator.h"
#include "state/StateHelper.h"
#include "core/SimpleDenseMapping.h"


using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

namespace openvinsli {


const double viewpoint_distance = 5.0;
const double viewpoint_angle = 30.0 / 180.0 * M_PI;


OpenvinsliViewer::OpenvinsliViewer(VioManager* interal_app) : _interal_app(interal_app) {

  nav_ = nullptr;

  vi_player_ = nullptr;

  std::cout << "OpenvinsliViewer::OpenvinsliViewer():  Use Pangolin!" << std::endl;

  // std::cout << "OpenvinsliViewer::OpenvinsliViewer():  Loading Chinese font ..." << std::endl;
  // slam_viz::pangolin_helper::loadChineseFont();
}

void OpenvinsliViewer::init() {

  pangolin::CreateWindowAndBind("SLAM Demo",1280,720);

  double menu_width = 0.1;
// panel
#if 1
  pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(200));
  // pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,menu_width);
  // pangolin::Var<bool> boolVar1("menu.boolVar1", false, true);
  // pangolin::Var<int> intVar1("menu.intVar1", 10, 0, 20);
  // pangolin::Var<double> doubleVar1("menu.doubleVar1",10, 0, 20);
  pangolin::Var<std::function<void()>> pause_var("menu.PauseData", [this](){
    if (vi_player_) {
      vi_player_->pause();
      FLAGS_data_player_paused = true;
    }
  });
  pangolin::Var<std::function<void()>> frame_by_frame_var("menu.FrameByFrame", [this](){
    if (vi_player_) {
      vi_player_->pauseAtEveryImage();
      FLAGS_data_player_paused = true;
    }
  });
  pangolin::Var<std::function<void()>> resume_var("menu.ResumeData", [this](){
    if (vi_player_) {
      vi_player_->resume();
      FLAGS_data_player_paused = false;
    }
  });
  keep_old_map_points_var_ = std::make_shared<pangolin::Var<bool>>("menu.KeepOldMapPoints", false, true);
  infinite_traj_length_var_ = std::make_shared<pangolin::Var<bool>>("menu.InfiniteTrajLength", false, true);
  pangolin::Var<std::function<void()>> clear_traj_var("menu.ClearTraj", [this](){
    _traj.clear();
  });
  pangolin::Var<std::function<void()>> clear_points_var("menu.ClearPoints", [this](){
    _map_points.clear();
  });

  show_nav_var_ = std::make_shared<pangolin::Var<bool>>("menu.ShowNav", true, true);
  pangolin::Var<std::function<void()>> start_path_record_var("menu.StartPathRecrod", [this](){
    if (nav_) {
      nav_->startPathRecording();
    }
  });
  target_point_name_var_ = std::make_shared<pangolin::Var<std::string>>("menu.TargetName", "");
  pangolin::Var<std::function<void()>> add_nav_target_var("menu.AddNavTarget", [this](){
    if (nav_) {
      nav_->addTargetPoint(*target_point_name_var_);
      (*target_point_name_var_) = "";
    }
  });
  pangolin::Var<std::function<void()>> finish_path_record_var("menu.FinishPathRecrod", [this](){
    if (nav_) {
      nav_->finishPathRecording();
    }
  });
  pangolin::Var<std::function<void()>> navigate_to_var("menu.NavigateTo", [this](){
    if (nav_) {
      nav_->navigateTo(*target_point_name_var_);
      (*target_point_name_var_) = "";
    }
  });
  pangolin::Var<std::function<void()>> cancel_nav_var("menu.CancelNav", [this](){
    if (nav_) {
      nav_->stopNav();
    }
  });
  show_rgbd_map_var_ = std::make_shared<pangolin::Var<bool>>("menu.ShowRgbdMap", false, true);
  pangolin::Var<std::function<void()>> begin_rgbd_mapping_var("menu.BeginRgbdMapping", [this](){
    std::cout << "widget thread id (menu.BeginRgbdMapping): " << std::this_thread::get_id() << std::endl;
    if (_interal_app) {
      _interal_app->begin_rgbd_mapping();
    }
  });
  pangolin::Var<std::function<void()>> stop_rgbd_mapping_var("menu.StopRgbdMapping", [this](){
    std::cout << "widget thread id (menu.StopRgbdMapping): " << std::this_thread::get_id() << std::endl;
    if (_interal_app) {
      _interal_app->stop_rgbd_mapping();
    }
  });
  pangolin::Var<std::function<void()>> clear_rgbd_map_var("menu.ClearRgbdMap", [this](){
    std::cout << "widget thread id (menu.ClearRgbdMap): " << std::this_thread::get_id() << std::endl;
    if (_interal_app) {
      _interal_app->clear_rgbd_map();
    }
  });
  show_reloc_var_ = std::make_shared<pangolin::Var<bool>>("menu.ShowReloc", false, true);

#endif



  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  // Issue specific OpenGl we might need
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlMatrix proj = pangolin::ProjectionMatrix(640,480,320,320,320,240,0.1,1000);
  // pangolin::OpenGlRenderState s_cam1(proj, pangolin::ModelViewLookAt(1,0.5,-2,0,0,0, pangolin::AxisY) );

std::cout << "OpenvinsliViewer::init(): Before pangolin::OpenGlRenderState" << std::endl;

  // auto modelview1 = pangolin::ModelViewLookAt(0, -viewpoint_distance*sin(viewpoint_angle), viewpoint_distance * cos(viewpoint_angle), 0, 0, 0, pangolin::AxisY);
  auto modelview1 = pangolin::ModelViewLookAt(0, 0, viewpoint_distance, 0, 0, 0, pangolin::AxisY);

std::cout << "Viewer::init(): modelview1 created." << std::endl;


  auto modelview2 = pangolin::ModelViewLookAt(0, -viewpoint_distance * sin(viewpoint_angle), -viewpoint_distance * cos(viewpoint_angle), 0, 0, 0, pangolin::AxisZ);
  // auto modelview2 = pangolin::ModelViewLookAt(0, -viewpoint_distance * sin(viewpoint_angle), -viewpoint_distance * cos(viewpoint_angle), 0, 0, 0,      0, 0, 1);

std::cout << "OpenvinsliViewer::init(): modelview2 created." << std::endl;

  // Keep robot's position and map's orientation  unchanged in the screen
  s_cam1 = std::make_shared<pangolin::OpenGlRenderState>(proj, modelview1);
  assert(s_cam1);

std::cout << "OpenvinsliViewer::init(): s_cam1 created." << std::endl;

  // Keep robot's position and robot's orientation unchanged in the screen
  s_cam2 = std::make_shared<pangolin::OpenGlRenderState>(proj, modelview2);
  assert(s_cam2);

std::cout << "OpenvinsliViewer::init(): s_cam2 created." << std::endl;

std::cout << "OpenvinsliViewer::init(): After pangolin::OpenGlRenderState" << std::endl;

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View& d_cam1 = pangolin::Display("cam1")
    .SetAspect(640.0f/480.0f)
    .SetHandler(new pangolin::Handler3D(*s_cam1))
    .SetBounds(0.5, 1.0, menu_width, 0.5 * (1 + menu_width));

  pangolin::View& d_cam2 = pangolin::Display("cam2")
    .SetAspect(640.0f/480.0f)
    .SetHandler(new pangolin::Handler3D(*s_cam2))
    .SetBounds(0.5, 1.0, 0.5 * (1 + menu_width), 1.0);

  pangolin::View& d_feat_track_img = pangolin::Display("feature_tracking")
    .SetAspect(640.0f/480.0f)
    .SetBounds(0.0, 0.5, 0.5 * (1 + menu_width), 1.0);
}

void OpenvinsliViewer::show(std::shared_ptr<VioManager::Output> output) {
  using namespace slam_viz::pangolin_helper;

  classifyPoints(output);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  cv::Mat img_history = _interal_app->get_historical_viz_image(*output);
  drawCvImageOnView(
      img_history,
      pangolin::Display("feature_tracking"),
      false);
  
  double image_time = output->status.timestamp;
  Eigen::Vector3f new_pos = output->state_clone->_imu->pos().cast<float>();
  _imu_pose = Eigen::Isometry3f(output->state_clone->_imu->Rot().inverse().cast<float>());
  _imu_pose.translation() = new_pos;
  _traj.push_back(new_pos);

  if (! *infinite_traj_length_var_) {
    while (_traj.size() > 100) {
      _traj.pop_front();
    }
  }

  double imu_time;
  if (output->status.initialized) {
    Eigen::Matrix<double, 13, 1> state_plus;
    Eigen::Matrix<double, 12, 12> covariance;
    bool propagate_ok = _interal_app->get_propagator()->fast_state_propagate(
        output->state_clone, -1.0, state_plus, covariance, &imu_time);
    if (propagate_ok) {
      Eigen::Matrix<double, 4, 1> qJPL_from_world_to_imu = state_plus.block(0, 0, 4, 1);
      Eigen::Vector3d pos = state_plus.block(4, 0, 3, 1);
      // Openvins outputs a quaternion in JPL convention and interpret it as the rotation from world to imu,
      // while we want a quaternion in Hamilton convention representing the rotation from imu to world.
      // However, the two quaternions coincide with each other in numbers.
      Eigen::Matrix<double, 4, 1> qHamilton_from_imu_to_world = qJPL_from_world_to_imu;
      auto& q = qHamilton_from_imu_to_world;
      Eigen::Quaternionf quat(q[3], q[0], q[1], q[2]);
      _predicted_imu_pose = Eigen::Isometry3f(quat);
      _predicted_imu_pose.translation() = Eigen::Vector3f(pos[0], pos[1], pos[2]);
    }
  }

  Eigen::Matrix4f fT_MtoG = output->status.T_MtoG.cast<float>();
  Eigen::Matrix3f R_MtoG = fT_MtoG.block<3,3>(0,0);
  Eigen::Vector3f t_MinG = fT_MtoG.block<3,1>(0,3);
  Eigen::Vector3f transformed_new_pos = R_MtoG * new_pos + t_MinG;

  std::shared_ptr<Nav2dFlow::NavInfoForDisplay> nav_info(nullptr);
  if (*show_nav_var_ && nav_) {
    nav_info = nav_->getCurNavInfoForDisplay();
  }


  {
    Eigen::Isometry3f view_anchor_pose = Eigen::Isometry3f::Identity();  
    // view_anchor_pose.translation() = Eigen::Vector3f(transformed_new_pos(0), transformed_new_pos(1), 0);
    view_anchor_pose.translation() = Eigen::Vector3f(transformed_new_pos(0), transformed_new_pos(1), transformed_new_pos(2));
    pangolin::OpenGlMatrix Twa = makeGlMatrix(view_anchor_pose.matrix());

    // s_cam1->SetModelViewMatrix(pangolin::ModelViewLookAt(transformed_new_pos(0), transformed_new_pos(1), viewpoint_distance, transformed_new_pos(0), transformed_new_pos(1), 0, pangolin::AxisY));
    s_cam1->Follow(Twa);
    pangolin::Display("cam1").Activate(*s_cam1);
    drawRobotAndMap(output, *show_rgbd_map_var_, nav_info);    
  }

  {
    Eigen::Isometry3f view_anchor_pose = _imu_pose;
    // Eigen::Isometry3f view_anchor_pose = _imu_pose * Eigen::AngleAxisf(0.5*M_PI, Eigen::Vector3f::UnitX());
    // Z is upward for view_anchor_pose.

    pangolin::OpenGlMatrix Twa = makeGlMatrix(fT_MtoG * view_anchor_pose.matrix());
    s_cam2->Follow(Twa);
    pangolin::Display("cam2").Activate(*s_cam2);
    drawRobotAndMap(output, *show_rgbd_map_var_, nav_info);
  }

  pangolin::Display("cam1").Activate();

  // draw text
  // get latest imu time_str

  auto get_time_str = [](double timestamp) {
    bool use_utc_time = false;  // use local time
    int64_t ts = int64_t(timestamp*1e9);
    ts += 8 * 3600 * 1e9;  // add 8 hours
    std::chrono::nanoseconds time_since_epoch(ts);
    std::chrono::time_point
        <std::chrono::system_clock, std::chrono::nanoseconds>
        time_point(time_since_epoch);
    std::time_t tt = std::chrono::system_clock::to_time_t(time_point);
    struct tm* ptm;
    if (use_utc_time) {
      ptm = gmtime(&tt);
    } else {
      ptm = localtime(&tt);
    }
    struct tm& tm = *ptm;
    int64_t sub_seconds_in_nano = ts % 1000000000;
    char dt[100];
    sprintf(  // NOLINT
        dt, "%04d-%02d-%02d %02d:%02d:%02d", tm.tm_year + 1900, tm.tm_mon + 1,
        tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    // sprintf(dt, "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
    return std::string(dt) + " " + std::to_string(sub_seconds_in_nano/1000000);
  };

  std::string imu_time_str = get_time_str(imu_time);
  std::string image_time_str = get_time_str(image_time);
  char tmp_buf[100];

  sprintf(tmp_buf, "%.3f, %.3f, %.3f", transformed_new_pos(0), transformed_new_pos(1), transformed_new_pos(2));
  std::string slam_pos_str(tmp_buf);
  Eigen::Vector3f predict_pos = _predicted_imu_pose.translation();
  Eigen::Vector3f transformed_predict_pos = R_MtoG * predict_pos + t_MinG;
  sprintf(tmp_buf, "%.3f, %.3f, %.3f", transformed_predict_pos(0), transformed_predict_pos(1), transformed_predict_pos(2));
  std::string predict_pos_str(tmp_buf);
  sprintf(tmp_buf, "%.3f", output->status.distance);
  std::string distance_str(tmp_buf);


  Eigen::MatrixXd cov =
  StateHelper::get_marginal_covariance(output->state_clone, {output->state_clone->_imu->pose()});
  Eigen::MatrixXd q_cov = cov.block(0,0,3,3);
  Eigen::MatrixXd p_cov = cov.block(3,3,3,3);
  double std_q = std::sqrt(q_cov.trace());
  double std_p = std::sqrt(p_cov.trace());
  sprintf(tmp_buf, "%.3f[m], %.3f[rad]", std_p, std_q);
  std::string std_pq_str(tmp_buf);


  drawMultiTextLinesInViewCoord(
      { TextLine("IMU time:   " + imu_time_str)
       ,TextLine("Image time: " + image_time_str)
       ,TextLine("Image delay: " + (imu_time > 0 ? std::to_string(int64_t((imu_time - image_time) * 1e3)) + " ms" : std::string("unknown")))
       ,TextLine("Relocalized: " + std::to_string(output->status.localized))
       ,TextLine("Reloc cnt: " + std::to_string(output->status.accepted_localization_cnt))
       ,TextLine("Stable points:  " + std::to_string(_slam_points.size()))
       ,TextLine("Short-term points: " + std::to_string(_msckf_points.size()))
       ,TextLine("std_pq: " + std_pq_str)
       ,TextLine("slam_pos:     " + slam_pos_str)
       ,TextLine("predict_pos: " + predict_pos_str)
       ,TextLine("Traveled distance: " + distance_str)       
       ,TextLine("NAV State: " + (nav_info ? nav_info->state : std::string("none")))       
      //  ,TextLine("NAV target: " + std::string(*target_point_name_var_))
      },
      10, -120,
      1.0);

  drawTextLineInWindowCoord(
      TextLine("SLAM demo"),
      10, 30,
      2.0);  // enlarge

  pangolin::FinishFrame();
}

void OpenvinsliViewer::classifyPoints(std::shared_ptr<VioManager::Output> output) {
  std::set<size_t> new_ids;
  _slam_points.clear();
  _msckf_points.clear();
  _old_points.clear();
  _active_points.clear();

  CHECK_EQ(output->visualization.feature_ids_SLAM.size(), output->visualization.features_SLAM.size());
  for (size_t i=0; i<output->visualization.feature_ids_SLAM.size(); i++) {
    auto id = output->visualization.feature_ids_SLAM[i];
    const auto& p = output->visualization.features_SLAM[i];
    if (*keep_old_map_points_var_) {
      _map_points[id] = p;
    }
    _slam_points.push_back(&p);
    new_ids.insert(id);
  }
  CHECK_EQ(output->visualization.good_feature_ids_MSCKF.size(), output->visualization.good_features_MSCKF.size());
  for (size_t i=0; i<output->visualization.good_feature_ids_MSCKF.size(); i++) {
    auto id = output->visualization.good_feature_ids_MSCKF[i];
    const auto& p = output->visualization.good_features_MSCKF[i];
    if (*keep_old_map_points_var_) {
      _map_points[id] = p;
    }
    _msckf_points.push_back(&p);
    new_ids.insert(id);
  }

  if (*keep_old_map_points_var_) {
    for (const auto& item : _map_points) {
      if (new_ids.count(item.first) == 0) {
        const Eigen::Vector3d& p = item.second;
        _old_points.push_back(&p);
      }
    }
  } else {
    _map_points.clear();
  }

  for (const auto& item : output->visualization.active_tracks_posinG) {
    if (new_ids.count(item.first) == 0) {
      const Eigen::Vector3d& p = item.second;
      _active_points.push_back(&p);
    }
  }
}

void OpenvinsliViewer::drawRobotAndMap(std::shared_ptr<VioManager::Output> output,
                                       bool draw_rgbd,
                                       std::shared_ptr<Nav2dFlow::NavInfoForDisplay> nav_info) {
  using namespace slam_viz::pangolin_helper;
  bool need_draw_rgbd = draw_rgbd && output->visualization.rgbd_dense_map_builder;

  Eigen::Vector3f new_pos = _imu_pose.translation();

  Eigen::Matrix4f fT_MtoG = output->status.T_MtoG.cast<float>();
  Eigen::Matrix3f R_MtoG = fT_MtoG.block<3,3>(0,0);
  Eigen::Vector3f t_MinG = fT_MtoG.block<3,1>(0,3);
  Eigen::Vector3f transformed_new_pos = R_MtoG * new_pos + t_MinG;

  // draw grid
  if (!need_draw_rgbd) {
    drawGrids2D(
        transformed_new_pos(0), transformed_new_pos(1),
        100, 100,
        Color(255, 255, 255, 40), 1.0f);
    drawFrame(2.0, 10.0, 80);
  }

  // draw blue line connecting the origin point (of the global map) and current pos.
  glLineWidth(1.0);
  glColor4ub(0, 0, 255, 120);
  glBegin(GL_LINES);
  glVertex3f(0,0,0);
  glVertex3f(transformed_new_pos(0), transformed_new_pos(1), transformed_new_pos(2));
  glEnd();

  // then draw everything in mission frame
  glPushMatrix();
  glMultMatrixf(fT_MtoG.data());

  // draw yellow line connecting the origin point (of the mission, not of the global map) and current pos.
  glLineWidth(1.0);
  glColor4ub(255, 255, 0, 80);
  glBegin(GL_LINES);
  glVertex3f(0,0,0);
  glVertex3f(new_pos(0), new_pos(1), new_pos(2));
  glEnd();


  if (!need_draw_rgbd) {
    drawMultiTextLines(
        {TextLine("起点", false, getChineseFont()),
        TextLine("0 point", false, getChineseFont())
        },
        Eigen::Vector3f(0, 0, 0),
        Eigen::Matrix3f::Identity(),
        1.0 / 36.0);
  }

  // Return if we have not inited
  if (!output->status.initialized) {
    pangolin::FinishFrame();
    return;
  }

  // draw rgbd map
  using Voxel = dense_mapping::Voxel;
  using VoxColor = ov_msckf::dense_mapping::VoxColor;
  using VoxPosition = ov_msckf::dense_mapping::VoxPosition;
  if (need_draw_rgbd) {
    // auto map_ptr = output->visualization.rgbd_dense_map_builder->get_output_map();
    auto map_ptr = output->visualization.rgbd_dense_map_builder->get_display_map(_interal_app->get_params().rgbd_mapping_max_dispaly_voxels);
    if (map_ptr) {
      size_t n_reserved = map_ptr->reservedVoxelSize();
      glPointSize(2.0);
      glBegin(GL_POINTS);
      size_t n_valid = 0;
      
      map_ptr->foreachVoxel([&](const VoxPosition& p, const VoxColor& c){
        n_valid ++;
        glColor4ub(c[0], c[1], c[2], 255);
        Eigen::Vector3f pf(p.x(), p.y(), p.z());
        pf *= map_ptr->resolution;
        glVertex3f(pf.x(), pf.y(), pf.z());
      });

      std::cout << "DEBUG_RGBD_MAP: map_ptr is not null, n_reserved is " << n_reserved << ", n_valid is " << n_valid << ", resolution is " << map_ptr->resolution << std::endl;
      glEnd();
    }
  } else {
    // draw points
    // drawPointCloud2(_old_points, Color(127,127,127,80), 3.0);
    drawPointCloud2(_old_points, Color(200,200,200,80), 3.0);
    drawPointCloud2(_active_points, Color(255,255,255,150), 3.0);
    drawPointCloud2(_slam_points, Color(255,0,0,255), 5.0);
    drawPointCloud2(_msckf_points, Color(0,0,255,255), 5.0);
  }

  // draw traj
  drawPointTrajectory(_traj, Color(255,0,0,127), 4.0);

  // draw IMU / Camera
  multMatrixfAndDraw(_imu_pose.matrix(), [&](){
    // drawFrame(0.25);

    // drawCamera(0.5, Color(0,0,255,255), 1.0f);

    drawVehicle(
        0.4, Eigen::Vector3f(0, 0, 1), Eigen::Vector3f(0, -1, 0),
        Color(255,255,0,80), 4.0f);

    // drawTextLineFacingScreen(
    //     TextLine("Hello Robot", true),
    //     Eigen::Vector3f(1.0, 0.0, 0.0),
    //     2.0);  // enlarge    

    drawMultiTextLines(
        {TextLine("Hello Robot", true, getChineseFont()),
         TextLine("你好机器人", true, getChineseFont())},
        Eigen::Vector3f(-1.0, 0.0, -1.0),
        Eigen::AngleAxisf(0.5*M_PI, Eigen::Vector3f::UnitX()).toRotationMatrix(),
        // 1.0 / 36.0);
        1.0 / 54.0);
        // 1.0 / 72.0);
  });


  const bool draw_imu_predict = true;
  if (draw_imu_predict) {
    multMatrixfAndDraw(_predicted_imu_pose.matrix(), [&](){
      drawVehicle(
          0.4 * 1.3, Eigen::Vector3f(0, 0, 1), Eigen::Vector3f(0, -1, 0),
          Color(255,0,255,80), 4.0f);
    });    
  }

  glPopMatrix();

  // draw last reloc
  if (*show_reloc_var_ && output->status.localized) {
    Eigen::Matrix4f last_accepted_reloc_TItoG = output->status.last_accepted_reloc_TItoG.cast<float>();
    multMatrixfAndDraw(last_accepted_reloc_TItoG.matrix(), [&](){
      drawVehicle(
          0.4, Eigen::Vector3f(0, 0, 1), Eigen::Vector3f(0, -1, 0),
          // Color(0,0,255,80), 4.0f);
          Color(255,255,0,255), 4.0f);
      // drawMultiTextLines(
      //     {TextLine("last_reloc", true, getChineseFont())},
      //     Eigen::Vector3f(-1.0, 0.0, -1.0),
      //     Eigen::AngleAxisf(0.5*M_PI, Eigen::Vector3f::UnitX()).toRotationMatrix(),
      //     // 1.0 / 36.0);
      //     1.0 / 54.0);
      //     // 1.0 / 72.0);
    });
  }
  
  // draw NAV
  if (nav_info) {
    // // draw whole traj
    // drawPointTrajectory(nav_info->traj, Color(255, 255, 0, 90), 1.0);

    // draw target points
    for (size_t i=0; i<nav_info->nav_targets.size(); i++) {
      Eigen::Isometry3f nav_target = nav_info->nav_targets[i].cast<float>();
      multMatrixfAndDraw(nav_target.matrix(), [&](){
        drawVehicle(
            0.4, Eigen::Vector3f(0, 0, 1), Eigen::Vector3f(0, -1, 0),
            Color(255,0,128,80), 4.0f);
        drawMultiTextLines(
            {TextLine(nav_info->target_names[i], true, getChineseFont())},
            Eigen::Vector3f(0.0, 0.0, 0.05),
            Eigen::AngleAxisf(0.5*M_PI, Eigen::Vector3f::UnitX()).toRotationMatrix(),
            // 1.0 / 36.0);
            1.0 / 36.0);
            // 1.0 / 72.0);
      });
    }

    if (nav_info->state == "NAVIGATING") {
      // draw current path
      // drawPointTrajectory(nav_info->current_path, Color(255, 255, 0, 30), 8.0);

      // draw nav-cmd
      if (cur_nav_cmd_) {
        std::vector<Eigen::Vector3d> plan;
        Eigen::Vector3d cur_pos = cur_nav_cmd_->cur_pose2d;
        cur_pos.z() = 0;
        plan.push_back(cur_pos);
        for (size_t i=0; i<cur_nav_cmd_->next_pathpoints.size(); i++) {
          Eigen::Vector3d next_pos = cur_nav_cmd_->next_pathpoints[i];
          next_pos.z() = 0;
          plan.push_back(next_pos);
        }
        drawPointTrajectory(plan, Color(255, 255, 0, 120), 8.0);
      }
    }
  }
}


}  // namespace openvinsli



#endif