#include "openvinsli/data-publisher-flow.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <maplab-common/conversions.h>
#include <maplab-common/file-logger.h>
#include <maplab_msgs/OdometryWithImuBiases.h>
#include <minkindr_conversions/kindr_msg.h>
#include <nav_msgs/Odometry.h>

#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "openvinsli/ros-helpers.h"
#include "core/SimpleDenseMapping.h"      // ov_msckf

#include "hear_slam/utils/yaml_helper.h"
#include "hear_slam/basic/string_helper.h"
#include "hear_slam/basic/logging.h"

DEFINE_double(
    map_publish_interval_s, 2.0,
    "Interval of publishing the visual-inertial map to ROS [seconds].");

DEFINE_double(
    rgbd_map_publish_interval_s, 1.0,
    "Interval of publishing the rgbd dense local map to ROS [seconds].");

DEFINE_bool(
    publish_only_on_keyframes, false,
    "Publish frames only on keyframes instead of the IMU measurements. This "
    "means a lower frequency.");

DEFINE_bool(
    publish_debug_markers, true,
    "Publish debug sphere markers for T_M_I, T_G_I and localization frames.");

DEFINE_string(
    export_estimated_poses_to_csv, "",
    "If not empty, the map builder will export the estimated poses to a CSV "
    "file.");

DEFINE_bool(
    openvinsli_visualize_map, true,
    "Set to false to disable map visualization. Note: map building needs to be "
    "active for the visualization.");

DEFINE_string(
    share_raw_image0_topic, "/raw_image0",
    "Publish raw image [0] in this topic if it's set to non-empty");

DEFINE_string(
    share_raw_image1_topic, "/raw_image1",
    "Publish raw image [1] in this topic if it's set to non-empty");

DEFINE_double(
    height_map_resolution, 0.01, "height_map_resolution");

DEFINE_double(
    height_map_min_h, -5.0, "height_map_min_h");

DEFINE_double(
    height_map_max_h, 5.0, "height_map_max_h");

DEFINE_double(
    height_map_color_min_h, -1.5, "height_map_color_min_h");

DEFINE_double(
    height_map_color_max_h, 0.0, "height_map_color_max_h");

DEFINE_double(
    height_map_discrepancy_thr, 0.1, "height_discrepancy_thr");

DEFINE_double(
    height_map_upperbound_param_theta_a, 50.0,
    "param (theta_a in degrees) used for filtering overhanging obstacles");

DEFINE_double(
    height_map_upperbound_param_b, 0.1,
    "param (b in metres) used for filtering overhanging obstacles");

DEFINE_double(
    height_map_upperbound_param_c, -0.5,
    "param (c in metres) used for filtering overhanging obstacles");

DEFINE_double(
    height_map_upperbound_param_d, 0.0,
    "param (d in metres) used for filtering overhanging obstacles");

DEFINE_string(
    local_grasp_points_yaml, "",
    "Grasp points yaml file");


DECLARE_bool(openvinsli_run_map_builder);

namespace openvinsli {
namespace {
inline ros::Time createRosTimestamp(int64_t timestamp_nanoseconds) {
  static constexpr uint32_t kNanosecondsPerSecond = 1e9;
  const uint64_t timestamp_u64 = static_cast<uint64_t>(timestamp_nanoseconds);
  const uint32_t ros_timestamp_sec = timestamp_u64 / kNanosecondsPerSecond;
  const uint32_t ros_timestamp_nsec =
      timestamp_u64 - (ros_timestamp_sec * kNanosecondsPerSecond);
  return ros::Time(ros_timestamp_sec, ros_timestamp_nsec);
}

void rgbdLocalMapToPointCloud(
    std::shared_ptr<const ov_msckf::dense_mapping::SimpleDenseMap> rgbd_dense_map,
    sensor_msgs::PointCloud2* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  CHECK_NOTNULL(rgbd_dense_map);

  using Voxel = ov_msckf::dense_mapping::Voxel;
  using VoxColor = ov_msckf::dense_mapping::VoxColor;
  using VoxPosition = ov_msckf::dense_mapping::VoxPosition;

  size_t num_points = 0;
  std::vector<VoxPosition> ipoints;
  std::vector<VoxColor> colors;
  size_t n_reserved = rgbd_dense_map->reservedVoxelSize();
  ipoints.reserve(n_reserved);
  colors.reserve(n_reserved);

  rgbd_dense_map->foreachVoxel([&](const VoxPosition& p, const VoxColor& c){
    ++num_points;
    ipoints.emplace_back(p);
    colors.emplace_back(c);
  });

  point_cloud->height = 3;
  point_cloud->width = num_points;
  point_cloud->fields.resize(4);

  point_cloud->fields[0].name = "x";
  point_cloud->fields[0].offset = 0;
  point_cloud->fields[0].count = 1;
  point_cloud->fields[0].datatype = sensor_msgs::PointField::FLOAT32;

  point_cloud->fields[1].name = "y";
  point_cloud->fields[1].offset = 4;
  point_cloud->fields[1].count = 1;
  point_cloud->fields[1].datatype = sensor_msgs::PointField::FLOAT32;

  point_cloud->fields[2].name = "z";
  point_cloud->fields[2].offset = 8;
  point_cloud->fields[2].count = 1;
  point_cloud->fields[2].datatype = sensor_msgs::PointField::FLOAT32;

  point_cloud->fields[3].name = "rgb";
  point_cloud->fields[3].offset = 12;
  point_cloud->fields[3].count = 1;
  point_cloud->fields[3].datatype = sensor_msgs::PointField::UINT32;

  point_cloud->point_step = 16;
  point_cloud->row_step = point_cloud->point_step * point_cloud->width;
  point_cloud->data.resize(point_cloud->row_step * point_cloud->height);

  // https://pointclouds.org/documentation/singletonpcl_1_1_point_cloud.html#a3ca88d8ebf6f4f35acbc31cdfb38aa94
  // True if no points are invalid (e.g., have NaN or Inf values).
  point_cloud->is_dense = true;
                                    

  int offset = 0;
  for (size_t point_idx = 0u; point_idx < num_points; ++point_idx) {
    const auto& p = ipoints[point_idx];
    const auto& c = colors[point_idx];
    Eigen::Vector3f point(p.x(), p.y(), p.z());
    point *= rgbd_dense_map->resolution;
    memcpy(&point_cloud->data[offset + 0], &point.x(), sizeof(point.x()));
    memcpy(
        &point_cloud->data[offset + sizeof(point.x())], &point.y(),
        sizeof(point.y()));
    memcpy(
        &point_cloud->data[offset + sizeof(point.x()) + sizeof(point.y())],
        &point.z(), sizeof(point.z()));

    const uint32_t rgb = (c[0] << 16) | (c[1] << 8) | c[2];
    memcpy(&point_cloud->data[offset + 12], &rgb, sizeof(uint32_t));
    offset += point_cloud->point_step;
  }
}

void heightMapToPointCloud(
    const cv::Mat& height_map,
    sensor_msgs::PointCloud2* point_cloud,
    double hmap_resolution) {
  CHECK_NOTNULL(point_cloud);
  CHECK_EQ(height_map.type(), CV_16UC1);

  size_t num_points = 0;
  using Vector3i = Eigen::Matrix<int32_t, 3, 1>;
  std::vector<Vector3i> ipoints;
  ipoints.reserve(height_map.rows * height_map.cols);

  for (size_t i = 0; i < height_map.rows; ++i) {
    for (size_t j = 0; j < height_map.cols; ++j) {
      const uint16_t& v = height_map.at<uint16_t>(i, j);
      if (v != 0) {
        ++num_points;
        int32_t z = v-32768;
        ipoints.emplace_back(j-height_map.cols/2, i-height_map.rows/2, z);
        // if (min_z == 0 || z < min_z) {
        //   min_z = z;
        // }
        // if (z > max_z) {
        //   max_z = z;
        // }
      }
    }
  }


  int32_t min_z = FLAGS_height_map_color_min_h / hmap_resolution;
  int32_t max_z = FLAGS_height_map_color_max_h / hmap_resolution;
  int32_t z_range = max_z - min_z;

  auto z_to_color = [&](const int32_t& z) {
    int32_t v = (z - min_z) * 255 / z_range;
    v = std::min(std::max(v, 0), 255);    
    uint8_t r = v;
    uint8_t g = 0;
    uint8_t b = 255 - v;
    uint32_t rgb = (r << 16) | (g << 8) | b;
    return rgb;
  };

  point_cloud->height = 3;
  point_cloud->width = num_points;
  point_cloud->fields.resize(4);

  point_cloud->fields[0].name = "x";
  point_cloud->fields[0].offset = 0;
  point_cloud->fields[0].count = 1;
  point_cloud->fields[0].datatype = sensor_msgs::PointField::FLOAT32;

  point_cloud->fields[1].name = "y";
  point_cloud->fields[1].offset = 4;
  point_cloud->fields[1].count = 1;
  point_cloud->fields[1].datatype = sensor_msgs::PointField::FLOAT32;

  point_cloud->fields[2].name = "z";
  point_cloud->fields[2].offset = 8;
  point_cloud->fields[2].count = 1;
  point_cloud->fields[2].datatype = sensor_msgs::PointField::FLOAT32;

  point_cloud->fields[3].name = "rgb";
  point_cloud->fields[3].offset = 12;
  point_cloud->fields[3].count = 1;
  point_cloud->fields[3].datatype = sensor_msgs::PointField::UINT32;

  point_cloud->point_step = 16;
  point_cloud->row_step = point_cloud->point_step * point_cloud->width;
  point_cloud->data.resize(point_cloud->row_step * point_cloud->height);

  // https://pointclouds.org/documentation/singletonpcl_1_1_point_cloud.html#a3ca88d8ebf6f4f35acbc31cdfb38aa94
  // True if no points are invalid (e.g., have NaN or Inf values).
  point_cloud->is_dense = true;

  int offset = 0;
  for (size_t point_idx = 0u; point_idx < num_points; ++point_idx) {
    const auto& v = ipoints.at(point_idx);
    Eigen::Vector3f point(v.x(), v.y(), v.z());
    point *= hmap_resolution;
    memcpy(&point_cloud->data[offset + 0], &point.x(), sizeof(point.x()));
    memcpy(
        &point_cloud->data[offset + sizeof(point.x())], &point.y(),
        sizeof(point.y()));
    memcpy(
        &point_cloud->data[offset + sizeof(point.x()) + sizeof(point.y())],
        &point.z(), sizeof(point.z()));

    const uint32_t rgb = z_to_color(v.z());
    memcpy(&point_cloud->data[offset + 12], &rgb, sizeof(uint32_t));
    offset += point_cloud->point_step;
  }
}

}  // namespace

DataPublisherFlow::DataPublisherFlow()
    : map_publisher_timeout_(common::TimeoutCounter(
          FLAGS_map_publish_interval_s * kSecondsToNanoSeconds)) {
  visualization::RVizVisualizationSink::init();
  plotter_.reset(new visualization::ViwlsGraphRvizPlotter);
  height_map_work_queue_.reset(new hear_slam::TaskQueue("height_map_q", 1, 1, true));
  if (!FLAGS_local_grasp_points_yaml.empty()) {
    auto local_grasp_points_yamlptr = hear_slam::loadYaml(FLAGS_local_grasp_points_yaml);
    auto& local_grasp_points_yaml = *local_grasp_points_yamlptr;
    const auto& objectes = local_grasp_points_yaml["objects"];
    size_t n_objects = objectes.size();
    for (size_t i = 0; i < n_objects; ++i) {
      const auto& object = objectes[i];
      GraspObjectId object_id = object["id"].as<GraspObjectId>();
      const auto& grasp_points = object["grasp_points"];
      size_t n_grasp_points = grasp_points.size();
      std::vector<Eigen::Vector3d> grasp_points_vec;
      for (size_t j = 0; j < n_grasp_points; ++j) {
        const auto& grasp_point = grasp_points[j];
        Eigen::Vector3d grasp_point_vec(grasp_point[0].as<double>(), grasp_point[1].as<double>(), grasp_point[2].as<double>());
        grasp_points_vec.push_back(grasp_point_vec);
      }
      local_grasp_points_map_[object_id] = grasp_points_vec;
    }
    for (auto& pair : local_grasp_points_map_) {
      using hear_slam::toStr;
      LOGI("Grasp points for object %d: %s", pair.first, toStr(pair.second, [](const Eigen::Vector3d& v) { return v.transpose(); }).c_str());
    }
  }  
}

DataPublisherFlow::~DataPublisherFlow() {
  if (pub_raw_image0_) {
    LOG(INFO) << "Before destroying the publisher pub_raw_image0_";
    pub_raw_image0_.reset();
    LOG(INFO) << "After destroying the publisher pub_raw_image0_";
  }
  if (pub_raw_image1_) {
    LOG(INFO) << "Before destroying the publisher pub_raw_image1_";
    pub_raw_image1_.reset();
    LOG(INFO) << "After destroying the publisher pub_raw_image1_";
  }

  if (pub_local_heightmap_) {
    LOG(INFO) << "Before destroying the publisher pub_local_heightmap_";
    pub_local_heightmap_.reset();
    LOG(INFO) << "After destroying the publisher pub_local_heightmap_";
  }
  if (it_) {
    // Note: Destroying the image transport will causes crash! Don't know how to fix for now.
    // Error info:
    //     terminate called after throwing an instance of 'class_loader::LibraryUnloadException'
    //     what():  Attempt to unload library that class_loader is unaware of.
    LOG(INFO) << "Before destroying the image transport";
    it_.reset();
    LOG(INFO) << "After destroying the image transport";
  }

  height_map_work_queue_.reset();
}

void DataPublisherFlow::registerPublishers() {
  pub_pose_T_M_I_ =
      node_handle_.advertise<geometry_msgs::PoseStamped>(kTopicPoseMission, 1);
  pub_pose_T_G_I_ =
      node_handle_.advertise<geometry_msgs::PoseStamped>(kTopicPoseGlobal, 1);
  pub_transform_T_G_I_ =
      node_handle_.advertise<geometry_msgs::TransformStamped>(
          kTopicTransformGlobal, 1);
  pub_baseframe_T_G_M_ =
      node_handle_.advertise<geometry_msgs::PoseStamped>(kTopicBaseframe, 1);
  pub_velocity_I_ =
      node_handle_.advertise<geometry_msgs::Vector3Stamped>(kTopicVelocity, 1);
  pub_imu_acc_bias_ =
      node_handle_.advertise<geometry_msgs::Vector3Stamped>(kTopicBiasAcc, 1);
  pub_imu_gyro_bias_ =
      node_handle_.advertise<geometry_msgs::Vector3Stamped>(kTopicBiasGyro, 1);
  pub_extrinsics_T_C_Bs_ = node_handle_.advertise<geometry_msgs::PoseArray>(
      kCameraExtrinsicTopic, 1);
  pub_maplab_odom_T_M_I_ =
      node_handle_.advertise<maplab_msgs::OdometryWithImuBiases>(
          kTopicMaplabOdomMsg, 1);
  pub_odom_T_M_I_ =
      node_handle_.advertise<nav_msgs::Odometry>(kTopicOdomMsg, 1);
  pub_grasp_points_computed_from_tag_ = node_handle_.advertise<geometry_msgs::PoseArray>(
      "grasp_points", 1);

  if (!FLAGS_share_raw_image0_topic.empty() || !FLAGS_share_raw_image1_topic.empty()) {
    it_.reset(new image_transport::ImageTransport(node_handle_));
  }
  if (!FLAGS_share_raw_image0_topic.empty()) {
    pub_raw_image0_.reset(new image_transport::Publisher(it_->advertise(FLAGS_share_raw_image0_topic, 1)));
  }
  if (!FLAGS_share_raw_image1_topic.empty()) {
    pub_raw_image1_.reset(new image_transport::Publisher(it_->advertise(FLAGS_share_raw_image1_topic, 1)));
  }
  pub_local_heightmap_.reset(new image_transport::Publisher(it_->advertise("/local_height_map", 1)));
}

void DataPublisherFlow::attachToMessageFlow(message_flow::MessageFlow* flow) {
  CHECK_NOTNULL(flow);
  registerPublishers();
  static constexpr char kSubscriberNodeName[] = "DataPublisherFlow";

  if (FLAGS_openvinsli_run_map_builder && FLAGS_openvinsli_visualize_map) {
    flow->registerSubscriber<message_flow_topics::RAW_VIMAP>(
        kSubscriberNodeName, message_flow::DeliveryOptions(),
        [this](const VIMapWithMutex::ConstPtr& map_with_mutex) {
          if (map_publisher_timeout_.reached()) {
            std::lock_guard<std::mutex> lock(map_with_mutex->mutex);
            visualizeMap(map_with_mutex->vi_map);
            map_publisher_timeout_.reset();
          }
        });
  }

  // Publish localization results.
  flow->registerSubscriber<message_flow_topics::LOCALIZATION_RESULT>(
      kSubscriberNodeName, message_flow::DeliveryOptions(),
      [this](const vio::LocalizationResult::ConstPtr& localization) {
        CHECK(localization != nullptr);
        localizationCallback(localization->T_G_B.getPosition());
      });

  // Publish global pose fusion.
  flow->registerSubscriber<message_flow_topics::GLOBAL_POSE_FUSION>(
      kSubscriberNodeName, message_flow::DeliveryOptions(),
      [this](const StampedGlobalPose::ConstPtr& global_pose) {
        CHECK(global_pose != nullptr);
        if (global_pose->global_pose_valid) {
          // publish T_I_G to tf.
          aslam::Transformation T_G_I(global_pose->global_pose.translation(), Eigen::Quaterniond(global_pose->global_pose.linear().matrix()));
          ros::Time timestamp_ros = createRosTimestamp(global_pose->timestamp_ns);
          LOG(INFO) << "publishing tf T_G_I at time " << timestamp_ros;
          visualization::publishTF(
              T_G_I.inverse(), FLAGS_tf_imu_frame, FLAGS_tf_map_frame, timestamp_ros);
        }
      });

  auto image_to_rosmsg = [](const vio::ImageMeasurement::Ptr& image) {
    cv_bridge::CvImage cv_img;
    cv_img.header.frame_id = "";
    cv_img.header.stamp = createRosTimestamp(image->timestamp);
    cv_img.image = image->image;
    if (image->image.channels() == 1) {
      // mono8 or mono16
      if (image->image.type() == CV_16UC1) {
        cv_img.encoding = "mono16";  // "mono16"  "bgr8"  "rgb8"  "bgra8"  "rgba8"
      } else {
        ASSERT(image->image.type() == CV_16UC1);
        cv_img.encoding = "mono8";  // "mono16"  "bgr8"  "rgb8"  "bgra8"  "rgba8"
      }
    } else {
      cv_img.encoding = "rgb8";  // "mono16"  "bgr8"  "rgb8"  "bgra8"  "rgba8"
    }
    sensor_msgs::ImagePtr msg = cv_img.toImageMsg();
    return msg;
  };

  // Publish raw image.
  flow->registerSubscriber<message_flow_topics::IMAGE_MEASUREMENTS>(
      kSubscriberNodeName, message_flow::DeliveryOptions(),
      [this, image_to_rosmsg](const vio::ImageMeasurement::Ptr& image) {
        CHECK(image);
        if (image->camera_index == 0) {
          if (!FLAGS_share_raw_image0_topic.empty() &&
              pub_raw_image0_->getNumSubscribers() > 0) {
            pub_raw_image0_->publish(image_to_rosmsg(image));
          }
        } else if (image->camera_index == 1 ||
                   image->camera_index == -1 /*depth*/) {
          if (!FLAGS_share_raw_image1_topic.empty() &&
              pub_raw_image1_->getNumSubscribers() > 0) {
            pub_raw_image1_->publish(image_to_rosmsg(image));
          }
        }
      });

  // publish grasp points (tag)
  flow->registerSubscriber<message_flow_topics::TAG_DETECTIONS>(
      kSubscriberNodeName, message_flow::DeliveryOptions(),
      [this](StampedTagDetections::ConstPtr stamped_detections) {
        if (stamped_detections->cam_id == 0) {
          using hear_slam::TagFamily;
          using hear_slam::TagID;
          const TagID target_tag_id(TagFamily::April_36h11, 1);
          GraspObjectId obj_id = target_tag_id.id;
          auto iter = local_grasp_points_map_.find(obj_id);
          if (iter == local_grasp_points_map_.end()) {
            return;            
          }
          const auto& local_grasp_points = iter->second;

          for (const auto& detection :
               stamped_detections->detections) {
            if (detection.tag_id == target_tag_id) {
              const Eigen::Isometry3d& T_Cam_Tag = detection.T_Cam_Tag->toIsometry();
              Eigen::Quaterniond tag_q(T_Cam_Tag.rotation());
              Eigen::Vector3d tag_p(T_Cam_Tag.translation());

              // Publish estimated grasp points.
              geometry_msgs::PoseArray grasp_points_message;
              // grasp_points_message.header.frame_id = "color_camera";
              grasp_points_message.header.frame_id = FLAGS_tf_imu_frame;
                    // TODO: change to camera frame (the camera frame is not
                    // registered to tf for now, so we temporarily use the IMU
                    // frame)
              grasp_points_message.header.stamp = createRosTimestamp(stamped_detections->timestamp_ns);

              for (const auto& local_grasp_point : local_grasp_points) {
                Eigen::Vector3d grasp_position = T_Cam_Tag * local_grasp_point;
                Eigen::Quaterniond grasp_orientation = tag_q;
                geometry_msgs::Pose grasp_point_message;
                tf::pointEigenToMsg(grasp_position, grasp_point_message.position);
                tf::quaternionEigenToMsg(grasp_orientation, grasp_point_message.orientation);
                grasp_points_message.poses.emplace_back(grasp_point_message);
              }

              pub_grasp_points_computed_from_tag_.publish(grasp_points_message);
              break;
            }
          }
        }
      });

  if (FLAGS_publish_only_on_keyframes) {
    flow->registerSubscriber<message_flow_topics::MAP_UPDATES>(
        kSubscriberNodeName, message_flow::DeliveryOptions(),
        [this](const vio::MapUpdate::ConstPtr& vio_update) {
          CHECK(vio_update != nullptr);
          bool has_T_G_M =
              (vio_update->localization_state ==
                   common::LocalizationState::kLocalized ||
               vio_update->localization_state ==
                   common::LocalizationState::kMapTracking);
          publishVinsState(
              vio_update->timestamp_ns, vio_update->vinode, has_T_G_M,
              vio_update->T_G_M);
        });
  } else {
    flow->registerSubscriber<message_flow_topics::OPENVINS_ESTIMATES>(
        kSubscriberNodeName, message_flow::DeliveryOptions(),
        [this](const OpenvinsEstimate::ConstPtr& _state) {
          const OpenvinsEstimate::ConstPtr state = _state;
          CHECK(state != nullptr);
          publishVinsState(
              state->timestamp_ns, state->vinode, state->has_T_G_M,
              state->T_G_M);

          // Publish estimated camera-extrinsics.
          geometry_msgs::PoseArray T_C_Bs_message;
          T_C_Bs_message.header.frame_id = FLAGS_tf_imu_frame;
          T_C_Bs_message.header.stamp = createRosTimestamp(state->timestamp_ns);
          for (const auto& cam_idx_T_C_B :
               state->maplab_camera_index_to_T_C_B) {
            geometry_msgs::Pose T_C_B_message;
            tf::poseKindrToMsg(cam_idx_T_C_B.second, &T_C_B_message);
            T_C_Bs_message.poses.emplace_back(T_C_B_message);
          }
          pub_extrinsics_T_C_Bs_.publish(T_C_Bs_message);
        });
  }

  // CSV export for end-to-end test.
  if (!FLAGS_export_estimated_poses_to_csv.empty()) {
    // Lambda function will take ownership.
    std::shared_ptr<common::FileLogger> file_logger =
        std::make_shared<common::FileLogger>(
            FLAGS_export_estimated_poses_to_csv);
    constexpr char kDelimiter[] = " ";
    file_logger->writeDataWithDelimiterAndNewLine(
        kDelimiter, "# Timestamp [s]", "p_G_Ix", "p_G_Iy", "p_G_Iz", "q_G_Ix",
        "q_G_Iy", "q_G_Iz", "q_G_Iw");
    CHECK(file_logger != nullptr);
    flow->registerSubscriber<message_flow_topics::OPENVINS_ESTIMATES>(
        kSubscriberNodeName, message_flow::DeliveryOptions(),
        [file_logger, kDelimiter, this](const OpenvinsEstimate::ConstPtr& state) {
          CHECK(state != nullptr);
          aslam::Transformation T_M_I = state->vinode.get_T_M_I();

          if (state->has_T_G_M) {
            T_M_I = state->T_G_M * T_M_I;
          }

          file_logger->writeDataWithDelimiterAndNewLine(
              kDelimiter,
              aslam::time::nanoSecondsToSeconds(state->timestamp_ns),
              T_M_I.getPosition(), T_M_I.getEigenQuaternion());
        });
  }

  // publish rgbd local map
  flow->registerSubscriber<message_flow_topics::RGBD_LOCAL_MAP>(
      kSubscriberNodeName, message_flow::DeliveryOptions(),
      [this](DenseMapWrapper::ConstPtr map_wrapper) {
        {
          std::unique_lock<std::mutex> lock(mtx_latest_dense_map_ptr_);
          latest_dense_map_ptr_ = map_wrapper;
        }

        int64_t time_tolerance_ns = 1e8;  // 100 ms
        if (map_wrapper->timestamp_ns - last_published_rgbd_map_timestamp_ns_ <= FLAGS_rgbd_map_publish_interval_s * 1e9 - time_tolerance_ns) {
          return;
        }

        last_published_rgbd_map_timestamp_ns_ = map_wrapper->timestamp_ns;
        sensor_msgs::PointCloud2 point_cloud;
        rgbdLocalMapToPointCloud(map_wrapper->map_data, &point_cloud);
        point_cloud.header.frame_id = FLAGS_tf_mission_frame;
        point_cloud.header.stamp = createRosTimestamp(map_wrapper->timestamp_ns);
        visualization::RVizVisualizationSink::publish<sensor_msgs::PointCloud2>("/rgbd_local_map", point_cloud);
      });
}

void DataPublisherFlow::visualizeMap(const vi_map::VIMap& vi_map) const {
  static constexpr bool kPublishBaseframes = true;
  static constexpr bool kPublishVertices = true;
  static constexpr bool kPublishEdges = true;
  static constexpr bool kPublishLandmarks = true;
  static constexpr bool kPublishAbsolute6DofConstraints = true;
  plotter_->visualizeMap(
      vi_map, kPublishBaseframes, kPublishVertices, kPublishEdges,
      kPublishLandmarks, kPublishAbsolute6DofConstraints);
}

void DataPublisherFlow::publishVinsState(
    int64_t timestamp_ns, const vio::ViNodeState& vinode, const bool has_T_G_M,
    const aslam::Transformation& T_G_M) {
  ros::Time timestamp_ros = createRosTimestamp(timestamp_ns);

  // Check whether publishing is needed.
  const bool maplab_odom_should_publish =
      pub_maplab_odom_T_M_I_.getNumSubscribers() > 0;
  const bool odom_should_publish = pub_odom_T_M_I_.getNumSubscribers() > 0;
  const bool pose_T_M_I_should_publish =
      pub_pose_T_M_I_.getNumSubscribers() > 0;
  const bool velocity_I_should_publish =
      pub_velocity_I_.getNumSubscribers() > 0;
  const bool imu_acc_bias_should_publish =
      pub_imu_acc_bias_.getNumSubscribers() > 0;
  const bool imu_gyro_bias_should_publish =
      pub_imu_gyro_bias_.getNumSubscribers() > 0;

  // Publish pose in mission frame.
  maplab_msgs::OdometryWithImuBiases maplab_odom_T_M_I;
  nav_msgs::Odometry odom_T_M_I;
  // const aslam::Transformation& T_M_I = vinode.get_T_M_I();
  const aslam::Transformation T_M_I = vinode.get_T_M_I();

  aslam::Transformation gravity_aligned_T_M_I;
  {
    Eigen::Isometry3d pose_M_I(T_M_I.getRotation().toImplementation());
    pose_M_I.translation() = T_M_I.getPosition();
    auto Zc = pose_M_I.rotation().col(2);
    float yaw = atan2(Zc.y(), Zc.x());
    pose_M_I.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    // pose_M_I.translation().z() = 0.0;  // Fix z = 0?

    gravity_aligned_T_M_I = aslam::Transformation(pose_M_I.translation(), Eigen::Quaterniond(pose_M_I.rotation()));
  }
  visualization::publishTF(
      gravity_aligned_T_M_I, FLAGS_tf_mission_frame, "imu_with_gravity_aligned", timestamp_ros);

  // Publish height map if available.
  bool always_publish_heightmap = true;
  if (always_publish_heightmap || pub_local_heightmap_->getNumSubscribers() > 0) {
    Eigen::Isometry3d pose(T_M_I.getRotation().toImplementation());
    pose.translation() = T_M_I.getPosition();
    
    Eigen::Isometry3f pose_f = pose.cast<float>();

    // TODO(jeffrey): Publish height map in another thread? Or parallelize getHeightMap().
    auto get_and_publish_height_map = [this, pose_f, timestamp_ros]() {
      DenseMapWrapper::ConstPtr map_wrapper;
      {
        std::unique_lock<std::mutex> lock(mtx_latest_dense_map_ptr_);
        map_wrapper = latest_dense_map_ptr_;
      }
      if (map_wrapper) {
        cv::Mat hmap_img = map_wrapper->map_data->getHeightMap(
            pose_f, FLAGS_height_map_min_h, FLAGS_height_map_max_h, FLAGS_height_map_resolution, FLAGS_height_map_discrepancy_thr,
            FLAGS_height_map_upperbound_param_theta_a,
            FLAGS_height_map_upperbound_param_b,
            FLAGS_height_map_upperbound_param_c,
            FLAGS_height_map_upperbound_param_d
            );

        // cv::medianBlur(src, dst, 5);
        cv::medianBlur(hmap_img, hmap_img, 5);

        cv_bridge::CvImage cv_img;
        cv_img.header.frame_id = "";
        cv_img.header.stamp = timestamp_ros;
        cv_img.image = hmap_img;
        cv_img.encoding = "mono16";
        sensor_msgs::ImagePtr msg = cv_img.toImageMsg();
        pub_local_heightmap_->publish(msg);

        // Publish the height map as a point cloud.
        sensor_msgs::PointCloud2 height_pc;
        const double hmap_resolution = 0.01;
        heightMapToPointCloud(hmap_img, &height_pc, hmap_resolution);
        height_pc.header.frame_id = "imu_with_gravity_aligned";
        height_pc.header.stamp = timestamp_ros;
        visualization::RVizVisualizationSink::publish<sensor_msgs::PointCloud2>("/local_height_map_pc", height_pc);
      }
    };

    height_map_work_queue_->enqueue(get_and_publish_height_map, true);
  }

  if (pose_T_M_I_should_publish || maplab_odom_should_publish ||
      odom_should_publish) {
    geometry_msgs::PoseStamped T_M_I_message;
    tf::poseStampedKindrToMsg(
        T_M_I, timestamp_ros, FLAGS_tf_mission_frame, &T_M_I_message);
    if (pose_T_M_I_should_publish) {
      pub_pose_T_M_I_.publish(T_M_I_message);
    }
    maplab_odom_T_M_I.header = T_M_I_message.header;
    maplab_odom_T_M_I.child_frame_id = FLAGS_tf_imu_frame;
    maplab_odom_T_M_I.pose.pose = T_M_I_message.pose;
    eigenMatrixToOdometryCovariance(
        vinode.getPoseCovariance(), maplab_odom_T_M_I.pose.covariance.data());
    maplab_odom_T_M_I.odometry_state = 0u;  //  = OK

    odom_T_M_I.header = T_M_I_message.header;
    odom_T_M_I.child_frame_id = FLAGS_tf_imu_frame;
    odom_T_M_I.pose.pose = T_M_I_message.pose;
    eigenMatrixToOdometryCovariance(
        vinode.getPoseCovariance(), odom_T_M_I.pose.covariance.data());
  }

  aslam::Transformation T_I_Curdf;
  std::string tf_urdf_cam_frame = "realsense_link";
  {
    // - [1.0, 0.0, 0.0, -0.03022]
    // - [0.0, 1.0, 0.0,  0.0074 ]
    // - [0.0, 0.0, 1.0,  0.01602]
    Eigen::Matrix3d R_I_Curdf;
    R_I_Curdf << 1, 0, 0,
                  0, 1, 0,
                  0, 0, 1;
    Eigen::Vector3d t_I_Curdf(-0.03022, 0.0074, 0.01602);

    T_I_Curdf = aslam::Transformation(t_I_Curdf, Eigen::Quaterniond(R_I_Curdf));
  }

  visualization::publishTF(
      T_M_I, FLAGS_tf_mission_frame, FLAGS_tf_imu_frame, timestamp_ros);
  // visualization::publishTF(
  //     T_M_I * T_I_Curdf, FLAGS_tf_mission_frame, tf_urdf_cam_frame, timestamp_ros);
  visualization::publishTF(
      (T_M_I * T_I_Curdf).inverse(), tf_urdf_cam_frame, FLAGS_tf_mission_frame, timestamp_ros);

  // Publish pose in global frame.
  aslam::Transformation T_G_I = T_G_M * T_M_I;
  // jeffrey: only publish T_G_I when (has T_G_M == true)
  // if (pub_pose_T_G_I_.getNumSubscribers() > 0) {
  if (has_T_G_M && pub_pose_T_G_I_.getNumSubscribers() > 0) {
    geometry_msgs::PoseStamped T_G_I_message;
    tf::poseStampedKindrToMsg(
        T_G_I, timestamp_ros, FLAGS_tf_map_frame, &T_G_I_message);
    pub_pose_T_G_I_.publish(T_G_I_message);
  }

  // Publish transform in global frame.
  if (pub_transform_T_G_I_.getNumSubscribers() > 0) {
    geometry_msgs::TransformStamped Transform_G_I_message;
    Transform_G_I_message.child_frame_id = FLAGS_tf_imu_frame;
    Transform_G_I_message.header.stamp = timestamp_ros;
    tf::transformKindrToMsg(T_G_I, &Transform_G_I_message.transform);
    pub_transform_T_G_I_.publish(Transform_G_I_message);
  }

  // Publish baseframe transformation.
  if (pub_baseframe_T_G_M_.getNumSubscribers() > 0) {
    geometry_msgs::PoseStamped T_G_M_message;
    tf::poseStampedKindrToMsg(
        T_G_M, timestamp_ros, FLAGS_tf_map_frame, &T_G_M_message);
    pub_baseframe_T_G_M_.publish(T_G_M_message);
  }
  // visualization::publishTF(
  //     T_G_M, FLAGS_tf_map_frame, FLAGS_tf_mission_frame, timestamp_ros);
  // visualization::publishTF(
  //     T_G_M.inverse(), FLAGS_tf_mission_frame, FLAGS_tf_map_frame, timestamp_ros);

  // Publish velocity.
  if (velocity_I_should_publish || maplab_odom_should_publish ||
      odom_should_publish) {
    const Eigen::Vector3d& v_M_I = vinode.get_v_M_I();
    geometry_msgs::Vector3Stamped velocity_msg;
    velocity_msg.header.stamp = timestamp_ros;
    velocity_msg.vector.x = v_M_I[0];
    velocity_msg.vector.y = v_M_I[1];
    velocity_msg.vector.z = v_M_I[2];
    // Also copy the velocity to the maplab odom message
    maplab_odom_T_M_I.twist.twist.linear = velocity_msg.vector;
    odom_T_M_I.twist.twist.linear = velocity_msg.vector;
    if (velocity_I_should_publish) {
      pub_velocity_I_.publish(velocity_msg);
    }
    // add the velocity covariance terms
    eigenMatrixToOdometryCovariance(
        vinode.getTwistCovariance(), maplab_odom_T_M_I.twist.covariance.data());
    eigenMatrixToOdometryCovariance(
        vinode.getTwistCovariance(), odom_T_M_I.twist.covariance.data());
  }

  // Publish IMU bias.
  if (imu_acc_bias_should_publish || maplab_odom_should_publish ||
      imu_gyro_bias_should_publish) {
    geometry_msgs::Vector3Stamped bias_msg;
    bias_msg.header.stamp = timestamp_ros;
    Eigen::Matrix<double, 6, 1> imu_bias_acc_gyro = vinode.getImuBias();
    bias_msg.vector.x = imu_bias_acc_gyro[0];
    bias_msg.vector.y = imu_bias_acc_gyro[1];
    bias_msg.vector.z = imu_bias_acc_gyro[2];
    // Also copy the bias to the maplab odom message
    maplab_odom_T_M_I.accel_bias = bias_msg.vector;
    if (imu_acc_bias_should_publish) {
      pub_imu_acc_bias_.publish(bias_msg);
    }

    bias_msg.vector.x = imu_bias_acc_gyro[3];
    bias_msg.vector.y = imu_bias_acc_gyro[4];
    bias_msg.vector.z = imu_bias_acc_gyro[5];
    // Also copy the bias to the maplab odom message
    maplab_odom_T_M_I.gyro_bias = bias_msg.vector;
    if (imu_gyro_bias_should_publish) {
      pub_imu_gyro_bias_.publish(bias_msg);
    }
  }

  if (maplab_odom_should_publish) {
    pub_maplab_odom_T_M_I_.publish(maplab_odom_T_M_I);
  }

  if (odom_should_publish) {
    pub_odom_T_M_I_.publish(odom_T_M_I);
  }

  if (FLAGS_publish_debug_markers) {
    stateDebugCallback(vinode, has_T_G_M, T_G_M);
  }
}

void DataPublisherFlow::stateDebugCallback(
    const vio::ViNodeState& vinode, const bool has_T_G_M,
    const aslam::Transformation& T_G_M) {
  constexpr size_t kMarkerId = 0u;
  visualization::Sphere sphere;
  const aslam::Transformation& T_M_I = vinode.get_T_M_I();
  sphere.position = T_M_I.getPosition();
  sphere.radius = 0.2;
  sphere.color = visualization::kCommonGreen;
  sphere.alpha = 0.8;
  T_M_I_spheres_.push_back(sphere);
  visualization::publishSpheres(
      T_M_I_spheres_, kMarkerId, FLAGS_tf_map_frame, "debug", "debug_T_M_I");

  // Publish OPENVINS velocity
  const aslam::Position3D& t_M_I = T_M_I.getPosition();
  const Eigen::Vector3d& v_M_I = vinode.get_v_M_I();
  visualization::Arrow arrow;
  arrow.from = t_M_I;
  arrow.to = t_M_I + v_M_I;
  arrow.scale = 0.3;
  arrow.color = visualization::kCommonRed;
  arrow.alpha = 0.8;
  visualization::publishArrow(
      arrow, kMarkerId, FLAGS_tf_map_frame, "debug", "debug_v_M_I_");

  // Publish OPENVINS global frame if it is available.
  if (has_T_G_M) {
    aslam::Transformation T_G_I = T_G_M * T_M_I;
    visualization::Sphere sphere;
    sphere.position = T_G_I.getPosition();
    sphere.radius = 0.2;
    sphere.color = visualization::kCommonWhite;
    sphere.alpha = 0.8;
    T_G_I_spheres_.push_back(sphere);

    visualization::publishSpheres(
        T_G_I_spheres_, kMarkerId, FLAGS_tf_map_frame, "debug", "debug_T_G_I");
  }
}

void DataPublisherFlow::localizationCallback(
    const Eigen::Vector3d& p_G_I_lc_pnp) {
  visualization::Sphere sphere;
  sphere.position = p_G_I_lc_pnp;
  sphere.radius = 0.2;
  sphere.color = visualization::kCommonRed;
  sphere.alpha = 0.8;
  T_G_I_loc_spheres_.push_back(sphere);

  constexpr size_t kMarkerId = 0u;
  visualization::publishSpheres(
      T_G_I_loc_spheres_, kMarkerId, FLAGS_tf_map_frame, "debug",
      "debug_T_G_I_raw_localizations");
}

}  //  namespace openvinsli
