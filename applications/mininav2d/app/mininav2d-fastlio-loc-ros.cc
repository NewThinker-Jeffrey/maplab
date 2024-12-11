#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <signal.h>

#include <Eigen/Geometry>

#include "mininav2d/flow-topics.h"
#include "mininav2d/mininav2d-flow.h"
#include "message-flow/message-flow.h"
#include "message-flow/message-dispatcher-fifo.h"


DEFINE_string(fastlio_odom_topic, "/Odometry", "fastlio_odom_topic");
DEFINE_string(fastlio_reloc_topic, "/map_to_odom", "fastlio_reloc_topic");

DEFINE_string(tf_mission_frame, "odom", "tf_mission_frame");
DEFINE_string(tf_map_frame, "map", "tf_map_frame");
DEFINE_string(tf_imu_frame, "imu", "tf_imu_frame");
DEFINE_string(tf_urdf_cam_frame, "head_rs_link", "tf_urdf_cam_frame");


static std::unique_ptr<ros::NodeHandle> nh;  ///, nh_private;
static std::unique_ptr<ros::Subscriber> fastlio_odom_subscriber;
static std::unique_ptr<ros::Subscriber> fastlio_reloc_subscriber;
static std::unique_ptr<ros::Publisher> publish_global_pose_to_ros;

// nav2d related
using StampedGlobalPose = mininav2d::StampedGlobalPose;
using Pose3d = StampedGlobalPose::Pose3d;
static std::unique_ptr<message_flow::MessageFlow> flow;
static std::function<void(const StampedGlobalPose::ConstPtr&)> publish_global_pose_to_msgflow;
static std::unique_ptr<mininav2d::Nav2dFlow> nav2d;

static std::mutex mutex_T_map_odom;
static Pose3d T_map_odom;
static bool T_map_odom_valid = false;

static Pose3d fromRosPose(const geometry_msgs::Pose& ros_pose) {
  double pose_x =  ros_pose.position.x;
  double pose_y =  ros_pose.position.y;
  double pose_z =  ros_pose.position.z;
  double pose_qx = ros_pose.orientation.x;
  double pose_qy = ros_pose.orientation.y;
  double pose_qz = ros_pose.orientation.z;
  double pose_qw = ros_pose.orientation.w;
  Eigen::Vector3d position(pose_x, pose_y, pose_z);
  Eigen::Quaterniond orientation(pose_qw, pose_qx, pose_qy, pose_qz);
  return Pose3d(orientation.toRotationMatrix(), position);
}

static geometry_msgs::Pose toRosPose(const Pose3d& pose) {
  geometry_msgs::Pose ros_pose;
  Eigen::Quaterniond q(pose.linear().matrix());
  Eigen::Vector3d t = pose.translation();
  ros_pose.position.x = t.x();
  ros_pose.position.y = t.y();
  ros_pose.position.z = t.z();
  ros_pose.orientation.x = q.x();
  ros_pose.orientation.y = q.y();
  ros_pose.orientation.z = q.z();
  ros_pose.orientation.w = q.w();
  return ros_pose;
}

static void fastlioOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // get odom pose from msg
  auto stamped_pose = std::make_shared<StampedGlobalPose>();
  stamped_pose->timestamp_ns = msg->header.stamp.toNSec();
  stamped_pose->odom_pose = fromRosPose(msg->pose.pose);
  stamped_pose->global_pose_valid = false;
  {
    std::unique_lock lock(mutex_T_map_odom);
    if (T_map_odom_valid) {
      stamped_pose->global_pose = T_map_odom * stamped_pose->odom_pose;
      stamped_pose->global_pose_valid = true;
    }
  }

  if (publish_global_pose_to_msgflow) {
    publish_global_pose_to_msgflow(stamped_pose);  // send to message flow
  }

  if (stamped_pose->global_pose_valid && publish_global_pose_to_ros) {
    // forward to new pose topic
    geometry_msgs::PoseStamped new_pose_msg;
    new_pose_msg.header = msg->header;
    new_pose_msg.pose = toRosPose(stamped_pose->global_pose);
    publish_global_pose_to_ros->publish(new_pose_msg);
  }
}

static void fastlioRelocCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // get map_to_odom from msg
  Pose3d map_to_odom = fromRosPose(msg->pose.pose);
  {
    std::unique_lock lock(mutex_T_map_odom);
    T_map_odom = map_to_odom;
    T_map_odom_valid = true;
  }
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "mininav2d_fastlio_loc_ros");
  nh.reset(new ros::NodeHandle);


  publish_global_pose_to_ros.reset(new ros::Publisher(
      nh->advertise<geometry_msgs::PoseStamped>("output_pose", 1)));

  // Create our message flow
  flow.reset(
      message_flow::MessageFlow::create<message_flow::MessageDispatcherFifo>(
          1, // common::getNumHardwareThreads(),
          "maplab_msg_flow"));
  publish_global_pose_to_msgflow =
      flow->registerPublisher<message_flow_topics::GLOBAL_POSE_FUSION>();
  // Nav2d session
  nav2d.reset(new mininav2d::Nav2dFlow());
  nav2d->configFromGflags();
  nav2d->attachToMessageFlow(flow.get());

  // // Override the default sigint handler.
  // // This must be set after the first NodeHandle is created.
  // old_sigint_handler = signal(SIGINT, shutdownSigintHandler);

  fastlio_odom_subscriber.reset(new ros::Subscriber(
      nh->subscribe(FLAGS_fastlio_odom_topic, 1, fastlioOdomCallback)));
  fastlio_reloc_subscriber.reset(new ros::Subscriber(
      nh->subscribe(FLAGS_fastlio_reloc_topic, 1, fastlioRelocCallback)));

  ros::spin();

  std::cout << "[APP STATUS] Going to shutdown message flow ..." << std::endl;

  flow->shutdown();

  std::cout << "[APP STATUS] Waiting message flow idle ..." << std::endl;

  flow->waitUntilIdle();

  std::cout << "[APP STATUS] All done." << std::endl;

  return 0;
}
