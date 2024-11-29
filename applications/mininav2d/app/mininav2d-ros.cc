#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
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


DEFINE_string(input_pose_topic, "input_pose", "input_pose_topic");

DEFINE_string(tf_mission_frame, "odom", "tf_mission_frame");
DEFINE_string(tf_map_frame, "map", "tf_map_frame");
DEFINE_string(tf_imu_frame, "imu", "tf_imu_frame");
DEFINE_string(tf_urdf_cam_frame, "head_rs_link", "tf_urdf_cam_frame");

// std::shared_ptr<mininav2d::OpenvinsliNode> openvins_localization_node = nullptr;

// __sighandler_t old_sigint_handler = nullptr;
// void shutdownSigintHandler(int sig) {
//   std::cout << "[APP STATUS] Stop Requested ... " << std::endl;
//   if (openvins_localization_node) {
//     openvins_localization_node->shutdown();
//   }
//   // Keep ros alive to finish the map visualization.

//   // ros::shutdown();

//   // if (old_sigint_handler) {
//   //   old_sigint_handler(sig);
//   // }
// }


static std::unique_ptr<ros::NodeHandle> nh;  ///, nh_private;
static std::unique_ptr<ros::Subscriber> pose_subscriber;
static std::unique_ptr<ros::Publisher> pose_pub;
static std::unique_ptr<mininav2d::Nav2dFlow> nav2d;

using StampedGlobalPose = mininav2d::StampedGlobalPose;
using Pose3d = StampedGlobalPose::Pose3d;
static std::function<void(const StampedGlobalPose::ConstPtr&)> publish_global_pose;

namespace {
void publishTF(
    const Pose3d& T, const std::string& frame_id,
    const std::string& child_frame_id, const ros::Time& ros_time) {
  CHECK(!frame_id.empty());
  CHECK(!child_frame_id.empty());
  const Eigen::Vector3d& p = T.translation();

  tf::Transform tf_transform;
  tf_transform.setOrigin(tf::Vector3(p(0), p(1), p(2)));

  // tf::Quaternion tf_quaternion(
  //     T.getRotation().x(), T.getRotation().y(), T.getRotation().z(),
  //     T.getRotation().w());
  Eigen::Quaterniond q(T.linear().matrix());
  tf::Quaternion tf_quaternion(q.x(), q.y(), q.z(), q.w());
  tf_transform.setRotation(tf_quaternion);

  static tf::TransformBroadcaster tf_br;
  tf_br.sendTransform(
      tf::StampedTransform(tf_transform, ros_time, frame_id, child_frame_id));
}
}  // namespace

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  double pose_x = msg->pose.position.x;
  double pose_y = msg->pose.position.y;
  double pose_z = msg->pose.position.z;
  double pose_qx = msg->pose.orientation.x;
  double pose_qy = msg->pose.orientation.y;
  double pose_qz = msg->pose.orientation.z;
  double pose_qw = msg->pose.orientation.w;

  Eigen::Vector3d position(pose_x, pose_y, pose_z);
  Eigen::Quaterniond orientation(pose_qw, pose_qx, pose_qy, pose_qz);
  Pose3d transform(orientation.toRotationMatrix(), position);

  // // publish to TF
  // publishTF(transform, "world", "robot", msg->header.stamp);

  auto stamped_pose = std::make_shared<mininav2d::StampedGlobalPose>();
  stamped_pose->timestamp_ns = msg->header.stamp.toNSec();
  stamped_pose->odom_pose = transform;
  stamped_pose->global_pose = transform;
  stamped_pose->global_pose_valid = true;
  if (publish_global_pose) {
    publish_global_pose(stamped_pose);  // send to message flow
  }

  // forward to new pose topic
  geometry_msgs::PoseStamped new_pose_msg;
  new_pose_msg.header = msg->header;
  new_pose_msg.pose.position.x = pose_x;
  new_pose_msg.pose.position.y = pose_y;
  new_pose_msg.pose.position.z = pose_z;
  new_pose_msg.pose.orientation.x = pose_qx;
  new_pose_msg.pose.orientation.y = pose_qy;
  new_pose_msg.pose.orientation.z = pose_qz;
  new_pose_msg.pose.orientation.w = pose_qw;
  pose_pub->publish(new_pose_msg);
}


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "mininav2d-ros");
  nh.reset(new ros::NodeHandle);


  pose_pub.reset(new ros::Publisher(
      nh->advertise<geometry_msgs::PoseStamped>("output_pose", 1)));

  std::unique_ptr<message_flow::MessageFlow> flow(
      message_flow::MessageFlow::create<message_flow::MessageDispatcherFifo>(
          1, // common::getNumHardwareThreads(),
          "maplab_msg_flow"));
  publish_global_pose =
      flow->registerPublisher<message_flow_topics::GLOBAL_POSE_FUSION>();

  nav2d.reset(new mininav2d::Nav2dFlow());
  nav2d->attachToMessageFlow(flow.get());


  // // Override the default sigint handler.
  // // This must be set after the first NodeHandle is created.
  // old_sigint_handler = signal(SIGINT, shutdownSigintHandler);

  pose_subscriber.reset(new ros::Subscriber(
      nh->subscribe(FLAGS_input_pose_topic, 1, poseCallback)));

  ros::spin();

  std::cout << "[APP STATUS] Going to shutdown message flow ..." << std::endl;

  flow->shutdown();

  std::cout << "[APP STATUS] Waiting message flow idle ..." << std::endl;

  flow->waitUntilIdle();

  std::cout << "[APP STATUS] All done." << std::endl;

  return 0;
}
