#ifndef OPENVINSLI_DATASOURCE_FLOW_H_
#define OPENVINSLI_DATASOURCE_FLOW_H_
#include <memory>

#include <aslam/cameras/ncamera.h>
#include <message-flow/message-flow.h>
#include <sensors/imu.h>
#include <vio-common/rostopic-settings.h>
#include <vio-common/vio-types.h>

#include "openvinsli/datasource-factory.h"
#include "openvinsli/flow-topics.h"

namespace openvinsli {

class DataSourceFlow {
 public:
  explicit DataSourceFlow(const vio_common::RosTopicSettings& topic_settings, bool rgbd=false) {
    datasource_.reset(createAndConfigureDataSourcefromGFlags(topic_settings, rgbd));
    CHECK(datasource_);
  }

  ~DataSourceFlow() {
    shutdown();
  }

  void attachToMessageFlow(message_flow::MessageFlow* flow) {
    CHECK_NOTNULL(flow);
    datasource_->registerImageCallback(
        flow->registerPublisher<message_flow_topics::IMAGE_MEASUREMENTS>());
    datasource_->registerImuCallback(
        flow->registerPublisher<message_flow_topics::IMU_MEASUREMENTS>());
    datasource_->registerOdometryCallback(
        flow->registerPublisher<message_flow_topics::ODOMETRY_MEASUREMENTS>());
  }

  void startStreaming() {
    datasource_->startStreaming();
  }

  void shutdown() {
    datasource_->shutdown();
  }

  void registerEndOfDataCallback(const std::function<void()>& cb) {
    CHECK(cb);
    datasource_->registerEndOfDataCallback(cb);
  }

  DataSource* getDataSource() const {
    return datasource_.get();
  }

 private:
  std::unique_ptr<DataSource> datasource_;
};

}  // namespace openvinsli

#endif  // OPENVINSLI_DATASOURCE_FLOW_H_
