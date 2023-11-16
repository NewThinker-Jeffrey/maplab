#ifndef OPENVINSLI_DATASOURCE_FACTORY_H_
#define OPENVINSLI_DATASOURCE_FACTORY_H_

#include <string>

#include <glog/logging.h>
#include <sensors/imu.h>
#include <vio-common/rostopic-settings.h>

#include "openvinsli/datasource.h"

namespace openvinsli {
enum class DataSourceType { kRosTopic, kRosBag, kHearSlam };

DataSourceType stringToDataSource(const std::string& str);

// Camera topics are read from the camera calibration file. Caller takes
// ownership!
DataSource* createAndConfigureDataSourcefromGFlags(
    const vio_common::RosTopicSettings& topic_settings);
}  // namespace openvinsli
#endif  // OPENVINSLI_DATASOURCE_FACTORY_H_
