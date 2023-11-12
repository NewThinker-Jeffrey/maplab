#include "rovioli/datasource-factory.h"

#include <string>

#include <aslam/cameras/ncamera.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/file-system-tools.h>

#include "rovioli/datasource-hearslam.h"
#include "rovioli/datasource-rosbag.h"
#include "rovioli/datasource-rostopic.h"

DEFINE_string(
    datasource_type, "rosbag",
    "Data source type: rosbag / rostopic / hearslam");
DEFINE_string(datasource_rosbag, "", "Path to rosbag for bag sources.");
DEFINE_string(datasource_hearslam, "", "Path to hearslam dataset.");

namespace rovioli {

DataSourceType stringToDataSource(const std::string& str) {
  if (str == "rostopic") {
    return DataSourceType::kRosTopic;
  } else if (str == "rosbag") {
    return DataSourceType::kRosBag;
  } else if (str == "hearslam") {
    return DataSourceType::kHearSlam;
  }
  LOG(FATAL) << "Unknown datasource: " << str;
  return DataSourceType::kRosBag;  // Silence warning.
}

DataSource* createAndConfigureDataSourcefromGFlags(
    const vio_common::RosTopicSettings& topic_settings) {
  const DataSourceType source_type = stringToDataSource(FLAGS_datasource_type);
  switch (source_type) {
    case DataSourceType::kRosBag:
      CHECK(!FLAGS_datasource_rosbag.empty());
      CHECK(common::fileExists(FLAGS_datasource_rosbag));
      return new DataSourceRosbag(FLAGS_datasource_rosbag, topic_settings);
      break;
    case DataSourceType::kRosTopic:
      return new DataSourceRostopic(topic_settings);
      break;
    case DataSourceType::kHearSlam:
      // CHECK(!FLAGS_datasource_hearslam.empty());  // use rs_capture?
      // CHECK(common::fileExists(FLAGS_datasource_hearslam));
      return new DataSourceHearslam(FLAGS_datasource_hearslam);
      break;
    default:
      LOG(FATAL);
      break;
  }
  return nullptr;
}
}  // namespace rovioli
