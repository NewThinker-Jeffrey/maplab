#ifndef OPENVINSLI_OPENVINS_MAPLAB_TIMETRANSLATION_H_
#define OPENVINSLI_OPENVINS_MAPLAB_TIMETRANSLATION_H_

#include <map>

#include <aslam/common/time.h>
#include <maplab-common/accessors.h>
#include <message-flow/message-flow.h>

namespace openvinsli {
// OPENVINS uses seconds (double) and maplab nanoseconds (int64_t) as
// timestamps. To reduce precision loss the timestamps are zeroed using the
// first timestamp of maplab before conversion.
class OpenvinsMaplabTimeTranslation {
 public:
  OpenvinsMaplabTimeTranslation(bool use_zeroed_openvins_time = false)
      : use_zeroed_openvins_time_(use_zeroed_openvins_time),
        first_maplab_time_nanoseconds_(aslam::time::getInvalidTime()) {}

  double convertMaplabToOpenvinsTimestamp(int64_t maplab_time_nanoseconds) const {
    if (!aslam::time::isValidTime(first_maplab_time_nanoseconds_)) {
      CHECK_GE(maplab_time_nanoseconds, 0.0);
      if (use_zeroed_openvins_time_) {
        first_maplab_time_nanoseconds_ =
            maplab_time_nanoseconds - kStartOffsetNanoseconds;
      } else {
        first_maplab_time_nanoseconds_ = 0.0;  // let openvins use time.
      }
    }
    double openvins_time_seconds = aslam::time::nanoSecondsToSeconds(
        maplab_time_nanoseconds - first_maplab_time_nanoseconds_);
    CHECK_GE(openvins_time_seconds, 0.0);
    return openvins_time_seconds;
  }

  int64_t convertOpenvinsToMaplabTimestamp(double openvins_time_seconds) const {
    CHECK_GE(openvins_time_seconds, 0);
    CHECK(aslam::time::isValidTime(first_maplab_time_nanoseconds_));
    int64_t maplab_time_nanoseconds =
        aslam::time::secondsToNanoSeconds(openvins_time_seconds);
    maplab_time_nanoseconds += first_maplab_time_nanoseconds_;
    CHECK_GE(maplab_time_nanoseconds, first_maplab_time_nanoseconds_);
    return maplab_time_nanoseconds;
  }

 private:
  // An offset is added such that the timestamps will not get negative if the
  // first translated timestamp is not the lowest timestamp (might happen due
  // to multi-threading).
  static constexpr int64_t kStartOffsetNanoseconds = aslam::time::seconds(1);
  mutable int64_t first_maplab_time_nanoseconds_;
  bool use_zeroed_openvins_time_ = false;
};
}  // namespace openvinsli
#endif  // OPENVINSLI_OPENVINS_MAPLAB_TIMETRANSLATION_H_
