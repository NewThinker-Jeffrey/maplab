#ifndef OPENVINSLI_VI_MAP_WITH_MUTEX_H_
#define OPENVINSLI_VI_MAP_WITH_MUTEX_H_

#include <memory>
#include <mutex>

#include <Eigen/Core>
#include <maplab-common/macros.h>
#include <vi-map/vi-map.h>

namespace openvinsli {

struct VIMapWithMutex {
  MAPLAB_POINTER_TYPEDEFS(VIMapWithMutex);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  vi_map::VIMap vi_map;
  mutable std::mutex mutex;
};

}  // namespace openvinsli

#endif  // OPENVINSLI_VI_MAP_WITH_MUTEX_H_
