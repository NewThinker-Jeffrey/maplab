cmake_minimum_required(VERSION 2.8.3)
project(openvinsli)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# set options for openvins
set(CMAKE_CXX_STANDARD 17)  # some openvins header filess are using new features in c++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# openvins headers need some macros for conditional compiling
add_definitions(-DROS_AVAILABLE=1)
option(ENABLE_PANGOLIN "Enable or disable Pangolin viewer" ON)
if (NOT ENABLE_PANGOLIN)
    add_definitions(-DENABLE_PANGOLIN=0)
    message(WARNING "Disable Pangolin viewer!")
else ()
    find_package(Pangolin REQUIRED)
    message(STATUS "Pangolin: " ${Pangolin_FOUND})
    add_definitions(-DENABLE_PANGOLIN=1)
    include_directories(${Pangolin_INCLUDE_DIRS})
    list(APPEND openvins_dep_libs
         ${Pangolin_LIBRARIES})
endif ()
option(USE_HEAR_SLAM "Enable or disable building with hear_slam" OFF)
if (USE_HEAR_SLAM)
    set(HEAR_SLAM_PKG hear_slam)
    add_definitions(-DUSE_HEAR_SLAM)
    message(STATUS "Will use hear_slam!!")
else()
    message(STATUS "Won't use hear_slam.")
endif()


#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}_lib
  src/data-publisher-flow.cc
  src/datasource-factory.cc
  src/datasource-rosbag.cc
  src/datasource-rostopic.cc
  src/datasource-hearslam.cc
  src/datasource.cc
  src/feature-tracking.cc
  src/imu-camera-synchronizer.cc
  src/localizer-flow.cc
  src/localizer-helpers.cc
  src/localizer.cc
  src/map-builder-flow.cc
  src/ros-helpers.cc
  src/openvins-factory.cc
  src/openvins-flow.cc
  src/openvins-localization-handler.cc
  src/openvinsli-node.cc
  src/vio-update-builder.cc
)

target_link_libraries(${PROJECT_NAME}_lib ${openvins_dep_libs})

#######
# APP #
#######
cs_add_executable(openvinsli
  app/openvinsli-app.cc
)
target_link_libraries(openvinsli ${PROJECT_NAME}_lib ${openvins_dep_libs})

#########
# SHARE #
#########
catkin_add_env_hooks(10.openvins-config-template
  SHELLS sh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks
)

##########
# GTESTS #
##########
# catkin_add_gtest(test_feature_tracking test/test-feature-tracking.cc)
# target_link_libraries(test_feature_tracking ${PROJECT_NAME}_lib ${openvins_dep_libs})
# maplab_import_test_maps(test_feature_tracking)

# catkin_add_gtest(test_vio_update_builder test/test-vio-update-builder.cc)
# target_link_libraries(test_vio_update_builder ${PROJECT_NAME}_lib ${openvins_dep_libs})

# catkin_add_gtest(test_ros_helpers test/test-ros-helpers.cc)
# target_link_libraries(test_ros_helpers ${PROJECT_NAME}_lib ${openvins_dep_libs})

############
## EXPORT ##
############
cs_install()
cs_export()
