cmake_minimum_required(VERSION 2.8.3)
project(mininav2d)

# NOTE: GTSAM should be find_package()'ed before calling catkin_simple(),
#       otherwise link errors about TBB (used by GTSAM) might raise.
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
list(APPEND mininav2d_dep_libs gtsam gtsam_unstable)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# set options for openvins
set(CMAKE_CXX_STANDARD 17)  # some openvins header filess are using new features in c++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

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
    list(APPEND mininav2d_dep_libs
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
  src/mininav2d-flow.cc
)

target_link_libraries(${PROJECT_NAME}_lib ${mininav2d_dep_libs})

#######
# APP #
#######
cs_add_executable(mininav2d
  app/mininav2d-ros.cc
)
target_link_libraries(mininav2d ${PROJECT_NAME}_lib ${mininav2d_dep_libs})

############
## EXPORT ##
############
cs_install()
cs_export()
