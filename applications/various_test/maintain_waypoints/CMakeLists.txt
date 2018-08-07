cmake_minimum_required(VERSION 2.8.3)
project(maintain_waypoints)

set(CMAKE_BUILD_TYPE release)
set(CMAKE_CXX_FLAGS "-std=c++11 ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fpic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic -Werror=return-type")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/waypoints_visualizer.cc
        src/waypoints_remap.cc
        src/waypoints_utils.cc
        )
target_link_libraries(${PROJECT_NAME}
		${EXTRA_LIBS})

#######
# APP #
#######

cs_add_executable(visualize_waypt
                visualize_waypt.cc
                )
target_link_libraries(visualize_waypt
                ${PROJECT_NAME}
                )

############
## EXPORT ##
############
cs_install()
cs_export()
