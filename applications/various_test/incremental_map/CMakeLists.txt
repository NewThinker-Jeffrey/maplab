cmake_minimum_required(VERSION 2.8.3)
project(incremental_map)

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
        src/incremental_pipeline.cc
        )
target_link_libraries(${PROJECT_NAME}
		${EXTRA_LIBS})

#######
# APP #
#######

cs_add_executable(merge_map
                merge_map.cc
                )
target_link_libraries(merge_map
                ${PROJECT_NAME}
                )

############
## EXPORT ##
############
cs_install()
cs_export()
