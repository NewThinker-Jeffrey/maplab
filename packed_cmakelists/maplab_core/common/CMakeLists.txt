cmake_minimum_required (VERSION 3.1)
project (common)

#cmake_policy(SET CMP0002 NEW)
set_property(GLOBAL PROPERTY ALLOW_DUPLICATE_CUSTOM_TARGETS 1)

set(PROJECT_SOURCE_DIR ./code)
message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
add_subdirectory (${PROJECT_SOURCE_DIR}/aslam-serialization code/aslam-serialization)
add_subdirectory (${PROJECT_SOURCE_DIR}/console-common code/console-common)
add_subdirectory (${PROJECT_SOURCE_DIR}/dense-reconstruction-common code/dense-reconstruction-common)
add_subdirectory (${PROJECT_SOURCE_DIR}/maplab-common code/maplab-common)
add_subdirectory (${PROJECT_SOURCE_DIR}/message_flow code/message_flow)
add_subdirectory (${PROJECT_SOURCE_DIR}/vio-common code/vio-common)

