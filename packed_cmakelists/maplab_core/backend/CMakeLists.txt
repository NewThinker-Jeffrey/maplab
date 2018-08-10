cmake_minimum_required (VERSION 3.1)
project (backend)

#cmake_policy(SET CMP0002 NEW)
set_property(GLOBAL PROPERTY ALLOW_DUPLICATE_CUSTOM_TARGETS 1)

set(PROJECT_SOURCE_DIR ./code)
message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
add_subdirectory (${PROJECT_SOURCE_DIR}/map-manager map-manager)
add_subdirectory (${PROJECT_SOURCE_DIR}/map-resources map-resources)

