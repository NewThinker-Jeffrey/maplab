cmake_minimum_required (VERSION 3.1)
project (map-structure)

#cmake_policy(SET CMP0002 NEW)
set_property(GLOBAL PROPERTY ALLOW_DUPLICATE_CUSTOM_TARGETS 1)

set(PROJECT_SOURCE_DIR ./code)
message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
add_subdirectory (${PROJECT_SOURCE_DIR}/localization-summary-map localization-summary-map)
add_subdirectory (${PROJECT_SOURCE_DIR}/posegraph posegraph)
#add_subdirectory (${PROJECT_SOURCE_DIR}/sensors sensors)
add_subdirectory (${PROJECT_SOURCE_DIR}/vi-map vi-map)

