cmake_minimum_required (VERSION 3.1)
project (maplab_core)

message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
add_subdirectory (${PROJECT_SOURCE_DIR}/algorithms algorithms)
add_subdirectory (${PROJECT_SOURCE_DIR}/aslam_cv2 aslam_cv2)
add_subdirectory (${PROJECT_SOURCE_DIR}/backend backend)
add_subdirectory (${PROJECT_SOURCE_DIR}/common common)
add_subdirectory (${PROJECT_SOURCE_DIR}/map-structure map-structure)

