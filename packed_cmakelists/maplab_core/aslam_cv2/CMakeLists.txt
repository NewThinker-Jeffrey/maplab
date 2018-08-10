cmake_minimum_required (VERSION 3.1)
project (aslam_cv2)

#cmake_policy(SET CMP0002 NEW)
set_property(GLOBAL PROPERTY ALLOW_DUPLICATE_CUSTOM_TARGETS 1)

set(PROJECT_SOURCE_DIR ./code)
message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")

#add_subdirectory (${PROJECT_SOURCE_DIR}/aslam_cv_calibration aslam_cv_calibration)
add_subdirectory (${PROJECT_SOURCE_DIR}/aslam_cv_cameras aslam_cv_cameras)
add_subdirectory (${PROJECT_SOURCE_DIR}/aslam_cv_common aslam_cv_common)
add_subdirectory (${PROJECT_SOURCE_DIR}/aslam_cv_detector aslam_cv_detector)
add_subdirectory (${PROJECT_SOURCE_DIR}/aslam_cv_frames aslam_cv_frames)
add_subdirectory (${PROJECT_SOURCE_DIR}/aslam_cv_geometric_vision aslam_cv_geometric_vision)
add_subdirectory (${PROJECT_SOURCE_DIR}/aslam_cv_matcher aslam_cv_matcher)
add_subdirectory (${PROJECT_SOURCE_DIR}/aslam_cv_pipeline aslam_cv_pipeline)
add_subdirectory (${PROJECT_SOURCE_DIR}/aslam_cv_tracker aslam_cv_tracker)
add_subdirectory (${PROJECT_SOURCE_DIR}/aslam_cv_triangulation aslam_cv_triangulation)
add_subdirectory (${PROJECT_SOURCE_DIR}/aslam_cv_visualization aslam_cv_visualization)


