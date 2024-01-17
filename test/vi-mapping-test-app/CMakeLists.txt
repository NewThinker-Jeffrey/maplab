cmake_minimum_required (VERSION 2.8)
project(vi_mapping_test_app)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-Wno-unknown-pragmas)

cs_add_library(${PROJECT_NAME} src/vi-mapping-test-app.cc)
# "-lgtest" is also needed when we compile with clang instead of gcc,
# otherwise we get "undefined reference to 'testing::xxx ...".
# if(APPLE)
  target_link_libraries(${PROJECT_NAME} -lgtest)
# endif()

cs_install()
cs_export()
