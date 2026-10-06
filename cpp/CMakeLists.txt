# Standalone CMake project for the C++ pieces of this repository.
#
# The Python package px4_offboard is an ament_python package built by colcon and
# is unaffected by this file. The C++ here sits outside ament on purpose:
#
#   px4v_mavlink_frame   dependency-free MAVLink v2 codec (static library)
#   test_mavlink_frame   golden-vector conformance test, registered with CTest
#   gz_cam_sub           Gazebo camera wake-up subscriber (needs gz-transport13 +
#                        gz-msgs10; skipped with a message when they are absent)
#
#   cmake -S cpp -B build/cpp -DCMAKE_BUILD_TYPE=Release
#   cmake --build build/cpp --parallel
#   ctest --test-dir build/cpp --output-on-failure
#   cmake --build build/cpp --target clean      # or: rm -rf build/cpp
cmake_minimum_required(VERSION 3.16)
project(px4v_cpp VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(PX4V_BUILD_GZ_CAM_SUB "Build the Gazebo camera subscriber when gz-transport is found" ON)
option(PX4V_WERROR "Treat warnings as errors" OFF)

add_library(px4v_mavlink_frame STATIC src/mavlink_frame.cpp)
add_library(px4v::mavlink_frame ALIAS px4v_mavlink_frame)
target_include_directories(px4v_mavlink_frame PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(px4v_mavlink_frame PUBLIC cxx_std_17)
target_compile_options(px4v_mavlink_frame PRIVATE -Wall -Wextra -Wpedantic
  $<$<BOOL:${PX4V_WERROR}>:-Werror>)

enable_testing()
add_executable(test_mavlink_frame test/test_mavlink_frame.cpp)
target_link_libraries(test_mavlink_frame PRIVATE px4v::mavlink_frame)
target_compile_options(test_mavlink_frame PRIVATE -Wall -Wextra -Wpedantic
  $<$<BOOL:${PX4V_WERROR}>:-Werror>)
add_test(NAME mavlink_frame_golden_vectors
         COMMAND test_mavlink_frame ${CMAKE_CURRENT_SOURCE_DIR}/test/golden_frames.txt)

if(PX4V_BUILD_GZ_CAM_SUB)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(GZ_TRANSPORT QUIET IMPORTED_TARGET gz-transport13 gz-msgs10)
  endif()
  if(GZ_TRANSPORT_FOUND)
    # Same source file the launch file compiles on demand (scripts/gz_cam_sub.cpp).
    add_executable(gz_cam_sub ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/gz_cam_sub.cpp)
    target_link_libraries(gz_cam_sub PRIVATE PkgConfig::GZ_TRANSPORT)
    message(STATUS "gz_cam_sub: building (gz-transport13 found)")
  else()
    message(STATUS "gz_cam_sub: skipped (gz-transport13 / gz-msgs10 not found)")
  endif()
endif()
