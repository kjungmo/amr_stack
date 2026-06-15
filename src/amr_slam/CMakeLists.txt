cmake_minimum_required(VERSION 3.10)
project(amr_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(amr_core REQUIRED)
find_package(amr_mapping REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Algorithm library (node-free, unit-testable): correlative scan matcher.
# ---------------------------------------------------------------------------
add_library(amr_slam_lib
  src/scan_matching_slam.cpp
)
target_include_directories(amr_slam_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_compile_features(amr_slam_lib PUBLIC cxx_std_17)
ament_target_dependencies(amr_slam_lib amr_core amr_mapping)

# ---------------------------------------------------------------------------
# Thin ROS node: slam_node.
# ---------------------------------------------------------------------------
add_executable(slam_node src/slam_node.cpp)
target_link_libraries(slam_node amr_slam_lib)
ament_target_dependencies(slam_node
  amr_core
  amr_mapping
  amr_interfaces
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include)
install(TARGETS amr_slam_lib
  EXPORT export_amr_slam
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS slam_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_amr_slam HAS_LIBRARY_TARGET)
ament_export_dependencies(amr_core amr_mapping)
ament_export_include_directories(include)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_scan_matching_slam test/test_scan_matching_slam.cpp)
  target_link_libraries(test_scan_matching_slam amr_slam_lib)
  ament_target_dependencies(test_scan_matching_slam amr_core amr_mapping)
endif()

ament_package()
