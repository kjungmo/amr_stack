cmake_minimum_required(VERSION 3.10)
project(amr_mapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(amr_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Algorithm library (node-free, unit-testable): map_io + log-odds mapper.
# ---------------------------------------------------------------------------
add_library(amr_mapping_lib
  src/map_io.cpp
  src/occupancy_grid_mapper.cpp
)
target_include_directories(amr_mapping_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_compile_features(amr_mapping_lib PUBLIC cxx_std_17)
ament_target_dependencies(amr_mapping_lib amr_core)

# ---------------------------------------------------------------------------
# Thin ROS node: map_publisher.
# ---------------------------------------------------------------------------
add_executable(map_publisher src/map_publisher_node.cpp)
target_link_libraries(map_publisher amr_mapping_lib)
ament_target_dependencies(map_publisher amr_core rclcpp nav_msgs)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include)
install(TARGETS amr_mapping_lib
  EXPORT export_amr_mapping
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS map_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_amr_mapping HAS_LIBRARY_TARGET)
ament_export_dependencies(amr_core rclcpp nav_msgs)
ament_export_include_directories(include)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_map_io test/test_map_io.cpp)
  target_link_libraries(test_map_io amr_mapping_lib)
  ament_target_dependencies(test_map_io amr_core)

  ament_add_gtest(test_mapper test/test_mapper.cpp)
  target_link_libraries(test_mapper amr_mapping_lib)
  ament_target_dependencies(test_mapper amr_core)
endif()

ament_package()
