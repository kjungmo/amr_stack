cmake_minimum_required(VERSION 3.10)
project(amr_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(amr_core REQUIRED)
find_package(amr_planning REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- Algorithm library (the FSM; node-free, unit-testable standalone) ---
add_library(amr_navigation
  src/navigator.cpp
)
target_include_directories(amr_navigation PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_compile_features(amr_navigation PUBLIC cxx_std_17)
ament_target_dependencies(amr_navigation amr_core amr_planning)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS amr_navigation
  EXPORT export_amr_navigation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_amr_navigation HAS_LIBRARY_TARGET)
ament_export_dependencies(amr_core amr_planning)
ament_export_include_directories(include)

# --- ROS node: thin rclcpp wrapper around the library ---
add_executable(navigator_node src/navigator_node.cpp)
target_link_libraries(navigator_node amr_navigation)
ament_target_dependencies(navigator_node
  amr_core
  amr_planning
  amr_interfaces
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS navigator_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Tests: gtest the FSM library directly (mirrors tests/test_navigator.py) ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_navigator test/test_navigator.cpp)
  target_link_libraries(test_navigator amr_navigation)
  ament_target_dependencies(test_navigator amr_core amr_planning)
endif()

ament_package()
