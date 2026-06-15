cmake_minimum_required(VERSION 3.8)
project(amr_localization)

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
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ---- Algorithm library (no ROS, unit-testable) ----------------------------
add_library(amr_localization_lib
  src/motion_model.cpp
  src/sensor_model.cpp
  src/mcl.cpp
)
target_include_directories(amr_localization_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_compile_features(amr_localization_lib PUBLIC cxx_std_17)
ament_target_dependencies(amr_localization_lib amr_core)

# ---- mcl_node executable (thin rclcpp wrapper) ----------------------------
add_executable(mcl_node src/mcl_node.cpp)
target_link_libraries(mcl_node amr_localization_lib)
ament_target_dependencies(mcl_node
  amr_core
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS amr_localization_lib
  EXPORT export_amr_localization
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS mcl_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_amr_localization HAS_LIBRARY_TARGET)
ament_export_dependencies(amr_core)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_localization test/test_localization.cpp)
  target_link_libraries(test_localization amr_localization_lib)
  ament_target_dependencies(test_localization amr_core)
endif()

ament_package()
