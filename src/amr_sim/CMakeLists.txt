cmake_minimum_required(VERSION 3.8)
project(amr_sim)

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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- Algorithm library (unit-testable with no running node). ---
add_library(amr_sim_lib
  src/world.cpp
  src/robot.cpp
  src/lidar.cpp
  src/simulator.cpp
)
target_include_directories(amr_sim_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_compile_features(amr_sim_lib PUBLIC cxx_std_17)
ament_target_dependencies(amr_sim_lib amr_core)

# --- Thin ROS node wrapping the library. ---
add_executable(sim_node src/sim_node.cpp)
target_link_libraries(sim_node amr_sim_lib)
ament_target_dependencies(sim_node
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
install(TARGETS amr_sim_lib
  EXPORT export_amr_sim
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# --- Offline tool: render a world YAML to a map_server PGM/YAML pair. ---
add_executable(world_to_map src/world_to_map.cpp)
target_link_libraries(world_to_map amr_sim_lib)
ament_target_dependencies(world_to_map amr_core amr_mapping)

install(TARGETS sim_node world_to_map
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_amr_sim HAS_LIBRARY_TARGET)
ament_export_dependencies(amr_core)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sim test/test_sim.cpp)
  target_link_libraries(test_sim amr_sim_lib)
  ament_target_dependencies(test_sim amr_core)
endif()

ament_package()
