cmake_minimum_required(VERSION 3.8)
project(amr_hri)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Qt5 moc/uic/rcc integration — must be set BEFORE find_package(Qt5)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# rviz_common exposes Qt5 usage requirements, but we also link directly.
add_library(amr_hri SHARED
  src/amr_console_panel.cpp
)

target_include_directories(amr_hri PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_compile_features(amr_hri PUBLIC cxx_std_17)

ament_target_dependencies(amr_hri
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  amr_interfaces
  rviz_common
  pluginlib
)

target_link_libraries(amr_hri
  Qt5::Widgets
)

# Register the plugin description with pluginlib / rviz_common
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS amr_hri
  EXPORT export_amr_hri
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_amr_hri HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  amr_interfaces
  rviz_common
  pluginlib
)

ament_package()
