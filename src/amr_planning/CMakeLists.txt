cmake_minimum_required(VERSION 3.10)
project(amr_planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(amr_core REQUIRED)

# --- Algorithm library (no ROS node; unit-testable standalone) ---
add_library(amr_planning
  src/costmap.cpp
  src/astar.cpp
  src/dwa.cpp
)
target_include_directories(amr_planning PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_compile_features(amr_planning PUBLIC cxx_std_17)
ament_target_dependencies(amr_planning amr_core)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS amr_planning
  EXPORT export_amr_planning
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_amr_planning HAS_LIBRARY_TARGET)
ament_export_dependencies(amr_core)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_planning test/test_planning.cpp)
  target_link_libraries(test_planning amr_planning)
  ament_target_dependencies(test_planning amr_core)
endif()

ament_package()
