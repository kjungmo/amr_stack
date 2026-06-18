cmake_minimum_required(VERSION 3.8)
project(amr_reference_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(amr_core REQUIRED)
find_package(amr_api REQUIRED)
find_package(amr_mapping REQUIRED)
find_package(amr_localization REQUIRED)
find_package(amr_slam REQUIRED)
find_package(amr_planning REQUIRED)
find_package(amr_navigation REQUIRED)

install(DIRECTORY include/ DESTINATION include)

ament_export_dependencies(
  amr_core amr_api amr_mapping amr_localization amr_slam amr_planning
  amr_navigation)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_reference_core test/test_reference_core.cpp)
  target_include_directories(test_reference_core PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include")
  ament_target_dependencies(test_reference_core
    amr_core amr_api amr_mapping amr_localization amr_slam amr_planning
    amr_navigation)
endif()

ament_package()
