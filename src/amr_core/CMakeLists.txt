cmake_minimum_required(VERSION 3.10)
project(amr_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(amr_core
  src/geometry.cpp
  src/config.cpp
)
target_include_directories(amr_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_compile_features(amr_core PUBLIC cxx_std_17)
# yaml-cpp exports an imported target in modern releases; fall back to the
# plain library variable on older ones.
if(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(amr_core PUBLIC yaml-cpp::yaml-cpp)
else()
  target_link_libraries(amr_core PUBLIC yaml-cpp)
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS amr_core
  EXPORT export_amr_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_amr_core HAS_LIBRARY_TARGET)
ament_export_dependencies(yaml-cpp)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_core test/test_core.cpp)
  target_link_libraries(test_core amr_core)
endif()

ament_package()
