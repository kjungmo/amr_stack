cmake_minimum_required(VERSION 3.8)
project(amr_api)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(amr_core REQUIRED)

# Interface layer: mostly headers; one small .cpp for to_string(BehaviorState).
add_library(amr_api
  src/behavior.cpp
)
target_include_directories(amr_api PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_compile_features(amr_api PUBLIC cxx_std_17)
ament_target_dependencies(amr_api amr_core)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS amr_api
  EXPORT export_amr_api
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_amr_api HAS_LIBRARY_TARGET)
ament_export_dependencies(amr_core)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_amr_api test/test_amr_api.cpp)
  target_link_libraries(test_amr_api amr_api)
  ament_target_dependencies(test_amr_api amr_core)
endif()

ament_package()
