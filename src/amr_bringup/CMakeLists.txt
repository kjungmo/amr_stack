cmake_minimum_required(VERSION 3.10)
project(amr_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install launch, config, and rviz directories
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # add_launch_test() is provided by launch_testing_ament_cmake.
  find_package(launch_testing_ament_cmake REQUIRED)

  # SLAM integration test: drive a bounded sweep, build + save an office map.
  add_launch_test(
    test/test_slam.py
    TARGET test_slam_launch
    TIMEOUT 240
  )

  # NAV integration test: localize + navigate the full-office goal tour. Slow
  # (multi-goal autonomous traversal); needs all nodes + the committed map fixture.
  add_launch_test(
    test/test_nav.py
    TARGET test_nav_launch
    TIMEOUT 420
  )
endif()

ament_package()
