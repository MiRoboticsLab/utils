cmake_minimum_required(VERSION 3.8)
project(cyberdog_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY include/
  DESTINATION include/)

if(LOCAL_BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(protocol REQUIRED)
  include_directories(include)
  include_directories(test)
  add_executable(heartbeats_test test/heartbeats_test.cpp)
  ament_target_dependencies(heartbeats_test rclcpp protocol)
  install(TARGETS
    heartbeats_test
    DESTINATION lib/${PROJECT_NAME})
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories()
ament_package()
