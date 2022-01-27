cmake_minimum_required(VERSION 3.8)
project(cyberdog_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_definitions(-Wclass-memaccess)
endif()
add_definitions(-Wclass-memaccess)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rapidjson REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY include/
  DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(TEST
    test/json_test.cpp)
  include_directories(include)
  ament_add_gtest(jsontest ${TEST})
  ament_target_dependencies(jsontest rclcpp rapidjson)
  install(TARGETS
    jsontest
    DESTINATION lib/${PROJECT_NAME})  
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
