cmake_minimum_required(VERSION 3.8)
project(cyberdog_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 # 暂时使用-Wno-class-memaccess屏蔽掉rapidjson的编译警告
  # 该警告来自于高等级(>=9)的gcc检查
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-class-memaccess)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rapidjson REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cyberdog_common REQUIRED)

install(DIRECTORY include/
  DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

if(LOCAL_BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  include_directories(include)
  include_directories(test)
  ament_add_gtest(heartbeats_test test/heartbeats_test.cpp)
  ament_target_dependencies(heartbeats_test rclcpp rapidjson)
  install(TARGETS
    heartbeats_test
    DESTINATION lib/${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_package()
