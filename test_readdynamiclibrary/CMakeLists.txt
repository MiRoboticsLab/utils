cmake_minimum_required(VERSION 3.8)
project(test_readdynamiclibrary)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(cyberdog_common REQUIRED) 
find_package(cyberdog_parameter REQUIRED) 
find_package(toml REQUIRED)

ament_register_extension("ament_package" "cyberdog_parameter"
  "functions.cmake"
)

project_initialize_config_parameter()
set(dependencies
  PythonInterp
  PythonLibs
  cyberdog_parameter
  cyberdog_common
  toml
  ament_index_cpp
)

include_directories(include ${PYTHON_INCLUDE_DIRS})

add_executable(test_sodemo src/main.cpp)
target_include_directories(test_sodemo PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>) 

target_link_libraries(test_sodemo ${PythonLibs} ${PYTHON_LIBRARIES})
ament_target_dependencies(
  test_sodemo
  ${dependencies}
)

install(TARGETS test_sodemo
  DESTINATION lib/${PROJECT_NAME})

# ament_export_targets(cyberdog_log HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_package()
