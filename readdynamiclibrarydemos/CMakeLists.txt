cmake_minimum_required(VERSION 3.15)
project(cppdemos)

set(CMAKE_CXX_STANDARD 17)

include("${PROJECT_SOURCE_DIR}/cmake/functions.cmake")
project_initialize_config_parameter()

find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(demo_binary main.cpp configuration.cpp)
target_link_libraries(demo_binary ${PythonLibs} ${PYTHON_LIBRARIES})


