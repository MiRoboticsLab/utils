cmake_minimum_required(VERSION 3.15)
project(cppdemos)

set(CMAKE_CXX_STANDARD 17)

# include("${PROJECT_SOURCE_DIR}/cmake/functions.cmake")
# project_initialize_config_parameter()

# message('----------------------')
# file(GLOB files  *)
# foreach(file IN LISTS files)
#     message(STATUS ${file})
# endforeach(file)
# message('----------------------')

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(PythonInterp 3 REQUIRED)
# find_package(PythonLibs REQUIRED)
# include_directories(${PYTHON_INCLUDE_DIRS})

# message('----------------------')
# message('${PythonLibs}')
# message('${PYTHON_LIBRARIES}')
# message('${CMAKE_CURRENT_LIST_DIR}')
# message('----------------------')

# list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
#   cmake/modules/FindCython.cmake
#   cmake/functions.cmake
# )

# find_package(GTest REQUIRED)
# include_directories(${GTEST_INCLUDE_DIRS})

# add_executable(demo_binary main.cpp configuration.cpp)
# target_link_libraries(demo_binary ${PythonLibs} ${PYTHON_LIBRARIES})

# install(DIRECTORY
#   scripts
#   DESTINATION share/${PROJECT_NAME}/
# )

# ament_package(CONFIG_EXTRAS
#     cmake/FindCython.cmake
#     cmake/functions.cmake
# )


