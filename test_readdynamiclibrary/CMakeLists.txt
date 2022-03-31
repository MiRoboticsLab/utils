cmake_minimum_required(VERSION 3.8)
project(test_readdynamiclibrary)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(cppdemos REQUIRED)
find_package(toml REQUIRED)
ament_register_extension("ament_package" "cppdemos"
"functions.cmake"
)
project_initialize_config_parameter()
set(dependencies
  PythonInterp
  PythonLibs
  cppdemos
)

include_directories(include ${PYTHON_INCLUDE_DIRS})

add_executable(test_sodemo src/main.cpp src/configuration.cpp)
target_link_libraries(test_sodemo ${PythonLibs} ${PYTHON_LIBRARIES})
ament_target_dependencies(
  test_sodemo
  ${dependencies}
)

install(TARGETS test_sodemo
  DESTINATION lib/${PROJECT_NAME})

# message('kkkkkkkkkkkkkk:: ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}')
# execute_process(
#     COMMAND
#       cp ${PROJECT_BINARY_DIR}/test_toml.so ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
#     WORKING_DIRECTORY
#         ${PROJECT_BINARY_DIR}
# )
# install(TARGETS ${PROJECT_BINARY_DIR}/test_toml.so
#   DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_export_targets(cyberdog_log HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_package()
