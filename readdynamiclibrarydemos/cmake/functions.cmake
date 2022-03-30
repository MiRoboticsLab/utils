include(CMakeParseArguments)
include(FindPkgConfig)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

find_package(PythonInterp 3 REQUIRED)
find_package(Cython REQUIRED)

file(GLOB_RECURSE ALL_CONFIG_TOML_FILES "${PROJECT_SOURCE_DIR}/*.toml")
file(GLOB_RECURSE ALL_CONFIG_SCRIPTS_FILES "${PROJECT_SOURCE_DIR}/*.py")

function(project_initialize_config_parameter)
    foreach(ABS_FIL ${ALL_CONFIG_TOML_FILES})
        file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
        get_filename_component(DIR ${REL_FIL} DIRECTORY)
        get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
        # toml--> pyx --> .c source --> .so
        generate_parameter_dynamic_library(${ABS_FIL} ${FIL_WE})
    endforeach()
endfunction()

macro(generate_parameter_dynamic_library FILENAME MODULE)
    execute_process(
        COMMAND
            mkdir -p ${PROJECT_BINARY_DIR}/config
        WORKING_DIRECTORY
            ${PROJECT_BINARY_DIR}
    )

    execute_process(
        COMMAND
            cp ${FILENAME} ${PROJECT_BINARY_DIR}/config
        COMMAND
            cp ${ALL_CONFIG_SCRIPTS_FILES} ${PROJECT_BINARY_DIR}/config -R
        COMMAND
            python3 setup.py build_ext --inplace
        WORKING_DIRECTORY
            ${PROJECT_BINARY_DIR}/config
    )

    execute_process(
        COMMAND
            rm -rf build
        WORKING_DIRECTORY
            ${PROJECT_BINARY_DIR}/config
    )
endmacro()


