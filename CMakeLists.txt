cmake_minimum_required(VERSION 3.28)
project(praxis VERSION 0.1.0 LANGUAGES CXX)

set(PRAXIS_CXX_STANDARD 20 CACHE STRING
    "The language standard this tree compiles at; 20 is the floor, the default, and the only value this tree is tested at")
set_property(CACHE PRAXIS_CXX_STANDARD PROPERTY STRINGS 20)

if (PRAXIS_CXX_STANDARD LESS 20)
    message(FATAL_ERROR "praxis: the language standard ${PRAXIS_CXX_STANDARD} is below the C++20 floor, and the floor does not move")
elseif (NOT PRAXIS_CXX_STANDARD EQUAL 20)
    message(FATAL_ERROR "praxis: the language standard ${PRAXIS_CXX_STANDARD} is not a configuration this tree is tested at; C++20 is the floor and the only accepted value")
endif ()

set(CMAKE_CXX_STANDARD ${PRAXIS_CXX_STANDARD})
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(GNUInstallDirs)

option(PRAXIS_BUILD_TESTS "Build unit tests" ${PROJECT_IS_TOP_LEVEL})
option(PRAXIS_BUILD_EXAMPLES "Build the demo application" ${PROJECT_IS_TOP_LEVEL})
option(PRAXIS_BUILD_DISPLAY_TESTS "Build the tests that need a display" OFF)
option(PRAXIS_ENABLE_CTAGS_GATES "Check the public surface, which needs Universal Ctags" ${PROJECT_IS_TOP_LEVEL})
option(PRAXIS_BUILD_CONSUMER_TESTS "Check what a consuming project rebuilds, which configures and builds a project of its own" OFF)

# These read this tree, not a consumer's, so a project that adds praxis runs none of them. None needs
# an extraction tool. The scripts stay usable on their own from a hook or a pipeline.
if (PROJECT_IS_TOP_LEVEL)
    execute_process(
        COMMAND ${CMAKE_COMMAND} -DPRAXIS_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR} -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/verify_conventions.cmake
        RESULT_VARIABLE praxis_conventions_status
        ERROR_VARIABLE praxis_conventions_report
    )
    if (NOT praxis_conventions_status EQUAL 0)
        message(FATAL_ERROR "${praxis_conventions_report}")
    endif ()

    execute_process(
        COMMAND ${CMAKE_COMMAND} -DPRAXIS_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR} -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/verify_declared_sources.cmake
        RESULT_VARIABLE praxis_sources_status
        ERROR_VARIABLE praxis_sources_report
    )
    if (NOT praxis_sources_status EQUAL 0)
        message(FATAL_ERROR "${praxis_sources_report}")
    endif ()

    execute_process(
        COMMAND ${CMAKE_COMMAND} -DPRAXIS_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR} -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/verify_snapshot_freshness.cmake
        RESULT_VARIABLE praxis_freshness_status
    )
    if (NOT praxis_freshness_status EQUAL 0)
        message(FATAL_ERROR "praxis: the public-surface snapshot does not describe this tree; see the report above")
    endif ()
endif ()

include(cmake/dependencies.cmake)
include(cmake/warnings.cmake)
include(cmake/persistent_directory.cmake)
include(cmake/module.cmake)
include(cmake/gates.cmake)

add_subdirectory(lib)

add_subdirectory(presets)

# The export calls run here rather than in a module's listfile because a module is exported only
# once every target its interface names exists, and the order the subdirectories above are added in
# is fixed by the include-leak assertion. Each call writes its own file: export truncates rather than
# appends, so a second call naming the same file erases the first. A target whose interface names one
# outside every export set is refused, so the dependency file carries every target the modules link
# that is not published under this repository's namespace -- the carrier the linear-algebra headers
# travel on among them, because the dependencies' own interfaces name it. That file names nothing out
# of the namespaced one, so a consumer reads it first and the namespaced one after.
set(praxis_dependency_targets imgui implot threepp glfw spdlog pugixml pugixml-static praxis_eigen_headers
    cartan_lie cartan_serial_chain cartan_analytical ctrlpp nucleus_core nucleus_xml
    meios_urdf meios_core meios_io meios_model meios_xacro meios_yaml yaml-cpp)

# The formatting library enters a dependency's public link interface only on a toolchain whose
# standard library does not supply the formatting header, so which set this target belongs to is
# decided by the toolchain rather than by this file.
if (TARGET fmt)
    list(APPEND praxis_dependency_targets fmt)
endif ()

export(TARGETS ${praxis_dependency_targets}
    FILE ${CMAKE_BINARY_DIR}/praxis-dependency-targets.cmake)

export(TARGETS praxis_compat praxis_config praxis_extension praxis_scheduler praxis_scene praxis_evaluation
    praxis_rigid_motion praxis_trajectory praxis_manipulator praxis_rigid_presets praxis_manipulator_presets praxis_all praxis_extensions
    NAMESPACE praxis:: FILE ${CMAKE_BINARY_DIR}/praxis-targets.cmake)

if (PRAXIS_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif ()

if (PRAXIS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif ()
