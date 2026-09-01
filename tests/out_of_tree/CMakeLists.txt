include(ExternalProject)

# The dependency carries the ordering: a parallel build would otherwise configure the nested project
# before the library its targets file points at has been archived. Every target the targets file
# exports is named here; the interface targets carry no archive but are listed with the rest.
ExternalProject_Add(out_of_tree_probe_project
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/probe
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/probe
    CMAKE_CACHE_ARGS
    -DPRAXIS_TARGETS:FILEPATH=${CMAKE_BINARY_DIR}/praxis-targets.cmake
    -DPRAXIS_DEPENDENCIES:FILEPATH=${CMAKE_BINARY_DIR}/praxis-dependency-targets.cmake
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    BUILD_ALWAYS TRUE
    INSTALL_COMMAND ""
    TEST_COMMAND ""
    DEPENDS praxis_compat praxis_config praxis_extension praxis_scheduler praxis_scene praxis_evaluation
    praxis_eigen_headers praxis_rigid_motion praxis_trajectory praxis_manipulator
    praxis_rigid_presets praxis_manipulator_presets praxis_all praxis_extensions
)

add_test(NAME out_of_tree_probe COMMAND ${CMAKE_CURRENT_BINARY_DIR}/probe/out_of_tree_probe)

# The test framework is acquired for this repository's own suite and neither published file names it,
# so the consuming project that composes a suite of its own is handed an export written here.
export(TARGETS Catch2 Catch2WithMain NAMESPACE Catch2::
    FILE ${CMAKE_CURRENT_BINARY_DIR}/praxis-test-framework-targets.cmake)

ExternalProject_Add(out_of_tree_checker_project
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/checker
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/checker
    CMAKE_CACHE_ARGS
    -DPRAXIS_TARGETS:FILEPATH=${CMAKE_BINARY_DIR}/praxis-targets.cmake
    -DPRAXIS_DEPENDENCIES:FILEPATH=${CMAKE_BINARY_DIR}/praxis-dependency-targets.cmake
    -DPRAXIS_TEST_FRAMEWORK:FILEPATH=${CMAKE_CURRENT_BINARY_DIR}/praxis-test-framework-targets.cmake
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    BUILD_ALWAYS TRUE
    INSTALL_COMMAND ""
    TEST_COMMAND ""
    DEPENDS praxis_compat praxis_config praxis_extension praxis_scheduler praxis_scene praxis_evaluation
    praxis_eigen_headers praxis_rigid_motion praxis_trajectory praxis_manipulator
    praxis_rigid_presets praxis_manipulator_presets praxis_all praxis_extensions Catch2 Catch2WithMain
)

add_test(NAME out_of_tree_checker COMMAND ${CMAKE_CURRENT_BINARY_DIR}/checker/out_of_tree_checker)
