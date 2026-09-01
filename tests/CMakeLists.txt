function(praxis_add_test SOURCE)
    cmake_parse_arguments(ARG "" "" "LIBS;HEADERS" ${ARGN})
    get_filename_component(name ${SOURCE} NAME_WE)
    add_executable(${name} ${SOURCE} ${ARG_HEADERS})
    praxis_target_warnings(${name})
    target_link_libraries(${name} PRIVATE Catch2::Catch2WithMain ${ARG_LIBS})
    add_test(NAME ${name} COMMAND ${name})
endfunction()

# A probe is a translation unit that must NOT compile, so it is excluded from the ordinary build and
# the test builds that one target: a build command returns non-zero exactly when the compile fails.
# The exit status alone cannot separate that from a target that does not exist or a listfile that no
# longer configures, so the diagnostic the rejection carries is named here and matched by the script.
function(praxis_add_failing_compile SOURCE)
    cmake_parse_arguments(ARG "" "DIAGNOSTIC" "LIBS;OPTIONS" ${ARGN})
    if (NOT ARG_DIAGNOSTIC)
        message(FATAL_ERROR "praxis: ${SOURCE} names no DIAGNOSTIC, so any failure at all would satisfy it")
    endif ()
    get_filename_component(name ${SOURCE} NAME_WE)
    add_library(praxis_failing_${name} OBJECT EXCLUDE_FROM_ALL ${SOURCE})
    praxis_target_warnings(praxis_failing_${name})

    # A rejection the project's advisory warning set only reports needs those flags promoted, and
    # only here: OPTIONS keeps that scoped to the one probe target, and keeps the generated target
    # name from being spelled in a caller's listfile.
    target_compile_options(praxis_failing_${name} PRIVATE ${ARG_OPTIONS})
    target_link_libraries(praxis_failing_${name} PRIVATE ${ARG_LIBS})

    add_test(NAME ${name} COMMAND ${CMAKE_COMMAND}
        -DPRAXIS_BUILD_DIR=${CMAKE_BINARY_DIR}
        -DPRAXIS_TARGET=praxis_failing_${name}
        -DPRAXIS_DIAGNOSTIC=${ARG_DIAGNOSTIC}
        -P ${PROJECT_SOURCE_DIR}/cmake/verify_failing_compile.cmake)

    # Every probe drives a build of the one tree they all share, and concurrent builds of a single
    # build directory race on its reconfigure step and on each other's object files.
    set_tests_properties(${name} PROPERTIES RESOURCE_LOCK praxis_build_tree)
endfunction()

add_subdirectory(support)

# A module's tests are added only when that module is in the umbrella's list, so removing an entry
# there removes its tests rather than failing the configure on a target that no longer exists.
foreach (module IN LISTS PRAXIS_CORE_MODULES PRAXIS_EXTENSIONS)
    if (TARGET praxis_${module})
        add_subdirectory(${module})
    endif ()
endforeach ()

add_subdirectory(aliases)
add_subdirectory(compile_once)
add_subdirectory(failing_compile)
add_subdirectory(out_of_tree)

# A suite that links every extension at once, which a module's own test directory cannot do without
# inverting the layering the module structure exists to hold.
add_subdirectory(presets)

# The demonstration's own sources, which carry no interface and are compiled by no module.
add_subdirectory(examples)

add_subdirectory(gates)
