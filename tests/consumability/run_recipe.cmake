cmake_minimum_required(VERSION 3.28)

if (NOT DEFINED PRAXIS_SOURCE_DIR OR NOT DEFINED RECIPE_MODE OR NOT DEFINED RECIPE_OUT)
    message(FATAL_ERROR "praxis: run with -DPRAXIS_SOURCE_DIR=<source> -DRECIPE_MODE=<coordinates|working-tree|published> "
        "-DRECIPE_OUT=<scratch directory> and optionally -DRECIPE_REF=<reference>")
endif ()

set(PRAXIS_RECIPE_ADDRESS "https://github.com/skrede/praxis.git")
set(PRAXIS_RECIPE_MODES coordinates working-tree published)

if (NOT RECIPE_MODE IN_LIST PRAXIS_RECIPE_MODES)
    string(REPLACE ";" ", " praxis_recipe_named "${PRAXIS_RECIPE_MODES}")
    message(FATAL_ERROR "praxis: '${RECIPE_MODE}' is not a mode of this recipe; the modes are ${praxis_recipe_named}")
endif ()

get_filename_component(PRAXIS_SOURCE_DIR "${PRAXIS_SOURCE_DIR}" REALPATH)
get_filename_component(RECIPE_OUT "${RECIPE_OUT}" REALPATH)

if (NOT IS_DIRECTORY "${PRAXIS_SOURCE_DIR}")
    message(FATAL_ERROR "praxis: '${PRAXIS_SOURCE_DIR}' is no directory; PRAXIS_SOURCE_DIR names this repository's source tree")
endif ()

if (RECIPE_OUT STREQUAL PRAXIS_SOURCE_DIR)
    message(FATAL_ERROR "praxis: the output directory is the source tree itself; name a scratch directory the recipe may own")
endif ()

string(FIND "${PRAXIS_SOURCE_DIR}/" "${RECIPE_OUT}/" praxis_recipe_out_contains_source)
if (praxis_recipe_out_contains_source EQUAL 0)
    message(FATAL_ERROR "praxis: the output directory '${RECIPE_OUT}' contains the source tree; "
        "name a scratch directory the recipe may own")
endif ()

file(GLOB praxis_recipe_out_entries "${RECIPE_OUT}/*")
if (praxis_recipe_out_entries)
    message(FATAL_ERROR "praxis: the output directory '${RECIPE_OUT}' is not empty; this recipe writes only into a "
        "directory it creates, so name one that does not yet exist")
endif ()

if (NOT DEFINED RECIPE_REF OR RECIPE_REF STREQUAL "")
    file(READ "${PRAXIS_SOURCE_DIR}/CMakeLists.txt" praxis_recipe_listfile)
    if (NOT praxis_recipe_listfile MATCHES "project *\\( *praxis +VERSION +([0-9]+\\.[0-9]+\\.[0-9]+)")
        message(FATAL_ERROR "praxis: ${PRAXIS_SOURCE_DIR}/CMakeLists.txt declares no version to derive a reference from; "
            "give one with -DRECIPE_REF=<reference>")
    endif ()
    set(RECIPE_REF "v${CMAKE_MATCH_1}")
endif ()

find_program(PRAXIS_RECIPE_GIT NAMES git)
if (NOT PRAXIS_RECIPE_GIT)
    message(FATAL_ERROR "praxis: no git on the path, and the recipe resolves and fetches with it")
endif ()

function(praxis_recipe_step LABEL)
    execute_process(
        COMMAND ${ARGN}
        RESULT_VARIABLE status
        COMMAND_ECHO STDOUT
    )
    if (NOT status EQUAL 0)
        message(FATAL_ERROR "praxis: the ${LABEL} step of the ${RECIPE_MODE} mode failed (${status})")
    endif ()
endfunction()

function(praxis_recipe_executable BUILD_DIR TARGET OUT)
    foreach (candidate IN ITEMS "${TARGET}" "${TARGET}.exe" "Release/${TARGET}" "Release/${TARGET}.exe")
        if (EXISTS "${BUILD_DIR}/${candidate}")
            set(${OUT} "${BUILD_DIR}/${candidate}" PARENT_SCOPE)
            return ()
        endif ()
    endforeach ()
    message(FATAL_ERROR "praxis: the build produced no executable named '${TARGET}' under ${BUILD_DIR}")
endfunction()

if (RECIPE_MODE STREQUAL "coordinates")
    # A bare pattern matches trailing ref components, so `v0.1.0` is answered by `milestone/v0.1.0`;
    # only the full ref paths a fetch would resolve are asked for.
    execute_process(
        COMMAND "${PRAXIS_RECIPE_GIT}" ls-remote --exit-code "${PRAXIS_RECIPE_ADDRESS}"
            "refs/tags/${RECIPE_REF}" "refs/heads/${RECIPE_REF}"
        RESULT_VARIABLE praxis_recipe_resolution
        COMMAND_ECHO STDOUT
    )
    if (NOT praxis_recipe_resolution EQUAL 0)
        message(FATAL_ERROR "praxis: ${PRAXIS_RECIPE_ADDRESS} resolves no reference '${RECIPE_REF}' "
            "(${praxis_recipe_resolution}); publish it, or name another with -DRECIPE_REF=<reference>")
    endif ()
    message(STATUS "praxis: ${PRAXIS_RECIPE_ADDRESS} resolves '${RECIPE_REF}'")
    return ()
endif ()

file(MAKE_DIRECTORY "${RECIPE_OUT}")

set(praxis_recipe_build "${RECIPE_OUT}/build")
set(praxis_recipe_configure
    -S "${PRAXIS_SOURCE_DIR}/tests/consumability/consumer"
    -B "${praxis_recipe_build}"
    -DCMAKE_BUILD_TYPE=Release
    "-DPRAXIS_REPOSITORY=${PRAXIS_RECIPE_ADDRESS}"
    "-DPRAXIS_REFERENCE=${RECIPE_REF}"
)
if (RECIPE_MODE STREQUAL "working-tree")
    list(APPEND praxis_recipe_configure "-DFETCHCONTENT_SOURCE_DIR_PRAXIS=${PRAXIS_SOURCE_DIR}")
endif ()

# FetchContent reads the base directory from the cache variable, never from the environment.
if (DEFINED FETCHCONTENT_BASE_DIR AND NOT FETCHCONTENT_BASE_DIR STREQUAL "")
    list(APPEND praxis_recipe_configure "-DFETCHCONTENT_BASE_DIR=${FETCHCONTENT_BASE_DIR}")
endif ()

praxis_recipe_step("configure" "${CMAKE_COMMAND}" ${praxis_recipe_configure})
praxis_recipe_step("build" "${CMAKE_COMMAND}" --build "${praxis_recipe_build}" --config Release
    --target consumability_consumer --parallel 4)

praxis_recipe_executable("${praxis_recipe_build}" consumability_consumer praxis_recipe_program)
praxis_recipe_step("run" "${praxis_recipe_program}")

message(STATUS "praxis: the consumer built and ran against '${RECIPE_REF}' in ${RECIPE_MODE} mode")
