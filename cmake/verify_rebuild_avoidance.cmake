cmake_minimum_required(VERSION 3.28)

if (NOT DEFINED PRAXIS_SOURCE_DIR OR NOT DEFINED PRAXIS_BUILD_DIR)
    message(FATAL_ERROR "praxis: run with -DPRAXIS_SOURCE_DIR=<source> -DPRAXIS_BUILD_DIR=<build>")
endif ()

get_filename_component(PRAXIS_SOURCE_DIR "${PRAXIS_SOURCE_DIR}" ABSOLUTE)
get_filename_component(PRAXIS_BUILD_DIR "${PRAXIS_BUILD_DIR}" ABSOLUTE)

find_program(PRAXIS_NINJA NAMES ninja)
if (NOT PRAXIS_NINJA)
    message(FATAL_ERROR
        "praxis: Ninja is required, because what a rebuild would do is read from its dry run. "
        "Install it, or configure with -DPRAXIS_BUILD_CONSUMER_TESTS=OFF to run everything else "
        "without it.")
endif ()

include("${CMAKE_CURRENT_LIST_DIR}/listfile.cmake")

set(PRAXIS_CONSUMER_TARGET consumability_consumer)
set(PRAXIS_CONSUMER_DIR "${PRAXIS_SOURCE_DIR}/tests/consumability/consumer")

praxis_listfile_set("${PRAXIS_CONSUMER_DIR}/CMakeLists.txt" ${PRAXIS_CONSUMER_TARGET} praxis_consumer_sources)
if (NOT praxis_consumer_sources)
    message(FATAL_ERROR "praxis: ${PRAXIS_CONSUMER_DIR}/CMakeLists.txt names no source of "
        "${PRAXIS_CONSUMER_TARGET}, so there is nothing of the consumer's own to edit")
endif ()

list(GET praxis_consumer_sources 0 praxis_consumer_source)
set(praxis_consumer_source "${PRAXIS_CONSUMER_DIR}/${praxis_consumer_source}")
if (NOT EXISTS "${praxis_consumer_source}")
    message(FATAL_ERROR "praxis: ${PRAXIS_CONSUMER_DIR}/CMakeLists.txt names '${praxis_consumer_source}', "
        "which is not there")
endif ()

# FetchContent puts praxis and everything praxis brings with it under this directory of the
# consumer's build tree.
set(PRAXIS_DEPENDENCY_AREA "_deps")

set(praxis_recipe_out "${PRAXIS_BUILD_DIR}/consumer")
set(praxis_consumer_build "${praxis_recipe_out}/build")
set(praxis_praxis_build "${praxis_consumer_build}/${PRAXIS_DEPENDENCY_AREA}/praxis-build")

# The recipe writes only into a directory it creates.
file(REMOVE_RECURSE "${praxis_recipe_out}")

# The recipe names no generator, so the environment chooses the one the consumer is configured with.
set(ENV{CMAKE_GENERATOR} "Ninja")

execute_process(
    COMMAND ${CMAKE_COMMAND}
        -DPRAXIS_SOURCE_DIR=${PRAXIS_SOURCE_DIR}
        -DRECIPE_MODE=working-tree
        -DRECIPE_OUT=${praxis_recipe_out}
        -P ${PRAXIS_SOURCE_DIR}/tests/consumability/run_recipe.cmake
    RESULT_VARIABLE status
)
if (NOT status EQUAL 0)
    message(FATAL_ERROR "praxis: the consumer did not build against this working tree (${status})")
endif ()

function(praxis_built_artifacts DIRECTORY OUT)
    set(built "")
    foreach (pattern IN ITEMS "*.o" "*.obj" "*.a" "*.lib")
        file(GLOB_RECURSE matched "${DIRECTORY}/${pattern}")
        list(APPEND built ${matched})
    endforeach ()
    set(${OUT} "${built}" PARENT_SCOPE)
endfunction()

praxis_built_artifacts("${praxis_praxis_build}" praxis_built)
if (NOT praxis_built)
    message(FATAL_ERROR "praxis: the consumer's build left no praxis output under "
        "${praxis_praxis_build}, so nothing was compiled there and a second build would list nothing "
        "whatever the property under test")
endif ()

if (NOT EXISTS "${praxis_consumer_build}/build.ninja")
    message(FATAL_ERROR "praxis: ${praxis_consumer_build} carries no Ninja manifest, and the dry run "
        "read from one is this gate's instrument")
endif ()

# Ninja brings its own manifest up to date before it reports anything, and a dry run cannot: it would
# report the regeneration and stop there. The manifest is read under a name no edge produces.
set(PRAXIS_DRY_RUN_MANIFEST dry-run.ninja)
file(COPY_FILE "${praxis_consumer_build}/build.ninja" "${praxis_consumer_build}/${PRAXIS_DRY_RUN_MANIFEST}")

function(praxis_dry_run BUILD_DIR TARGET OUT_ENTRIES OUT_EXPLANATION)
    execute_process(
        COMMAND ${PRAXIS_NINJA} -f ${PRAXIS_DRY_RUN_MANIFEST} -n ${TARGET}
        WORKING_DIRECTORY "${BUILD_DIR}"
        OUTPUT_VARIABLE reported
        ERROR_VARIABLE ignored
        RESULT_VARIABLE status
    )
    if (NOT status EQUAL 0)
        message(FATAL_ERROR "praxis: the dry run of ${TARGET} in ${BUILD_DIR} failed (${status})")
    endif ()

    execute_process(
        COMMAND ${PRAXIS_NINJA} -f ${PRAXIS_DRY_RUN_MANIFEST} -n -d explain ${TARGET}
        WORKING_DIRECTORY "${BUILD_DIR}"
        OUTPUT_VARIABLE unused
        ERROR_VARIABLE explained
        RESULT_VARIABLE explaining_status
    )
    if (NOT explaining_status EQUAL 0)
        message(FATAL_ERROR "praxis: the explaining dry run of ${TARGET} in ${BUILD_DIR} failed "
            "(${explaining_status})")
    endif ()

    # A status line carries the description of one edge the run would execute; anything else the run
    # writes is not an edge.
    string(REPLACE ";" "\\;" reported "${reported}")
    string(REPLACE "\n" ";" reported "${reported}")
    set(entries "")
    foreach (line IN LISTS reported)
        if (line MATCHES "^\\[[0-9]+/[0-9]+\\] (.+)$")
            list(APPEND entries "${CMAKE_MATCH_1}")
        endif ()
    endforeach ()

    set(${OUT_ENTRIES} "${entries}" PARENT_SCOPE)
    set(${OUT_EXPLANATION} "${explained}" PARENT_SCOPE)
endfunction()

# An edge whose output the explaining run says nothing about is downstream of one it does, so the
# line naming an input that outran its output is taken where the entry's own output is unmentioned.
function(praxis_reason_for EXPLANATION ENTRY OUT)
    string(REGEX REPLACE "^.* " "" output "${ENTRY}")
    string(REPLACE ";" "\\;" lines "${EXPLANATION}")
    string(REPLACE "\n" ";" lines "${lines}")

    set(named "")
    set(provoking "")
    foreach (line IN LISTS lines)
        string(FIND "${line}" "${output}" position)
        if (NOT named AND NOT position EQUAL -1)
            set(named "${line}")
        endif ()
        if (NOT provoking AND line MATCHES "older than most recent input")
            set(provoking "${line}")
        endif ()
    endforeach ()

    if (named)
        set(${OUT} "${named}" PARENT_SCOPE)
    else ()
        set(${OUT} "${provoking}" PARENT_SCOPE)
    endif ()
endfunction()

file(TOUCH_NOCREATE "${praxis_consumer_source}")

praxis_dry_run("${praxis_consumer_build}" ${PRAXIS_CONSUMER_TARGET} praxis_entries praxis_explanation)

if (NOT praxis_entries)
    message(FATAL_ERROR "praxis: editing ${praxis_consumer_source} puts nothing out of date, so the "
        "edit did not take and every absence read from this run would hold by asserting nothing")
endif ()

foreach (entry IN LISTS praxis_entries)
    string(FIND "${entry}" "${PRAXIS_DEPENDENCY_AREA}/" position)
    if (position EQUAL -1)
        continue()
    endif ()

    praxis_reason_for("${praxis_explanation}" "${entry}" praxis_reason)
    if (NOT praxis_reason)
        set(praxis_reason "the explaining run gave no reason for it")
    endif ()
    message(FATAL_ERROR
        "praxis: editing the consumer's own ${praxis_consumer_source} rebuilds something the "
        "consumer acquired rather than wrote:\n"
        "      ${entry}\n"
        "      ${praxis_reason}\n"
        "    Everything under ${PRAXIS_DEPENDENCY_AREA}/ is praxis or a library praxis brought with "
        "it, and it is compiled once.")
endforeach ()

list(LENGTH praxis_entries praxis_entry_count)
message(STATUS "praxis: editing the consumer's own source puts ${praxis_entry_count} entries out of "
    "date and none of them under ${PRAXIS_DEPENDENCY_AREA}/")
