cmake_minimum_required(VERSION 3.28)

if (NOT DEFINED PRAXIS_BUILD_DIR)
    message(FATAL_ERROR "praxis: run with -DPRAXIS_BUILD_DIR=<build>")
endif ()

# ctest records every test's complete stdout here, the passing ones included, whatever
# --output-on-failure was asked of it.
set(praxis_record "${PRAXIS_BUILD_DIR}/Testing/Temporary/LastTest.log")
if (NOT EXISTS "${praxis_record}")
    message(FATAL_ERROR
        "praxis: ${praxis_record} does not exist, so the suite has not run in ${PRAXIS_BUILD_DIR}; "
        "run it before asking which cases this platform guarded out")
endif ()

file(STRINGS "${praxis_record}" praxis_lines)

set(praxis_subject "")
set(praxis_pending "")
set(praxis_guarded "")
foreach (line IN LISTS praxis_lines)
    if (line MATCHES "^[0-9]+/[0-9]+ Testing: (.+)$")
        set(praxis_subject "${CMAKE_MATCH_1}")
    elseif (line MATCHES "^(.+):([0-9]+): SKIPPED:$")
        set(praxis_pending "${praxis_subject}")
    elseif (praxis_pending AND line MATCHES "^  (.+)$")
        list(APPEND praxis_guarded "${praxis_pending}: ${CMAKE_MATCH_1}")
        set(praxis_pending "")
    endif ()
endforeach ()

if (NOT praxis_guarded)
    message(STATUS "praxis: this platform guarded out no case")
    return ()
endif ()

list(REMOVE_DUPLICATES praxis_guarded)
foreach (guarded IN LISTS praxis_guarded)
    message(STATUS "praxis: ${guarded}")
endforeach ()
