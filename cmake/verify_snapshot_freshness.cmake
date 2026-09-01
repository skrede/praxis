cmake_minimum_required(VERSION 3.28)

if (NOT DEFINED PRAXIS_SOURCE_DIR)
    message(FATAL_ERROR "praxis: run with -DPRAXIS_SOURCE_DIR=<source>")
endif ()

get_filename_component(PRAXIS_SOURCE_DIR "${PRAXIS_SOURCE_DIR}" ABSOLUTE)

set(golden "${PRAXIS_SOURCE_DIR}/tests/golden/public_surface.txt")
if (NOT EXISTS "${golden}")
    return()
endif ()

# A row opens with the header the declaration was read from, and the first tab ends it.
string(ASCII 9 separator)

file(STRINGS "${golden}" rows)
set(vanished "")
set(count 0)
foreach (row IN LISTS rows)
    string(REGEX REPLACE "^([^${separator}]*)${separator}.*$" "\\1" header "${row}")
    if (NOT EXISTS "${PRAXIS_SOURCE_DIR}/${header}")
        math(EXPR count "${count} + 1")
        if (NOT header IN_LIST vanished)
            list(APPEND vanished "${header}")
        endif ()
    endif ()
endforeach ()

if (vanished)
    list(JOIN vanished "\n  " report)
    message(FATAL_ERROR
        "praxis: ${count} row(s) of the public-surface snapshot name a header that no longer "
        "exists, so the snapshot no longer describes the shipped surface:\n  ${report}")
endif ()
