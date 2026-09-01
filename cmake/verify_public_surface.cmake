cmake_minimum_required(VERSION 3.28)

if (NOT DEFINED PRAXIS_SOURCE_DIR OR NOT DEFINED PRAXIS_BUILD_DIR)
    message(FATAL_ERROR "praxis: run with -DPRAXIS_SOURCE_DIR=<source> -DPRAXIS_BUILD_DIR=<build>")
endif ()

get_filename_component(PRAXIS_SOURCE_DIR "${PRAXIS_SOURCE_DIR}" ABSOLUTE)

find_program(PRAXIS_CTAGS NAMES ctags)
if (NOT PRAXIS_CTAGS)
    message(FATAL_ERROR
        "praxis: Universal Ctags is required to extract the public surface. Install it, or "
        "configure with -DPRAXIS_ENABLE_CTAGS_GATES=OFF to run everything else without it.")
endif ()

# The record and the extractor are a matched pair; another version diverges from it without the
# surface having changed.
set(PRAXIS_CTAGS_VERSION 6.2.1)

execute_process(
    COMMAND ${PRAXIS_CTAGS} --version
    OUTPUT_VARIABLE reported
    ERROR_VARIABLE ignored
    RESULT_VARIABLE status
)
# Only the first line names the running tool; the two below it name Exuberant Ctags and its version.
string(REGEX MATCH "^[^\n]*" identity "${reported}")
if (NOT status EQUAL 0 OR NOT identity MATCHES "^Universal Ctags ([^(,]+)")
    message(FATAL_ERROR
        "praxis: the ctags at ${PRAXIS_CTAGS} is not Universal Ctags; it identifies itself as "
        "\"${identity}\". The public surface record is extracted by Universal Ctags "
        "${PRAXIS_CTAGS_VERSION}. Install that, or configure with -DPRAXIS_ENABLE_CTAGS_GATES=OFF to "
        "run everything else without it.")
endif ()
set(PRAXIS_CTAGS_FOUND_VERSION "${CMAKE_MATCH_1}")
if (NOT PRAXIS_CTAGS_FOUND_VERSION STREQUAL PRAXIS_CTAGS_VERSION)
    message(FATAL_ERROR
        "praxis: the ctags at ${PRAXIS_CTAGS} is Universal Ctags ${PRAXIS_CTAGS_FOUND_VERSION}, and "
        "the public surface record was extracted by Universal Ctags ${PRAXIS_CTAGS_VERSION}. What "
        "the two extract differs whether or not the surface changed, so this gate will not report "
        "on it. "
        "Install Universal Ctags ${PRAXIS_CTAGS_VERSION}, or configure with "
        "-DPRAXIS_ENABLE_CTAGS_GATES=OFF to run everything else without it.")
endif ()

include("${CMAKE_CURRENT_LIST_DIR}/listfile.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/surface_record.cmake")

set(modules_file "${PRAXIS_SOURCE_DIR}/lib/CMakeLists.txt")
praxis_listfile_set("${modules_file}" PRAXIS_CORE_MODULES PRAXIS_CORE_MODULES)
praxis_listfile_set("${modules_file}" PRAXIS_EXTENSIONS PRAXIS_EXTENSIONS)
if (NOT PRAXIS_CORE_MODULES OR NOT PRAXIS_EXTENSIONS)
    message(FATAL_ERROR "praxis: lib/CMakeLists.txt must declare core and extension module lists")
endif ()

# Every module the umbrella lists ships its include root as public surface. A `detail` directory
# carries a module's internals and is not shipped surface, so no declaration under one is measured.
set(modules ${PRAXIS_CORE_MODULES} ${PRAXIS_EXTENSIONS})
set(headers "")
foreach (name IN LISTS modules)
    set(directory "${PRAXIS_SOURCE_DIR}/lib/praxis-${name}/include")
    if (NOT IS_DIRECTORY "${directory}")
        continue()
    endif ()
    file(GLOB_RECURSE found RELATIVE "${PRAXIS_SOURCE_DIR}" "${directory}/*.h")
    foreach (header IN LISTS found)
        if (NOT header MATCHES "(^|/)detail/")
            list(APPEND headers "${header}")
        endif ()
    endforeach ()
endforeach ()

# The scenario target is named by neither module list and ships from an include root of its own, at
# a fixed path rather than one a list spells. The same `detail` exclusion holds over it.
set(scenarios "${PRAXIS_SOURCE_DIR}/presets/include")
if (IS_DIRECTORY "${scenarios}")
    file(GLOB_RECURSE found RELATIVE "${PRAXIS_SOURCE_DIR}" "${scenarios}/*.h")
    foreach (header IN LISTS found)
        if (NOT header MATCHES "(^|/)detail/")
            list(APPEND headers "${header}")
        endif ()
    endforeach ()
endif ()
list(SORT headers)

# A tab separates a row's fields because a declaration's name and its signature may both carry a
# space, and a name spelling an operator may carry any other punctuation the language admits.
string(ASCII 9 separator)

set(keyed "")
if (headers)
    execute_process(
        COMMAND ${PRAXIS_CTAGS} --languages=C++ --c++-kinds=+p-d --fields=+nSt --sort=no
                "--_xformat=%F${separator}%n${separator}%N${separator}%K${separator}%S${separator}%t"
                -x ${headers}
        WORKING_DIRECTORY "${PRAXIS_SOURCE_DIR}"
        OUTPUT_VARIABLE extracted
        ERROR_VARIABLE ignored
        RESULT_VARIABLE status
    )
    if (NOT status EQUAL 0)
        message(FATAL_ERROR "praxis: ctags failed to read the shipped headers (${status})")
    endif ()
    string(REPLACE "\n" ";" rows "${extracted}")
    list(REMOVE_ITEM rows "")
    foreach (row IN LISTS rows)
        string(REGEX MATCH "^([^${separator}]+)${separator}([0-9]+)${separator}(.*)$" parsed "${row}")
        if (parsed STREQUAL "")
            message(FATAL_ERROR "praxis: the extraction produced a row it cannot read:\n  ${row}")
        endif ()
        set(header "${CMAKE_MATCH_1}")
        set(line "${CMAKE_MATCH_2}")
        string(REGEX REPLACE " +$" "" rest "${CMAKE_MATCH_3}")
        set(padded "00000000${line}")
        string(LENGTH "${padded}" width)
        math(EXPR from "${width} - 8")
        string(SUBSTRING "${padded}" ${from} 8 padded)
        list(APPEND keyed "${header}${separator}${padded}${separator}${rest}")
    endforeach ()
    list(SORT keyed)
endif ()

set(declarations "")
foreach (entry IN LISTS keyed)
    string(REGEX REPLACE "^([^${separator}]+)${separator}[0-9]+${separator}" "\\1${separator}"
           declaration "${entry}")
    list(APPEND declarations "${declaration}")
endforeach ()

list(JOIN declarations "\n" snapshot)
file(WRITE "${PRAXIS_BUILD_DIR}/public_surface.txt" "${snapshot}\n")

set(golden "${PRAXIS_SOURCE_DIR}/tests/golden/public_surface.txt")
if (NOT EXISTS "${golden}")
    return()
endif ()

file(STRINGS "${golden}" recorded)
list(LENGTH recorded recorded_count)
list(LENGTH declarations extracted_count)

set(shared ${recorded_count})
if (extracted_count LESS shared)
    set(shared ${extracted_count})
endif ()
set(divergence "")
set(index 0)
while (index LESS shared)
    list(GET recorded ${index} was)
    list(GET declarations ${index} is)
    if (NOT was STREQUAL is)
        set(divergence "at row ${index}:\n      recorded:  ${was}\n      extracted: ${is}")
        break()
    endif ()
    math(EXPR index "${index} + 1")
endwhile ()
if (NOT divergence AND NOT recorded_count EQUAL extracted_count)
    set(divergence "in length: ${recorded_count} row(s) recorded, ${extracted_count} extracted")
endif ()

praxis_surface_report(recorded declarations "${divergence}")
