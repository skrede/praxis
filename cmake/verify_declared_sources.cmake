cmake_minimum_required(VERSION 3.28)

if (NOT DEFINED PRAXIS_SOURCE_DIR)
    message(FATAL_ERROR "praxis: run with -DPRAXIS_SOURCE_DIR=<source>")
endif ()

# file(GLOB) resolves nothing against a relative directory, so a relative argument would make every
# check below pass by finding no files at all.
get_filename_component(PRAXIS_SOURCE_DIR "${PRAXIS_SOURCE_DIR}" ABSOLUTE)

include("${CMAKE_CURRENT_LIST_DIR}/listfile.cmake")

# What ends a source list: the remaining keywords the module helper accepts, and the closing
# parenthesis, which is the only terminator a module naming no libraries has.
set(PRAXIS_SOURCE_LIST_END PUBLIC_LIBS PRIVATE_LIBS ")")

set(praxis_violations "")

function(praxis_declared_sources PATH OUT)
    praxis_listfile_tokens("${PATH}" tokens)

    set(entries "")
    set(collecting FALSE)
    foreach (token IN LISTS tokens)
        if (token STREQUAL "SOURCES")
            set(collecting TRUE)
        elseif (token IN_LIST PRAXIS_SOURCE_LIST_END)
            set(collecting FALSE)
        elseif (collecting AND NOT token STREQUAL "")
            list(APPEND entries "${token}")
        endif ()
    endforeach ()
    set(${OUT} "${entries}" PARENT_SCOPE)
endfunction()

function(praxis_named_files PATH OUT)
    praxis_listfile_tokens("${PATH}" tokens)

    set(entries "")
    foreach (token IN LISTS tokens)
        if (token MATCHES "\\.(cpp|h)$")
            list(APPEND entries "${token}")
        endif ()
    endforeach ()
    set(${OUT} "${entries}" PARENT_SCOPE)
endfunction()

# A directory holds many targets, and a fixture header shared by several of them is named by each,
# so a repeat here is the shape of the directory rather than a defect. Where one list is the whole
# directory's authority the repeat means something, and the caller asks for the named order instead.
function(praxis_declared_files PATH OUT)
    praxis_named_files("${PATH}" entries)

    if (entries)
        list(REMOVE_DUPLICATES entries)
    endif ()
    set(${OUT} "${entries}" PARENT_SCOPE)
endfunction()

# The argument of every `add_subdirectory` call the listfile makes, in the order it makes them. The
# opening parenthesis is a token of its own, so the argument stands two positions past the keyword.
# A quoted literal names the same folder as the bare form, so the quotes come off; a variable
# reference is not expanded here and stops the gate rather than being read as a folder name.
function(praxis_added_subdirectories PATH OUT)
    praxis_listfile_tokens("${PATH}" tokens)

    set(entries "")
    set(pending 0)
    foreach (token IN LISTS tokens)
        if (token STREQUAL "add_subdirectory")
            set(pending 2)
        elseif (pending EQUAL 1)
            string(REGEX REPLACE "^\"(.*)\"$" "\\1" folder "${token}")
            if (NOT folder MATCHES "^[A-Za-z0-9_.-]+$")
                message(FATAL_ERROR "praxis: ${PATH} adds a subdirectory as `${token}`, which this gate "
                    "cannot resolve: the scenarios it reads are the folders directly beneath the one "
                    "adding them, so the argument must name one of those and nothing else")
            endif ()
            list(APPEND entries "${folder}")
            set(pending 0)
        elseif (pending EQUAL 2)
            set(pending 1)
        endif ()
    endforeach ()
    set(${OUT} "${entries}" PARENT_SCOPE)
endfunction()

# A name said twice is a defect only where one source list is the whole directory's authority, so
# the caller asks for this rather than the comparison every directory goes through applying it. It
# reads the list as the listfile names it, repeats intact.
function(praxis_assert_named_once DIRECTORY OWNER DECLARED_VAR)
    set(found "${praxis_violations}")
    set(seen "")
    foreach (entry IN LISTS ${DECLARED_VAR})
        if (entry IN_LIST seen)
            list(APPEND found "${DIRECTORY}/${entry}: named more than once by the ${OWNER}'s source list")
        endif ()
        list(APPEND seen "${entry}")
    endforeach ()
    set(praxis_violations "${found}" PARENT_SCOPE)
endfunction()

function(praxis_compare DIRECTORY OWNER DECLARED_VAR PRESENT_VAR)
    set(named "${${DECLARED_VAR}}")
    set(found "${praxis_violations}")

    foreach (entry IN LISTS named)
        if (NOT EXISTS "${PRAXIS_SOURCE_DIR}/${DIRECTORY}/${entry}")
            list(APPEND found "${DIRECTORY}/${entry}: named by the ${OWNER}'s source list but absent from the tree")
        endif ()
    endforeach ()

    foreach (entry IN LISTS ${PRESENT_VAR})
        if (NOT entry IN_LIST named)
            list(APPEND found "${DIRECTORY}/${entry}: present in the tree but named by no source list")
        endif ()
    endforeach ()

    set(praxis_violations "${found}" PARENT_SCOPE)
endfunction()

file(GLOB praxis_module_lists RELATIVE "${PRAXIS_SOURCE_DIR}" "${PRAXIS_SOURCE_DIR}/lib/*/CMakeLists.txt")

foreach (listfile IN LISTS praxis_module_lists)
    get_filename_component(module "${listfile}" DIRECTORY)
    praxis_declared_sources("${PRAXIS_SOURCE_DIR}/${listfile}" declared)
    praxis_assert_named_once("${module}" "module" declared)

    file(GLOB_RECURSE present RELATIVE "${PRAXIS_SOURCE_DIR}/${module}"
        "${PRAXIS_SOURCE_DIR}/${module}/*.h" "${PRAXIS_SOURCE_DIR}/${module}/*.cpp")

    praxis_compare("${module}" "module" declared present)
endforeach ()

file(GLOB praxis_test_lists RELATIVE "${PRAXIS_SOURCE_DIR}"
    "${PRAXIS_SOURCE_DIR}/tests/*/CMakeLists.txt" "${PRAXIS_SOURCE_DIR}/tests/*/*/CMakeLists.txt")

foreach (listfile IN LISTS praxis_test_lists)
    get_filename_component(directory "${listfile}" DIRECTORY)
    praxis_declared_files("${PRAXIS_SOURCE_DIR}/${listfile}" declared)

    # Non-recursive: a nested directory carrying its own listfile is a separate project whose
    # sources answer to that listfile, not to this one.
    file(GLOB present RELATIVE "${PRAXIS_SOURCE_DIR}/${directory}"
        "${PRAXIS_SOURCE_DIR}/${directory}/*.h" "${PRAXIS_SOURCE_DIR}/${directory}/*.cpp")

    praxis_compare("${directory}" "test directory" declared present)
endforeach ()

# A scenario contributes its sources to the one preset target through a `target_sources` call in its
# own folder, so a source no such call names is compiled by nothing and linked into nothing.
file(GLOB praxis_preset_lists RELATIVE "${PRAXIS_SOURCE_DIR}" "${PRAXIS_SOURCE_DIR}/presets/*/CMakeLists.txt")

praxis_added_subdirectories("${PRAXIS_SOURCE_DIR}/presets/CMakeLists.txt" praxis_added_scenarios)
set(praxis_scenario_folders "")

foreach (listfile IN LISTS praxis_preset_lists)
    get_filename_component(directory "${listfile}" DIRECTORY)
    get_filename_component(folder "${directory}" NAME)
    list(APPEND praxis_scenario_folders "${folder}")

    praxis_named_files("${PRAXIS_SOURCE_DIR}/${listfile}" named)
    praxis_assert_named_once("${directory}" "preset directory" named)
    praxis_declared_files("${PRAXIS_SOURCE_DIR}/${listfile}" declared)

    # Recursive: a scenario folder carries the one list, so every source beneath it answers to it.
    file(GLOB_RECURSE present RELATIVE "${PRAXIS_SOURCE_DIR}/${directory}"
        "${PRAXIS_SOURCE_DIR}/${directory}/*.h" "${PRAXIS_SOURCE_DIR}/${directory}/*.cpp")

    praxis_compare("${directory}" "preset directory" declared present)

    if (NOT folder IN_LIST praxis_added_scenarios)
        list(APPEND praxis_violations
            "${directory}: carries a source list but is added by no add_subdirectory call")
    endif ()
endforeach ()

foreach (folder IN LISTS praxis_added_scenarios)
    if (NOT folder IN_LIST praxis_scenario_folders)
        list(APPEND praxis_violations
            "presets/${folder}: added by presets/CMakeLists.txt but carries no source list")
    endif ()
endforeach ()

if (IS_DIRECTORY "${PRAXIS_SOURCE_DIR}/presets/include")
    praxis_named_files("${PRAXIS_SOURCE_DIR}/presets/CMakeLists.txt" named)
    praxis_assert_named_once("presets" "preset directory" named)
    praxis_declared_files("${PRAXIS_SOURCE_DIR}/presets/CMakeLists.txt" declared)

    file(GLOB_RECURSE present RELATIVE "${PRAXIS_SOURCE_DIR}/presets"
        "${PRAXIS_SOURCE_DIR}/presets/include/*.h" "${PRAXIS_SOURCE_DIR}/presets/include/*.cpp")

    praxis_compare("presets" "preset directory" declared present)
endif ()

if (praxis_violations)
    list(LENGTH praxis_violations count)
    list(JOIN praxis_violations "\n  " report)
    message(FATAL_ERROR "praxis: ${count} declared-source violation(s):\n  ${report}")
endif ()
