cmake_minimum_required(VERSION 3.28)

if (NOT DEFINED PRAXIS_SOURCE_DIR OR NOT DEFINED PRAXIS_BUILD_DIR)
    message(FATAL_ERROR "praxis: run with -DPRAXIS_SOURCE_DIR=<source> -DPRAXIS_BUILD_DIR=<build>")
endif ()

include("${CMAKE_CURRENT_LIST_DIR}/listfile.cmake")

praxis_listfile_set("${PRAXIS_SOURCE_DIR}/lib/CMakeLists.txt" PRAXIS_CORE_MODULES PRAXIS_CORE_MODULES)
praxis_listfile_set("${PRAXIS_SOURCE_DIR}/lib/CMakeLists.txt" PRAXIS_EXTENSIONS PRAXIS_EXTENSIONS)
if (NOT PRAXIS_CORE_MODULES OR NOT PRAXIS_EXTENSIONS)
    message(FATAL_ERROR "praxis: lib/CMakeLists.txt declares no core-module or extension list")
endif ()

set(PRAXIS_GRAPH_PREFIX "${PRAXIS_BUILD_DIR}/graph/praxis.dot")

execute_process(
    COMMAND ${CMAKE_COMMAND} --graphviz=${PRAXIS_GRAPH_PREFIX} -S ${PRAXIS_SOURCE_DIR} -B ${PRAXIS_BUILD_DIR}
    RESULT_VARIABLE status
)
if (NOT status EQUAL 0)
    message(FATAL_ERROR "praxis: could not generate the link graph (${status})")
endif ()

function(praxis_target_declared TARGET OUT)
    file(READ "${PRAXIS_GRAPH_PREFIX}" graph)
    string(FIND "${graph}" "[ label = \"${TARGET}\\n" aliased)
    string(FIND "${graph}" "[ label = \"${TARGET}\"" plain)
    if (aliased EQUAL -1 AND plain EQUAL -1)
        set(${OUT} FALSE PARENT_SCOPE)
    else ()
        set(${OUT} TRUE PARENT_SCOPE)
    endif ()
endfunction()

# Each per-target file holds that target's full transitive link closure, so a substring search over
# one file answers "does anything this target links reach that dependency". Two cases produce no such
# file and are not the same: a target that links nothing, whose closure is genuinely empty, and a
# target that no longer exists, whose stale file is never rewritten and never removed. Only the
# whole-graph file, which every run rewrites, separates them, so the target is looked up there first
# and a name it does not carry is an error rather than a silent pass.
function(praxis_assert_closure TARGET MODE)
    praxis_target_declared(${TARGET} declared)
    if (NOT declared)
        message(FATAL_ERROR "praxis: ${TARGET} is not a target of the generated link graph")
    endif ()
    set(path "${PRAXIS_GRAPH_PREFIX}.${TARGET}")
    set(closure "")
    if (EXISTS "${path}")
        file(READ "${path}" closure)
    endif ()
    foreach (name IN LISTS ARGN)
        string(FIND "${closure}" "${name}" position)
        if (MODE STREQUAL "ABSENT" AND NOT position EQUAL -1)
            message(FATAL_ERROR "praxis: ${TARGET} reaches ${name} through its link closure")
        endif ()
        if (MODE STREQUAL "PRESENT" AND position EQUAL -1)
            message(FATAL_ERROR "praxis: ${TARGET} does not reach ${name} through its link closure")
        endif ()
    endforeach ()
endfunction()

# What ends a library list: the remaining keywords the module helper accepts, and the closing
# parenthesis. Reading the lists out of the module's own listfile is what keeps the assertions below
# free of any module's name, so an extension added to the umbrella is asserted rather than skipped.
set(PRAXIS_LIBRARY_LIST_END SOURCES PUBLIC_LIBS PRIVATE_LIBS ")")

function(praxis_declared_libraries MODULE OUT_MODULES OUT_DEPENDENCIES OUT_LINKED_PUBLICLY)
    praxis_listfile_tokens("${PRAXIS_SOURCE_DIR}/lib/praxis-${MODULE}/CMakeLists.txt" tokens)

    set(modules "")
    set(dependencies "")
    set(public_dependencies "")
    set(scope "")
    foreach (token IN LISTS tokens)
        if (token STREQUAL "")
            continue()
        elseif (token MATCHES "^(PUBLIC|PRIVATE)_LIBS$")
            set(scope "${CMAKE_MATCH_1}")
        elseif (token IN_LIST PRAXIS_LIBRARY_LIST_END)
            set(scope "")
        elseif (NOT scope)
            continue()
        elseif (token MATCHES "^praxis::([a-z_]+)$")
            list(APPEND modules "praxis_${CMAKE_MATCH_1}")
        else ()
            # A dependency is policed under the namespace its targets come from, so cartan::lie and
            # cartan::serial_chain are one name and a target carrying no namespace is its own.
            string(REGEX REPLACE "::.*$" "" name "${token}")
            list(APPEND dependencies "${name}")
            if (scope STREQUAL "PUBLIC")
                list(APPEND public_dependencies "${name}")
            endif ()
        endif ()
    endforeach ()

    list(REMOVE_DUPLICATES dependencies)
    list(REMOVE_DUPLICATES public_dependencies)
    set(${OUT_MODULES} "${modules}" PARENT_SCOPE)
    set(${OUT_DEPENDENCIES} "${dependencies}" PARENT_SCOPE)
    set(${OUT_LINKED_PUBLICLY} "${public_dependencies}" PARENT_SCOPE)
endfunction()

# What a module may not reach is subject matter, and a gate deriving it from the same declarations
# the closure is built from would only ever agree with them. Each module states it where it is
# declared, and one that states nothing at all stops the gate rather than passing it unasserted.
function(praxis_declared_absences MODULE OUT)
    set(path "${PRAXIS_SOURCE_DIR}/lib/praxis-${MODULE}/CMakeLists.txt")
    praxis_listfile_tokens("${path}" tokens)
    if (NOT "PRAXIS_CLOSURE_ABSENT" IN_LIST tokens)
        message(FATAL_ERROR "praxis: lib/praxis-${MODULE}/CMakeLists.txt names no PRAXIS_CLOSURE_ABSENT list, so nothing says what ${MODULE} may not reach")
    endif ()
    praxis_listfile_set("${path}" PRAXIS_CLOSURE_ABSENT absent)
    if (NOT absent)
        message(FATAL_ERROR "praxis: lib/praxis-${MODULE}/CMakeLists.txt declares an empty PRAXIS_CLOSURE_ABSENT, which asserts nothing while reading as though it does; a module with no absence to claim writes NONE")
    endif ()
    if (absent STREQUAL "NONE")
        set(absent "")
    endif ()
    set(${OUT} "${absent}" PARENT_SCOPE)
endfunction()

# What a module publishes is imposed on every project that links it, so each module names the
# dependencies it may carry into a consumer's build where its link lines are, and one that names
# nothing at all stops the gate rather than passing it unasserted.
function(praxis_declared_publications MODULE OUT)
    set(path "${PRAXIS_SOURCE_DIR}/lib/praxis-${MODULE}/CMakeLists.txt")
    praxis_listfile_tokens("${path}" tokens)
    if (NOT "PRAXIS_PUBLIC_DEPENDENCIES" IN_LIST tokens)
        message(FATAL_ERROR "praxis: lib/praxis-${MODULE}/CMakeLists.txt names no PRAXIS_PUBLIC_DEPENDENCIES list, so nothing says what ${MODULE} may publish")
    endif ()
    praxis_listfile_set("${path}" PRAXIS_PUBLIC_DEPENDENCIES published)
    if (NOT published)
        message(FATAL_ERROR "praxis: lib/praxis-${MODULE}/CMakeLists.txt declares an empty PRAXIS_PUBLIC_DEPENDENCIES, which permits nothing while reading as though it does; a module with nothing to publish writes NONE")
    endif ()
    if (published STREQUAL "NONE")
        set(published "")
    endif ()
    set(${OUT} "${published}" PARENT_SCOPE)
endfunction()

set(praxis_targets "")
set(praxis_extension_targets "")
set(PRAXIS_POLICED_DEPENDENCIES "")
foreach (module IN LISTS PRAXIS_CORE_MODULES PRAXIS_EXTENSIONS)
    list(APPEND praxis_targets praxis_${module})
    praxis_declared_libraries(${module}
        praxis_modules_${module} praxis_dependencies_${module} praxis_linked_publicly_${module})
    praxis_declared_absences(${module} praxis_absent_${module})
    praxis_declared_publications(${module} praxis_published_${module})
    list(APPEND PRAXIS_POLICED_DEPENDENCIES ${praxis_dependencies_${module}})
endforeach ()

list(REMOVE_DUPLICATES PRAXIS_POLICED_DEPENDENCIES)
list(SORT PRAXIS_POLICED_DEPENDENCIES)
if (NOT PRAXIS_POLICED_DEPENDENCIES)
    message(FATAL_ERROR "praxis: no module declares a library outside this repository's namespace, so the policed set is empty and every assertion drawn from it would hold by asserting nothing")
endif ()

foreach (module IN LISTS PRAXIS_CORE_MODULES PRAXIS_EXTENSIONS)
    foreach (name IN LISTS praxis_linked_publicly_${module})
        if (NOT name IN_LIST praxis_published_${module})
            message(FATAL_ERROR "praxis: praxis_${module} links ${name} publicly, and lib/praxis-${module}/CMakeLists.txt does not name ${name} in PRAXIS_PUBLIC_DEPENDENCIES")
        endif ()
    endforeach ()
endforeach ()

foreach (name IN LISTS PRAXIS_EXTENSIONS)
    list(APPEND praxis_extension_targets praxis_${name})
endforeach ()

# An extension is one target carrying its contracts, its reference and its scenery together. A
# second target beside it splitting any of the three off is the shape this library rejected, and
# nothing else in the tree would notice one appearing.
set(PRAXIS_SPLIT_SUFFIXES baseline reference scene)

foreach (name IN LISTS PRAXIS_EXTENSIONS)
    foreach (suffix IN LISTS PRAXIS_SPLIT_SUFFIXES)
        praxis_target_declared(praxis_${name}_${suffix} beside)
        if (beside)
            message(FATAL_ERROR "praxis: praxis_${name}_${suffix} is declared beside praxis_${name}, and an extension is one target")
        endif ()
    endforeach ()
endforeach ()

# The core links no extension, in either direction of the umbrella's two lists. A module declaring
# no library at all is held to every name there is: the machinery each extension binds is the one
# module exported, and linking anything outside the export set is a fatal error at its export call.
foreach (module IN LISTS PRAXIS_CORE_MODULES)
    praxis_assert_closure(praxis_${module} PRESENT ${praxis_modules_${module}} ${praxis_dependencies_${module}})
    praxis_assert_closure(praxis_${module} ABSENT ${praxis_extension_targets} ${praxis_absent_${module}})
    if (NOT praxis_modules_${module} AND NOT praxis_dependencies_${module})
        set(everything ${praxis_targets})
        list(REMOVE_ITEM everything praxis_${module})
        praxis_assert_closure(praxis_${module} ABSENT ${PRAXIS_POLICED_DEPENDENCIES} ${everything})
    endif ()
endforeach ()

# The umbrella's order is the order an extension may link in, so an extension reaches none of those
# listed after it. The names it declares itself are asserted present, so the absences beside them
# are read from a closure file that was actually populated.
foreach (name IN LISTS PRAXIS_EXTENSIONS)
    list(FIND PRAXIS_EXTENSIONS ${name} position)
    set(above_targets "")
    foreach (other IN LISTS PRAXIS_EXTENSIONS)
        list(FIND PRAXIS_EXTENSIONS ${other} other_position)
        if (other_position GREATER position)
            list(APPEND above_targets praxis_${other})
        endif ()
    endforeach ()

    praxis_assert_closure(praxis_${name} PRESENT ${praxis_modules_${name}} ${praxis_dependencies_${name}})
    praxis_assert_closure(praxis_${name} ABSENT ${above_targets} ${praxis_absent_${name}})
endforeach ()
