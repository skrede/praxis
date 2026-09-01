cmake_minimum_required(VERSION 3.28)

if (NOT DEFINED PRAXIS_SOURCE_DIR)
    message(FATAL_ERROR "praxis: run with -DPRAXIS_SOURCE_DIR=<source>")
endif ()

# file(GLOB) resolves nothing against a relative directory, so a relative argument would make every
# check below pass by finding no files at all.
get_filename_component(PRAXIS_SOURCE_DIR "${PRAXIS_SOURCE_DIR}" ABSOLUTE)


include("${CMAKE_CURRENT_LIST_DIR}/listfile.cmake")

# The umbrella listfile names every module once and this gate reads it there rather than carrying a
# copy the two could drift apart. A read that came back empty would make the scans below pass by
# scanning nothing, so it stops the gate instead.
praxis_listfile_set("${PRAXIS_SOURCE_DIR}/lib/CMakeLists.txt" PRAXIS_CORE_MODULES PRAXIS_CORE_MODULES)
praxis_listfile_set("${PRAXIS_SOURCE_DIR}/lib/CMakeLists.txt" PRAXIS_EXTENSIONS PRAXIS_EXTENSIONS)
if (NOT PRAXIS_CORE_MODULES OR NOT PRAXIS_EXTENSIONS)
    message(FATAL_ERROR "praxis: lib/CMakeLists.txt declares no core-module or extension list")
endif ()
set(PRAXIS_MODULE_DIRECTORIES ${PRAXIS_CORE_MODULES} ${PRAXIS_EXTENSIONS})
list(TRANSFORM PRAXIS_MODULE_DIRECTORIES PREPEND "praxis-")

praxis_listfile_set("${PRAXIS_SOURCE_DIR}/lib/CMakeLists.txt" PRAXIS_SYNCHRONIZED_FILES PRAXIS_SYNCHRONIZED_FILES)

set(PRAXIS_DOMAIN_TERMS
    robot robotic joint tool jog flange gripper manipulator actuator
    end_effector kinematic wrist payload waypoint trajectory screw tcp)

# Each term subsumes the forms built on it: mutex reaches shared_mutex and recursive_mutex, atomic
# reaches atomic_flag, and condition_variable reaches condition_variable_any.
set(PRAXIS_SYNCHRONIZATION_TERMS mutex lock_guard unique_lock scoped_lock condition_variable atomic)

# A slot signature carries robotics and nothing else, so every name the scheduler publishes is a
# term here. The shorter forms subsume the longer ones spelled from them.
set(PRAXIS_SCHEDULING_TERMS
    scheduler strand overrun rejection sample_time clock_source step_delta step_period replay_bound
    task_handle task_counters sampled_task stepped_task one_shot_task snapshot_publisher snapshot_reader)

# Every declaration a shipped header carries that holds an angle spells its unit in its own name: a
# floating-point triple named for the Euler angles, and a floating-point scalar named for an angle.
# Screw pitch is not a term here, being an axis translation per radian rather than an angle.
set(PRAXIS_EULER_TERMS euler)
set(PRAXIS_EULER_TYPES vector3f vector3d)
set(PRAXIS_ANGLE_TERMS angle theta roll yaw)
set(PRAXIS_ANGLE_TYPES float double)
set(PRAXIS_ANGLE_UNITS degrees radians)

# Each name below carries a rotation triple in radians, the unit every signature an extension
# publishes is written in. The name itself is published surface rather than spelling: it is a slot,
# the descriptor string the coverage facility reports that slot under, and an enumerator.
set(PRAXIS_ANGLE_EXCLUSIONS euler_from_rotation_matrix rotation_matrix_from_euler)

set(praxis_violations "")

# Every line is prefixed with a period so that a blank one survives the list conversion foreach would
# otherwise drop, which also keeps the reported line numbers true.
function(praxis_comment_free PATH OUT)
    file(READ "${PATH}" text)
    string(REGEX REPLACE "/\\*([^*]|\\*+[^*/])*\\*+/" "" text "${text}")
    string(REGEX REPLACE "//[^\n]*" "" text "${text}")
    string(TOLOWER "${text}" text)
    string(REGEX REPLACE ";" "\\\\;" text "${text}")
    string(REGEX REPLACE "\n" ";." text ".${text}")
    set(${OUT} "${text}" PARENT_SCOPE)
endfunction()

# The guard is the header's own path with the namespace directory dropped once, so a private header
# under a module's src/ takes the module's short name where the include path would have put it.
function(praxis_expected_guard PATH OUT)
    if (PATH MATCHES "^lib/praxis-[a-z_]+/include/praxis/(.+)$")
        set(tail "${CMAKE_MATCH_1}")
    elseif (PATH MATCHES "^lib/praxis-([a-z_]+)/src/(.+)$")
        set(tail "${CMAKE_MATCH_1}/${CMAKE_MATCH_2}")
    else ()
        set(tail "")
    endif ()
    string(REGEX REPLACE "\\.h$" "" tail "${tail}")
    string(REGEX REPLACE "[/.]" "_" tail "${tail}")
    string(TOUPPER "${tail}" tail)
    set(${OUT} "HPP_GUARD_PRAXIS_${tail}_H" PARENT_SCOPE)
endfunction()

# A word boundary is "not a letter" rather than the regular-expression one, so m_tool and tool_tf are
# caught while SetTooltip is not, and an underscore inside a term is matched literally.
function(praxis_line_terms LINE TERMS SUFFIX OUT)
    set(found "")
    foreach (term IN LISTS TERMS)
        if (LINE MATCHES "(^|[^a-z])${term}${SUFFIX}([^a-z]|$)")
            list(APPEND found "${term}")
        endif ()
    endforeach ()
    set(${OUT} "${found}" PARENT_SCOPE)
endfunction()

# The entries given here are declaration names rather than domain words, so the underscore and the
# digits sit inside the boundary alongside the letters: an identifier that merely contains an entry is
# a different name and is not one of them.
function(praxis_line_names LINE NAMES OUT)
    set(found "")
    foreach (name IN LISTS NAMES)
        if (LINE MATCHES "(^|[^a-z_0-9])${name}([^a-z_0-9]|$)")
            list(APPEND found "${name}")
        endif ()
    endforeach ()
    set(${OUT} "${found}" PARENT_SCOPE)
endfunction()

function(praxis_scan_terms PATH TERMS SUFFIX MESSAGE OUT)
    praxis_comment_free("${PRAXIS_SOURCE_DIR}/${PATH}" lines)
    string(JOIN "" whole ${lines})
    set(present "")
    foreach (term IN LISTS TERMS)
        string(FIND "${whole}" "${term}" position)
        if (NOT position EQUAL -1)
            list(APPEND present "${term}")
        endif ()
    endforeach ()

    set(found "")
    set(number 0)
    foreach (line IN LISTS lines)
        math(EXPR number "${number} + 1")
        praxis_line_terms("${line}" "${present}" "${SUFFIX}" named)
        foreach (term IN LISTS named)
            list(APPEND found "${PATH}:${number}: ${MESSAGE} '${term}'")
        endforeach ()
    endforeach ()
    set(${OUT} "${found}" PARENT_SCOPE)
endfunction()

# A declaration is read one line at a time, so the type, the angle term and the unit are looked for
# together on the line that declares the value rather than anywhere in the file. The exclusion list is
# consulted only after a line has been found to be a violation, so the entries reported back through
# EXCUSED are exactly the entries that silenced one.
function(praxis_angle_verdict LINE TERMS TYPES EXCLUSIONS NAMED_OUT EXCUSED_OUT)
    set(${NAMED_OUT} "" PARENT_SCOPE)
    set(${EXCUSED_OUT} "" PARENT_SCOPE)
    praxis_line_terms("${LINE}" "${TYPES}" "" carried)
    praxis_line_terms("${LINE}" "${PRAXIS_ANGLE_UNITS}" "" spelled)
    if (NOT carried OR spelled)
        return()
    endif ()
    praxis_line_terms("${LINE}" "${TERMS}" "" named)
    if (NOT named)
        return()
    endif ()
    praxis_line_names("${LINE}" "${EXCLUSIONS}" excluded)
    if (excluded)
        set(${EXCUSED_OUT} "${excluded}" PARENT_SCOPE)
        return()
    endif ()
    set(${NAMED_OUT} "${named}" PARENT_SCOPE)
endfunction()

function(praxis_scan_angle_units PATH TERMS TYPES EXCLUSIONS MESSAGE OUT EXCUSED)
    praxis_comment_free("${PRAXIS_SOURCE_DIR}/${PATH}" lines)
    set(found "")
    set(excusing "")
    set(number 0)
    foreach (line IN LISTS lines)
        math(EXPR number "${number} + 1")
        praxis_angle_verdict("${line}" "${TERMS}" "${TYPES}" "${EXCLUSIONS}" named excused)
        list(APPEND excusing ${excused})
        foreach (term IN LISTS named)
            list(APPEND found "${PATH}:${number}: ${MESSAGE} '${term}'")
        endforeach ()
    endforeach ()
    set(${OUT} "${found}" PARENT_SCOPE)
    set(${EXCUSED} "${excusing}" PARENT_SCOPE)
endfunction()

# The optional trailing s keeps plurals in reach.
# A run of value boxes is given the width left over once the step buttons beside it are drawn, split
# between its components and floored at one pixel, so a run asked for with steps draws numbers at a
# width nothing can read. The steps belong to a box holding one value.
function(praxis_scan_stepped_runs PATH OUT)
    praxis_comment_free("${PRAXIS_SOURCE_DIR}/${PATH}" lines)
    set(found "")
    set(number 0)
    foreach (line IN LISTS lines)
        math(EXPR number "${number} + 1")
        if (line MATCHES "inputscalarn" AND NOT line MATCHES "nullptr,[ \t]*nullptr")
            list(APPEND found "${PATH}:${number}: a run of value boxes is drawn with the step buttons beside it")
        endif ()
    endforeach ()
    set(${OUT} "${found}" PARENT_SCOPE)
endfunction()

function(praxis_scan_vocabulary PATH TERMS OUT)
    praxis_scan_terms("${PATH}" "${TERMS}" "s?" "the core names the domain term" found)
    set(${OUT} "${found}" PARENT_SCOPE)
endfunction()

function(praxis_scan_extension_names PATH TERMS OUT)
    praxis_scan_terms("${PATH}" "${TERMS}" "" "a core module names the extension" found)
    set(${OUT} "${found}" PARENT_SCOPE)
endfunction()

function(praxis_scan_synchronization PATH TERMS OUT)
    praxis_scan_terms("${PATH}" "${TERMS}" "" "a synchronization primitive outside the reviewed allowlist" found)
    set(${OUT} "${found}" PARENT_SCOPE)
endfunction()

function(praxis_scan_scheduling PATH TERMS OUT)
    praxis_scan_terms("${PATH}" "${TERMS}" "" "a slot table names the scheduling type" found)
    set(${OUT} "${found}" PARENT_SCOPE)
endfunction()

# A slot's signature is written in the capability header the slot table's enumerators are declared
# against, so the group scanned is the table, the descriptors that read it, and the headers of its
# own extension the table names. Following the table's own includes keeps the group true as an
# extension gains a capability.
function(praxis_slot_table_group MODULE OUT)
    set(table "lib/praxis-${MODULE}/include/praxis/${MODULE}/slots.h")
    set(group "${table}" "lib/praxis-${MODULE}/src/descriptors.cpp")
    if (EXISTS "${PRAXIS_SOURCE_DIR}/${table}")
        file(READ "${PRAXIS_SOURCE_DIR}/${table}" text)
        string(REGEX MATCHALL "\"praxis/${MODULE}/[a-z_/]+\\.h\"" named "${text}")
        foreach (entry IN LISTS named)
            string(REPLACE "\"" "" entry "${entry}")
            list(APPEND group "lib/praxis-${MODULE}/include/${entry}")
        endforeach ()
    endif ()
    set(${OUT} "${group}" PARENT_SCOPE)
endfunction()

file(GLOB_RECURSE praxis_headers RELATIVE "${PRAXIS_SOURCE_DIR}"
    "${PRAXIS_SOURCE_DIR}/lib/*/include/*.h" "${PRAXIS_SOURCE_DIR}/lib/*/src/*.h")
file(GLOB_RECURSE praxis_sources RELATIVE "${PRAXIS_SOURCE_DIR}" "${PRAXIS_SOURCE_DIR}/lib/*/src/*.cpp")
file(GLOB_RECURSE praxis_misplaced RELATIVE "${PRAXIS_SOURCE_DIR}" "${PRAXIS_SOURCE_DIR}/lib/*/include/*.cpp")

foreach (path IN LISTS praxis_misplaced)
    list(APPEND praxis_violations "${path}: an implementation file lives under an include directory")
endforeach ()

file(GLOB_RECURSE praxis_drawn RELATIVE "${PRAXIS_SOURCE_DIR}"
    "${PRAXIS_SOURCE_DIR}/presets/*.h" "${PRAXIS_SOURCE_DIR}/presets/*.cpp"
    "${PRAXIS_SOURCE_DIR}/examples/*.h" "${PRAXIS_SOURCE_DIR}/examples/*.cpp")

foreach (path IN LISTS praxis_headers praxis_sources praxis_drawn)
    praxis_scan_stepped_runs("${path}" found)
    list(APPEND praxis_violations ${found})
endforeach ()

foreach (path IN LISTS praxis_headers)
    file(READ "${PRAXIS_SOURCE_DIR}/${path}" text)
    praxis_expected_guard("${path}" guard)
    if (NOT text MATCHES "#ifndef[ \t]+${guard}" OR NOT text MATCHES "#define[ \t]+${guard}")
        list(APPEND praxis_violations "${path}: the include guard is not ${guard}")
    endif ()
endforeach ()

set(praxis_compat_patterns "")
foreach (module IN LISTS PRAXIS_CORE_MODULES)
    list(APPEND praxis_compat_patterns
        "${PRAXIS_SOURCE_DIR}/lib/praxis-${module}/*.h" "${PRAXIS_SOURCE_DIR}/lib/praxis-${module}/*.cpp")
endforeach ()
file(GLOB_RECURSE praxis_compat RELATIVE "${PRAXIS_SOURCE_DIR}" ${praxis_compat_patterns})

foreach (path IN LISTS praxis_compat)
    praxis_scan_vocabulary("${path}" "${PRAXIS_DOMAIN_TERMS}" found)
    list(APPEND praxis_violations ${found})
    praxis_scan_extension_names("${path}" "${PRAXIS_EXTENSIONS}" found)
    list(APPEND praxis_violations ${found})
endforeach ()

set(praxis_synchronized_patterns "")
foreach (module IN LISTS PRAXIS_CORE_MODULES PRAXIS_EXTENSIONS)
    list(APPEND praxis_synchronized_patterns
        "${PRAXIS_SOURCE_DIR}/lib/praxis-${module}/*.h" "${PRAXIS_SOURCE_DIR}/lib/praxis-${module}/*.cpp")
endforeach ()
file(GLOB_RECURSE praxis_synchronized RELATIVE "${PRAXIS_SOURCE_DIR}" ${praxis_synchronized_patterns})

foreach (path IN LISTS praxis_synchronized)
    if (path IN_LIST PRAXIS_SYNCHRONIZED_FILES)
        continue()
    endif ()
    praxis_scan_synchronization("${path}" "${PRAXIS_SYNCHRONIZATION_TERMS}" found)
    list(APPEND praxis_violations ${found})
endforeach ()

# An entry that names a file which is gone, or one whose primitive has since been removed, excuses
# nothing: it is a claim about a tree that no longer exists and it stops the configure.
foreach (path IN LISTS PRAXIS_SYNCHRONIZED_FILES)
    if (NOT EXISTS "${PRAXIS_SOURCE_DIR}/${path}")
        list(APPEND praxis_violations "${path}: allowlisted for a synchronization primitive but absent from the tree")
        continue()
    endif ()
    praxis_scan_synchronization("${path}" "${PRAXIS_SYNCHRONIZATION_TERMS}" found)
    if (NOT found)
        list(APPEND praxis_violations "${path}: allowlisted for a synchronization primitive it no longer carries")
    endif ()
endforeach ()

set(praxis_angle_exclusions_excusing "")
foreach (path IN LISTS praxis_headers)
    praxis_scan_angle_units("${path}" "${PRAXIS_EULER_TERMS}" "${PRAXIS_EULER_TYPES}" "${PRAXIS_ANGLE_EXCLUSIONS}"
                            "a rotation triple that does not spell its unit" found excused)
    list(APPEND praxis_violations ${found})
    list(APPEND praxis_angle_exclusions_excusing ${excused})
    praxis_scan_angle_units("${path}" "${PRAXIS_ANGLE_TERMS}" "${PRAXIS_ANGLE_TYPES}" "${PRAXIS_ANGLE_EXCLUSIONS}"
                            "an angle that does not spell its unit" found excused)
    list(APPEND praxis_violations ${found})
    list(APPEND praxis_angle_exclusions_excusing ${excused})
endforeach ()

# An entry that silences no line the scan would otherwise report excuses nothing: it is dead weight
# exempting whatever stands beside it, and it stops the configure.
foreach (name IN LISTS PRAXIS_ANGLE_EXCLUSIONS)
    if (NOT name IN_LIST praxis_angle_exclusions_excusing)
        list(APPEND praxis_violations "${name}: excluded from the angle-unit scan but silencing no line it would report")
    endif ()
endforeach ()

set(praxis_slot_tables "")
foreach (module IN LISTS PRAXIS_EXTENSIONS)
    praxis_slot_table_group("${module}" group)
    list(APPEND praxis_slot_tables ${group})
endforeach ()

foreach (path IN LISTS praxis_slot_tables)
    if (NOT EXISTS "${PRAXIS_SOURCE_DIR}/${path}")
        list(APPEND praxis_violations "${path}: an extension declares no slot table under the name the scan reads")
        continue()
    endif ()
    praxis_scan_scheduling("${path}" "${PRAXIS_SCHEDULING_TERMS}" found)
    list(APPEND praxis_violations ${found})
endforeach ()


file(GLOB praxis_modules RELATIVE "${PRAXIS_SOURCE_DIR}/lib" "${PRAXIS_SOURCE_DIR}/lib/*")
foreach (entry IN LISTS praxis_modules)
    if (entry MATCHES "^\\." OR NOT IS_DIRECTORY "${PRAXIS_SOURCE_DIR}/lib/${entry}")
        continue()
    elseif (NOT entry IN_LIST PRAXIS_MODULE_DIRECTORIES)
        list(APPEND praxis_violations "lib/${entry}/: not one of the repository's module directories")
    endif ()
endforeach ()

foreach (entry IN LISTS PRAXIS_MODULE_DIRECTORIES)
    if (NOT IS_DIRECTORY "${PRAXIS_SOURCE_DIR}/lib/${entry}")
        list(APPEND praxis_violations "lib/${entry}/: named by the umbrella listfile but absent from the tree")
    endif ()
endforeach ()

if (praxis_violations)
    list(LENGTH praxis_violations count)
    list(JOIN praxis_violations "\n  " report)
    message(FATAL_ERROR "praxis: ${count} convention violation(s):\n  ${report}")
endif ()
