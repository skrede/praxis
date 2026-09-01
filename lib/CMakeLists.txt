# These two lists are the single authority for what a module is. The conventions gate and the
# link-isolation gate parse them out of this file rather than carrying a copy, and the tests drive
# their link lists from them, so adding or removing a module is one entry on one line here.
# The order is load-bearing: a module's include-leak assertion runs where that module is declared and
# walks the interface of every target it links, skipping a name that is not yet a target, so a module
# has to be added after everything it links.
set(PRAXIS_CORE_MODULES compat config extension scheduler scene evaluation CACHE INTERNAL "")
set(PRAXIS_EXTENSIONS rigid_motion trajectory manipulator CACHE INTERNAL "")

# A synchronization primitive is policed in every module the two lists above name, and these are the
# files inside them where one is a reviewed hand-off across a boundary rather than the discipline by
# which two strands share an object. The conventions gate reads the list here, and an entry naming a
# file that is gone or that no longer carries a primitive stops the configure rather than standing as
# a claim about a tree that does not exist. `EXCEPTIONS.md` carries the argument for each entry.
set(PRAXIS_SYNCHRONIZED_FILES
    lib/praxis-scene/include/praxis/scene/log_buffer.h
    lib/praxis-scene/src/log_buffer.cpp
    lib/praxis-scene/src/windows/log_window.cpp
    lib/praxis-scheduler/include/praxis/scheduler/snapshot.h
    lib/praxis-scheduler/src/pool.h
    lib/praxis-scheduler/src/pool.cpp
    lib/praxis-scheduler/src/pool_admission.cpp
    lib/praxis-scheduler/src/pool_receipts.cpp
    lib/praxis-scheduler/src/pool_service.cpp
    lib/praxis-scheduler/src/pool_waiting.cpp
    CACHE INTERNAL "")

set(praxis_core_aggregate "")
foreach (module IN LISTS PRAXIS_CORE_MODULES)
    add_subdirectory(praxis-${module})
    list(APPEND praxis_core_aggregate praxis::${module})
endforeach ()

set(praxis_extension_aggregate "")
foreach (module IN LISTS PRAXIS_EXTENSIONS)
    add_subdirectory(praxis-${module})
    list(APPEND praxis_extension_aggregate praxis::${module})
endforeach ()

# Every Foundation target and no extension, so no preset is compiled in by linking it.
add_library(praxis_all INTERFACE)
add_library(praxis::praxis ALIAS praxis_all)

target_link_libraries(praxis_all INTERFACE ${praxis_core_aggregate})

# Every extension this repository ships, each of which carries one preset, so linking this pulls in
# all of them; a consumer wanting one preset links that extension instead.
add_library(praxis_extensions INTERFACE)
add_library(praxis::extensions ALIAS praxis_extensions)

target_link_libraries(praxis_extensions INTERFACE ${praxis_extension_aggregate})

# NAMESPACE prefixes the target name rather than the alias, so without EXPORT_NAME a consumer
# writing praxis::praxis or praxis::extensions is told about a target it never named.
set_target_properties(praxis_all PROPERTIES EXPORT_NAME praxis)
set_target_properties(praxis_extensions PROPERTIES EXPORT_NAME extensions)
