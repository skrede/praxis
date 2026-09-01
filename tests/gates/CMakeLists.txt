# The gate reconfigures whatever build directory it is given, and it cannot run inside the configure
# that would launch it. It is given a scratch directory of its own: pointed at the caller's it would
# rewrite the tree it is reading.
add_test(NAME link_isolation_gate COMMAND ${CMAKE_COMMAND}
    -DPRAXIS_SOURCE_DIR=${PROJECT_SOURCE_DIR}
    -DPRAXIS_BUILD_DIR=${CMAKE_CURRENT_BINARY_DIR}/link-isolation
    -P ${PROJECT_SOURCE_DIR}/cmake/verify_link_isolation.cmake)

# This one reconfigures nothing: it reads the shipped headers and compares what it extracts against
# the snapshot, so the extraction is all it writes. It is the only gate needing Universal Ctags, and
# where its option is on it is registered whether or not the tool is present, because an unregistered
# test is silent in the report and a report that says nothing reads as a report that found nothing.
if (PRAXIS_ENABLE_CTAGS_GATES)
    add_test(NAME public_surface_gate COMMAND ${CMAKE_COMMAND}
        -DPRAXIS_SOURCE_DIR=${PROJECT_SOURCE_DIR}
        -DPRAXIS_BUILD_DIR=${CMAKE_CURRENT_BINARY_DIR}
        -P ${PROJECT_SOURCE_DIR}/cmake/verify_public_surface.cmake)
endif ()
