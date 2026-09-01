set(PRAXIS_PERSISTENT_DIRECTORY "${CMAKE_SOURCE_DIR}/.praxis" CACHE PATH
    "Where praxis keeps state it writes for itself, anchored in the source tree rather than a build tree")

function(praxis_target_persistent_directory TARGET)
    cmake_parse_arguments(ARG "" "DIRECTORY" "" ${ARGN})
    if (NOT ARG_DIRECTORY)
        set(ARG_DIRECTORY "${PRAXIS_PERSISTENT_DIRECTORY}")
    endif ()
    target_compile_definitions(${TARGET} PRIVATE PRAXIS_PERSISTENT_DIRECTORY="${ARG_DIRECTORY}")
endfunction()
