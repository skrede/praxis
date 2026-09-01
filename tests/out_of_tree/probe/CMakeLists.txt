cmake_minimum_required(VERSION 3.28)
project(praxis_out_of_tree_probe LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (NOT DEFINED PRAXIS_TARGETS)
    message(FATAL_ERROR "praxis: run with -DPRAXIS_TARGETS=<generated targets file>")
endif ()
if (NOT DEFINED PRAXIS_DEPENDENCIES)
    message(FATAL_ERROR "praxis: run with -DPRAXIS_DEPENDENCIES=<generated dependency targets file>")
endif ()

# The targets file references every dependency the modules link by its plain name without defining
# it, and stops on any that is missing, so they are acquired before it is read. The thread library
# arrives from a find module rather than from that file, so the consumer asks for it itself.
find_package(Threads REQUIRED)

include(${PRAXIS_DEPENDENCIES})
include(${PRAXIS_TARGETS})

add_executable(out_of_tree_probe
    main.cpp
    shipped_extension.h
    shipped_extension.cpp
)

target_link_libraries(out_of_tree_probe PRIVATE praxis::extension praxis::scheduler praxis::scene praxis::manipulator praxis::rigid_presets)
