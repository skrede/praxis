cmake_minimum_required(VERSION 3.28)
project(praxis_consumability_consumer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (NOT DEFINED PRAXIS_REPOSITORY)
    message(FATAL_ERROR "praxis: no repository address given; run with -DPRAXIS_REPOSITORY=<address> -DPRAXIS_REFERENCE=<reference>")
endif ()
if (NOT DEFINED PRAXIS_REFERENCE)
    message(FATAL_ERROR "praxis: no reference to acquire given; run with -DPRAXIS_REPOSITORY=<address> -DPRAXIS_REFERENCE=<reference>")
endif ()

include(FetchContent)

FetchContent_Declare(
    praxis
    GIT_REPOSITORY ${PRAXIS_REPOSITORY}
    GIT_TAG ${PRAXIS_REFERENCE}
)
FetchContent_MakeAvailable(praxis)

add_executable(consumability_consumer
    main.cpp
)

target_link_libraries(consumability_consumer PRIVATE praxis::rigid_motion)
