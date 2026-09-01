cmake_minimum_required(VERSION 3.28)

if (NOT DEFINED PRAXIS_BUILD_DIR OR NOT DEFINED PRAXIS_TARGET OR NOT DEFINED PRAXIS_DIAGNOSTIC)
    message(FATAL_ERROR "praxis: run with -DPRAXIS_BUILD_DIR=<build> -DPRAXIS_TARGET=<target> -DPRAXIS_DIAGNOSTIC=<text>")
endif ()

execute_process(
    COMMAND ${CMAKE_COMMAND} --build "${PRAXIS_BUILD_DIR}" --target "${PRAXIS_TARGET}"
    RESULT_VARIABLE praxis_status
    OUTPUT_VARIABLE praxis_report
    ERROR_VARIABLE praxis_report
)

if (praxis_status EQUAL 0)
    message(FATAL_ERROR "praxis: ${PRAXIS_TARGET} compiled, so the rejection it probes is gone")
endif ()

# A target that does not exist, a listfile that no longer configures and a mistyped probe all fail the
# build as well, and none of them is the rejection under test.
if (NOT praxis_report MATCHES "${PRAXIS_DIAGNOSTIC}")
    message(FATAL_ERROR "praxis: ${PRAXIS_TARGET} failed without '${PRAXIS_DIAGNOSTIC}':\n${praxis_report}")
endif ()
