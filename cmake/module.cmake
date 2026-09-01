function(praxis_add_module NAME)
    cmake_parse_arguments(ARG "" "" "SOURCES;PUBLIC_LIBS;PRIVATE_LIBS" ${ARGN})

    add_library(praxis_${NAME} ${ARG_SOURCES})
    add_library(praxis::${NAME} ALIAS praxis_${NAME})

    praxis_target_warnings(praxis_${NAME})
    target_compile_features(praxis_${NAME} PUBLIC cxx_std_20)
    set_target_properties(praxis_${NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

    target_include_directories(praxis_${NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    if (ARG_PUBLIC_LIBS)
        target_link_libraries(praxis_${NAME} PUBLIC ${ARG_PUBLIC_LIBS})
    endif ()
    if (ARG_PRIVATE_LIBS)
        target_link_libraries(praxis_${NAME} PRIVATE ${ARG_PRIVATE_LIBS})
    endif ()
endfunction()
