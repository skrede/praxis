add_executable(optimization_demo
    ../main.cpp

    ../demo_documents.h
    ../demo_documents.cpp
    ../demo_write_back.h
    ../demo_write_back.cpp
    ../demo_configuration.h
    ../demo_configuration.cpp

    ../demo_offered.h
    optimization_offered.cpp
)

set(PRAXIS_OPTIMIZATION_DEMO_CONFIGURATION "${CMAKE_CURRENT_SOURCE_DIR}/praxis-optimization.xml" CACHE FILEPATH
    "The configuration document the optimization demonstration reads at start-up")

praxis_target_warnings(optimization_demo)
target_compile_features(optimization_demo PRIVATE cxx_std_20)
target_link_libraries(optimization_demo PRIVATE praxis::praxis praxis::optimization_presets praxis::optimization praxis::config spdlog::spdlog)
target_include_directories(optimization_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_compile_definitions(optimization_demo PRIVATE PRAXIS_DEMO_CONFIGURATION="${PRAXIS_OPTIMIZATION_DEMO_CONFIGURATION}")
praxis_target_persistent_directory(optimization_demo DIRECTORY "${PRAXIS_PERSISTENT_DIRECTORY}/optimization")
