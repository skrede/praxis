add_executable(rigid_demo
    ../main.cpp

    ../demo_documents.h
    ../demo_documents.cpp
    ../demo_write_back.h
    ../demo_write_back.cpp
    ../demo_configuration.h
    ../demo_configuration.cpp

    ../demo_offered.h
    rigid_offered.cpp
)

set(PRAXIS_RIGID_DEMO_CONFIGURATION "${CMAKE_CURRENT_SOURCE_DIR}/praxis-rigid.xml" CACHE FILEPATH
    "The configuration document the rigid-motion demonstration reads at start-up")

praxis_target_warnings(rigid_demo)
target_compile_features(rigid_demo PRIVATE cxx_std_20)
target_link_libraries(rigid_demo PRIVATE praxis::praxis praxis::rigid_presets praxis::rigid_motion praxis::config spdlog::spdlog)
target_include_directories(rigid_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_compile_definitions(rigid_demo PRIVATE PRAXIS_DEMO_CONFIGURATION="${PRAXIS_RIGID_DEMO_CONFIGURATION}")
praxis_target_persistent_directory(rigid_demo DIRECTORY "${PRAXIS_PERSISTENT_DIRECTORY}/rigid")
