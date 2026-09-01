add_executable(manipulator_demo
    ../main.cpp

    ../demo_machine.h
    ../demo_machines.h
    ../demo_machine.cpp
    ../demo_machines.cpp
    ../demo_machine_shape.cpp
    ../demo_machine_windows.cpp
    ../demo_documents.h
    ../demo_documents.cpp
    ../demo_write_back.h
    ../demo_write_back.cpp
    ../demo_configuration.h
    ../demo_configuration.cpp

    ../demo_offered.h
    manipulator_offered.cpp
)

set(PRAXIS_MANIPULATOR_DEMO_CONFIGURATION "${CMAKE_CURRENT_SOURCE_DIR}/praxis-manipulator.xml" CACHE FILEPATH
    "The configuration document the manipulator demonstration reads at start-up")

praxis_target_warnings(manipulator_demo)
target_compile_features(manipulator_demo PRIVATE cxx_std_20)
target_link_libraries(manipulator_demo PRIVATE praxis::praxis praxis::manipulator_presets praxis::rigid_motion praxis::trajectory praxis::manipulator
    praxis::config spdlog::spdlog)
target_include_directories(manipulator_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_compile_definitions(manipulator_demo PRIVATE PRAXIS_DEMO_CONFIGURATION="${PRAXIS_MANIPULATOR_DEMO_CONFIGURATION}")
praxis_target_persistent_directory(manipulator_demo DIRECTORY "${PRAXIS_PERSISTENT_DIRECTORY}/manipulator")

# The deployment attaches to the executable, so the descriptions and meshes land beside the binary,
# which is where the entry point resolves them from.
meios_target_deploy_resources(manipulator_demo
    RESOURCES kuka_experimental
    PACKAGES kuka_kr6_support kuka_resources
    SUBDIR urdf
)

meios_target_deploy_resources(manipulator_demo
    RESOURCES ur_description
    PACKAGES urdf config meshes/ur3e
    SUBDIR urdf/ur_description
)
