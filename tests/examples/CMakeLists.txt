# The demonstration's own sources carry no interface and link without the applications around them,
# so a suite compiles them beside its cases the way a module's suite compiles the module. A source is
# named relative to this directory, which is what the declared-source gate resolves against.
praxis_add_test(demo_documents_test.cpp HEADERS scratch_documents.h LIBS praxis::config spdlog::spdlog)
target_sources(demo_documents_test PRIVATE ../../examples/demo_documents.cpp)
target_include_directories(demo_documents_test PRIVATE ${PROJECT_SOURCE_DIR}/examples)

praxis_add_test(demo_write_back_test.cpp HEADERS scratch_documents.h LIBS praxis::scene praxis::config spdlog::spdlog)
target_sources(demo_write_back_test PRIVATE ../../examples/demo_documents.cpp ../../examples/demo_write_back.cpp)
target_include_directories(demo_write_back_test PRIVATE ${PROJECT_SOURCE_DIR}/examples)

praxis_add_test(shipped_presets_test.cpp LIBS praxis::scene praxis::manipulator_presets praxis::manipulator praxis::rigid_motion praxis::trajectory praxis::config spdlog::spdlog)
target_sources(shipped_presets_test PRIVATE ../../examples/demo_documents.cpp ../../examples/demo_write_back.cpp ../../examples/demo_configuration.cpp
    ../../examples/demo_machine_shape.cpp ../../examples/demo_machine_windows.cpp)
target_include_directories(shipped_presets_test PRIVATE ${PROJECT_SOURCE_DIR}/examples)

# The documents the demonstration ships sit beside its sources, which is where the application
# resolves them from too.
target_compile_definitions(shipped_presets_test PRIVATE PRAXIS_SHIPPED_MACHINE_DIR="${PROJECT_SOURCE_DIR}/examples/manipulator")

praxis_add_test(demo_machine_test.cpp HEADERS scratch_documents.h LIBS praxis::scene praxis::manipulator_presets praxis::manipulator praxis::rigid_motion praxis::trajectory praxis::config spdlog::spdlog)
target_sources(demo_machine_test PRIVATE ../../examples/demo_documents.cpp ../../examples/demo_write_back.cpp ../../examples/demo_configuration.cpp ../../examples/demo_machine.cpp
    ../../examples/demo_machine_shape.cpp ../../examples/demo_machine_windows.cpp)
target_include_directories(demo_machine_test PRIVATE ${PROJECT_SOURCE_DIR}/examples)

praxis_add_test(window_keyspace_test.cpp HEADERS scratch_documents.h LIBS praxis::scene praxis::manipulator_presets praxis::manipulator praxis::rigid_motion praxis::trajectory praxis::config spdlog::spdlog)
target_sources(window_keyspace_test PRIVATE ../../examples/demo_documents.cpp ../../examples/demo_write_back.cpp ../../examples/demo_configuration.cpp
    ../../examples/demo_machine_shape.cpp ../../examples/demo_machine_windows.cpp)
target_include_directories(window_keyspace_test PRIVATE ${PROJECT_SOURCE_DIR}/examples)

praxis_add_test(shipped_scenarios_test.cpp HEADERS scratch_documents.h LIBS praxis::scene praxis::manipulator_presets praxis::manipulator praxis::rigid_motion praxis::trajectory praxis::config praxis::scheduler spdlog::spdlog)
target_sources(shipped_scenarios_test PRIVATE ../../examples/demo_documents.cpp ../../examples/demo_write_back.cpp ../../examples/demo_configuration.cpp
    ../../examples/demo_machine.cpp ../../examples/demo_machine_shape.cpp ../../examples/demo_machine_windows.cpp ../../examples/demo_machines.cpp)
target_include_directories(shipped_scenarios_test PRIVATE ${PROJECT_SOURCE_DIR}/examples)
target_compile_definitions(shipped_scenarios_test PRIVATE PRAXIS_SHIPPED_MACHINE_DIR="${PROJECT_SOURCE_DIR}/examples/manipulator")

# The descriptions are deployed beside the demonstration executable, which a configure without the
# demonstration never creates; the case reading them skips when the directory it is pointed at is
# not there.
if (TARGET manipulator_demo)
    target_compile_definitions(shipped_scenarios_test PRIVATE PRAXIS_DEMO_RESOURCE_DIR="$<TARGET_FILE_DIR:manipulator_demo>/urdf")
endif ()

set_tests_properties(shipped_scenarios_test PROPERTIES TIMEOUT 60)
