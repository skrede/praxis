add_library(praxis_rigid_presets)
add_library(praxis::rigid_presets ALIAS praxis_rigid_presets)

add_library(praxis_manipulator_presets)
add_library(praxis::manipulator_presets ALIAS praxis_manipulator_presets)

praxis_target_warnings(praxis_rigid_presets)
praxis_target_warnings(praxis_manipulator_presets)

target_compile_features(praxis_rigid_presets PUBLIC cxx_std_20)
target_compile_features(praxis_manipulator_presets PUBLIC cxx_std_20)

set_target_properties(praxis_rigid_presets PROPERTIES POSITION_INDEPENDENT_CODE ON EXPORT_NAME rigid_presets)
set_target_properties(praxis_manipulator_presets PROPERTIES POSITION_INDEPENDENT_CODE ON EXPORT_NAME manipulator_presets)

target_include_directories(praxis_rigid_presets PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(praxis_manipulator_presets PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# The logging library is a private link of the extensions a scenario composes from, so it does not
# arrive transitively.
target_link_libraries(praxis_rigid_presets PUBLIC praxis::praxis praxis::rigid_motion praxis::config
    PRIVATE spdlog::spdlog)
target_link_libraries(praxis_manipulator_presets PUBLIC praxis::praxis praxis::rigid_motion praxis::trajectory praxis::manipulator praxis::config
    PRIVATE spdlog::spdlog)

target_sources(praxis_rigid_presets PRIVATE include/praxis/presets/screw.h include/praxis/presets/euler_rungs.h
    include/praxis/presets/arrangements.h include/praxis/presets/rotation_axis.h include/praxis/presets/frame_workbench.h
    include/praxis/presets/twist_axis.h include/praxis/presets/two_pose.h
    include/praxis/presets/arrangement_scenarios.h)

target_sources(praxis_manipulator_presets PRIVATE include/praxis/presets/arm.h include/praxis/presets/screw_table.h)

add_subdirectory(arm)
add_subdirectory(screw)
add_subdirectory(euler_rungs)
add_subdirectory(rotation_axis)
add_subdirectory(twist_axis)
add_subdirectory(two_pose)
add_subdirectory(frame_workbench)
add_subdirectory(arrangements)

praxis_assert_no_include_leak(praxis_rigid_presets cartan ctrlpp spdlog)
praxis_assert_no_include_leak(praxis_manipulator_presets cartan ctrlpp spdlog)
