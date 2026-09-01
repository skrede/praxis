praxis_add_test(shipped_preset_lifetime_test.cpp LIBS praxis::scene praxis::rigid_presets praxis::manipulator praxis::trajectory praxis::rigid_motion praxis_test_support)
praxis_add_test(frame_window_configuration_test.cpp HEADERS scratch_directory.h LIBS praxis::rigid_presets praxis::rigid_motion praxis::config)
praxis_add_test(screw_table_test.cpp HEADERS scratch_directory.h LIBS praxis::manipulator_presets praxis::manipulator praxis::rigid_motion praxis::config)
praxis_add_test(composed_windows_test.cpp HEADERS composed_panels.h described_arm.h scratch_directory.h LIBS praxis::rigid_presets praxis::manipulator_presets praxis::rigid_motion praxis::trajectory praxis::manipulator praxis::scene praxis_test_support)
praxis_add_test(arm_scenarios_test.cpp HEADERS opened_arm.h drawn_lines.h described_arm.h carried_models.h supplied_chain.h composed_panels.h scratch_directory.h ../support/drawn_chain.h LIBS praxis::manipulator_presets praxis::rigid_motion praxis::trajectory praxis::manipulator praxis::scene praxis_test_support)
praxis_add_test(arm_ik_scenarios_test.cpp HEADERS described_arm.h composed_panels.h scratch_directory.h LIBS praxis::manipulator_presets praxis::rigid_motion praxis::trajectory praxis::manipulator praxis::scene praxis::config praxis_test_support)
praxis_add_test(arm_trajectory_scenarios_test.cpp HEADERS drawn_lines.h described_arm.h composed_panels.h scratch_directory.h LIBS praxis::manipulator_presets praxis::rigid_motion praxis::trajectory praxis::manipulator praxis::scene praxis_test_support)
praxis_add_test(arm_comparison_scenario_test.cpp HEADERS drawn_lines.h described_arm.h composed_panels.h scratch_directory.h LIBS praxis::manipulator_presets praxis::rigid_motion praxis::trajectory praxis::manipulator praxis::scene praxis_test_support)
praxis_add_test(arm_tooling_test.cpp HEADERS opened_arm.h drawn_lines.h described_arm.h carried_models.h composed_panels.h scratch_directory.h LIBS praxis::manipulator_presets praxis::rigid_motion praxis::trajectory praxis::manipulator praxis::scene spdlog::spdlog praxis_test_support)
praxis_add_test(euler_rung_controls_test.cpp HEADERS opened_rung.h composed_panels.h labeled_panels.h LIBS praxis::rigid_presets praxis::rigid_motion praxis::scene praxis_test_support)
praxis_add_test(euler_rung_readouts_test.cpp HEADERS opened_rung.h composed_panels.h labeled_panels.h LIBS praxis::rigid_presets praxis::rigid_motion praxis::scene praxis_test_support)
praxis_add_test(frame_workbench_test.cpp HEADERS opened_workbench.h composed_panels.h labeled_panels.h LIBS praxis::rigid_presets praxis::rigid_motion praxis::scene praxis_test_support)
praxis_add_test(screw_preset_test.cpp HEADERS drawn_lines.h composed_panels.h LIBS praxis::rigid_presets praxis::rigid_motion praxis::scene praxis_test_support)
praxis_add_test(twist_axis_preset_test.cpp HEADERS drawn_lines.h composed_panels.h LIBS praxis::rigid_presets praxis::rigid_motion praxis::scene praxis_test_support)
praxis_add_test(rotation_axis_preset_test.cpp HEADERS drawn_lines.h labeled_panels.h composed_panels.h LIBS praxis::rigid_presets praxis::rigid_motion praxis::scene praxis_test_support)
praxis_add_test(two_pose_preset_test.cpp HEADERS drawn_lines.h composed_panels.h opened_two_pose.h substituted_rigid_motion.h LIBS praxis::rigid_presets praxis::rigid_motion praxis::scene praxis_test_support)
praxis_add_test(scenario_defaults_test.cpp LIBS praxis::manipulator_presets praxis::manipulator)
praxis_add_test(registered_presets_test.cpp HEADERS scratch_directory.h LIBS praxis::rigid_presets praxis::scene praxis::rigid_motion praxis::config)
praxis_add_test(seam_routing_test.cpp HEADERS substituted_rigid_motion.h LIBS praxis::scene praxis::manipulator praxis::trajectory praxis::rigid_motion praxis_test_support)
praxis_add_test(body_frame_routing_test.cpp HEADERS substituted_rigid_motion.h LIBS praxis::manipulator praxis::rigid_motion praxis_test_support)

# The descriptions are deployed beside the demonstration executable, which a configure without the
# demonstration never creates; the case reading them skips when the directory it is pointed at is
# not there, and the rest of the suite runs either way.
if (TARGET manipulator_demo)
    target_compile_definitions(arm_scenarios_test PRIVATE PRAXIS_DEMO_RESOURCE_DIR="$<TARGET_FILE_DIR:manipulator_demo>/urdf")
    target_compile_definitions(arm_tooling_test PRIVATE PRAXIS_DEMO_RESOURCE_DIR="$<TARGET_FILE_DIR:manipulator_demo>/urdf")
    target_compile_definitions(arm_comparison_scenario_test PRIVATE PRAXIS_DEMO_RESOURCE_DIR="$<TARGET_FILE_DIR:manipulator_demo>/urdf")
endif ()

# A case that drains a scheduler hangs rather than failing if a retirement stops being acknowledged.
set_tests_properties(shipped_preset_lifetime_test arm_scenarios_test arm_tooling_test arm_comparison_scenario_test PROPERTIES TIMEOUT 120)
