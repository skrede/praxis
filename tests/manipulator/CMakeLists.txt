praxis_add_test(types_test.cpp LIBS praxis::manipulator)
praxis_add_test(conversions_test.cpp LIBS praxis::manipulator)
praxis_add_test(kinematics_test.cpp HEADERS two_joint_bindings.h LIBS praxis::manipulator)
praxis_add_test(forward_kinematics_test.cpp HEADERS two_joint_bindings.h LIBS praxis::manipulator)
praxis_add_test(differential_kinematics_test.cpp HEADERS two_joint_bindings.h LIBS praxis::manipulator)
praxis_add_test(inverse_kinematics_test.cpp HEADERS two_joint_bindings.h recorded_chain.h LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(robot_test.cpp LIBS praxis::manipulator)
praxis_add_test(motion_test.cpp LIBS praxis::manipulator)
praxis_add_test(modeling_test.cpp LIBS praxis::manipulator)
praxis_add_test(coverage_test.cpp LIBS praxis::manipulator)
praxis_add_test(composition_test.cpp LIBS praxis::manipulator)
praxis_add_test(downstream_extension_test.cpp LIBS praxis::manipulator)
praxis_add_test(manipulator_baseline_test.cpp LIBS praxis::manipulator)
praxis_add_test(manipulator_partition_test.cpp LIBS praxis::manipulator)
praxis_add_test(baseline_kinematics_test.cpp LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(solver_convergence_test.cpp HEADERS recorded_chain.h LIBS praxis::manipulator praxis_test_support)
praxis_add_test(baseline_trajectory_test.cpp LIBS praxis::manipulator)
praxis_add_test(task_space_chain_test.cpp LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(baseline_robot_test.cpp LIBS praxis::manipulator praxis_test_support)
praxis_add_test(baseline_motion_test.cpp LIBS praxis::manipulator praxis_test_support)
praxis_add_test(option_cycle_test.cpp LIBS praxis::manipulator)
praxis_add_test(arm_snapshot_test.cpp HEADERS fixtures.h LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(published_manipulability_test.cpp HEADERS fixtures.h LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(published_iterates_test.cpp HEADERS fixtures.h LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(solution_publication_test.cpp HEADERS fixtures.h ../support/drawn_chain.h LIBS praxis::manipulator praxis::evaluation spdlog::spdlog praxis_test_support)
praxis_add_test(multi_start_test.cpp HEADERS fixtures.h LIBS praxis::manipulator praxis::evaluation spdlog::spdlog praxis_test_support)
praxis_add_test(scene_robot_test.cpp HEADERS fixtures.h LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(fatality_test.cpp HEADERS fixtures.h LIBS praxis::manipulator spdlog::spdlog)
praxis_add_test(refusal_standing_test.cpp HEADERS six_axis_machine.h LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(preset_composition_tolerance_test.cpp LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(compose_arm_joint_guard_test.cpp LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(motion_commands_test.cpp HEADERS fixtures.h LIBS praxis::manipulator praxis_test_support)
praxis_add_test(robot_controller_test.cpp HEADERS fixtures.h LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(trajectory_preview_test.cpp HEADERS fixtures.h LIBS praxis::manipulator praxis::evaluation spdlog::spdlog praxis_test_support)
target_include_directories(trajectory_preview_test PRIVATE ${PROJECT_SOURCE_DIR}/lib/praxis-manipulator/src)
praxis_add_test(window_lifetime_test.cpp HEADERS fixtures.h LIBS praxis::manipulator)
praxis_add_test(node_ownership_test.cpp HEADERS fixtures.h LIBS praxis::manipulator)
praxis_add_test(robot_stencil_models_test.cpp HEADERS fixtures.h LIBS praxis::manipulator)
praxis_add_test(robot_decoration_test.cpp HEADERS fixtures.h ../presets/drawn_lines.h ../support/drawn_chain.h LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(solution_figures_test.cpp HEADERS fixtures.h ../presets/drawn_lines.h ../support/drawn_chain.h LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(teardown_lifetime_test.cpp HEADERS fixtures.h LIBS praxis::manipulator)
praxis_add_test(tool_axes_test.cpp HEADERS fixtures.h LIBS praxis::manipulator)
praxis_add_test(tool_transform_guard_test.cpp HEADERS fixtures.h LIBS praxis::manipulator praxis_test_support)
praxis_add_test(screw_jog_value_test.cpp HEADERS fixtures.h window_stage.h LIBS praxis::manipulator praxis_test_support)
praxis_add_test(pose_orientation_value_test.cpp HEADERS fixtures.h window_stage.h LIBS praxis::manipulator praxis_test_support)
praxis_add_test(transform_window_angle_value_test.cpp HEADERS fixtures.h window_stage.h LIBS praxis::manipulator praxis_test_support)
praxis_add_test(trajectory_recording_window_test.cpp HEADERS fixtures.h LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(trajectory_recording_test.cpp HEADERS fixtures.h LIBS praxis::manipulator)
praxis_add_test(csv_write_outcome_test.cpp LIBS praxis::manipulator)
praxis_add_test(pose_readout_test.cpp LIBS praxis::manipulator praxis_test_support)
praxis_add_test(unpublished_arm_test.cpp HEADERS fixtures.h LIBS praxis::manipulator praxis_test_support)
praxis_add_test(edited_pose_test.cpp LIBS praxis::manipulator)
praxis_add_test(joint_control_window_test.cpp HEADERS fixtures.h window_stage.h LIBS praxis::manipulator praxis_test_support)
praxis_add_test(task_space_window_test.cpp HEADERS fixtures.h window_stage.h LIBS praxis::manipulator praxis_test_support)
praxis_add_test(tool_jog_window_test.cpp HEADERS fixtures.h window_stage.h LIBS praxis::manipulator praxis_test_support)
praxis_add_test(screw_jog_window_test.cpp HEADERS fixtures.h window_stage.h LIBS praxis::manipulator praxis_test_support)
praxis_add_test(idle_window_test.cpp HEADERS fixtures.h window_stage.h LIBS praxis::manipulator praxis_test_support)
praxis_add_test(ik_branch_window_test.cpp HEADERS fixtures.h window_stage.h ../support/drawn_chain.h LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(ik_seed_window_test.cpp HEADERS fixtures.h window_stage.h LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(edited_list_window_test.cpp HEADERS fixtures.h window_stage.h LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(ik_iterate_window_test.cpp HEADERS fixtures.h window_stage.h LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(ik_convergence_window_test.cpp HEADERS fixtures.h window_stage.h LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(screw_chain_builder_test.cpp LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(malformed_topology_test.cpp LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(unwalkable_topology_test.cpp LIBS praxis::manipulator praxis_test_support)
praxis_add_test(screw_chain_test.cpp HEADERS recorded_chain.h LIBS praxis::manipulator)
praxis_add_test(baseline_modeling_test.cpp HEADERS recorded_chain.h LIBS praxis::manipulator)
praxis_add_test(arm_configuration_test.cpp HEADERS fixtures.h window_stage.h LIBS praxis::manipulator praxis::config praxis_test_support)
praxis_add_test(kinematics_configuration_test.cpp LIBS praxis::manipulator praxis::config spdlog::spdlog praxis_test_support)
praxis_add_test(waypoints_configuration_test.cpp HEADERS fixtures.h window_stage.h LIBS praxis::manipulator praxis::config spdlog::spdlog praxis_test_support)
praxis_add_test(robot_view_window_test.cpp HEADERS fixtures.h window_stage.h ../presets/drawn_lines.h ../support/drawn_chain.h LIBS praxis::manipulator praxis::config praxis_test_support)
praxis_add_test(injected_chain_test.cpp HEADERS six_axis_machine.h LIBS praxis::manipulator)
praxis_add_test(screw_modeling_window_test.cpp HEADERS six_axis_machine.h ../presets/drawn_lines.h LIBS praxis::manipulator praxis::config spdlog::spdlog praxis_test_support)
praxis_add_test(pose_path_test.cpp HEADERS fixtures.h ../presets/drawn_lines.h LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(path_comparison_window_test.cpp HEADERS fixtures.h window_stage.h ../presets/drawn_lines.h LIBS praxis::manipulator praxis::trajectory spdlog::spdlog praxis_test_support)
praxis_add_test(render_configuration_test.cpp HEADERS fixtures.h window_stage.h drawn_columns.h drawn_ellipsoids.h velocity_kinematics_stage.h ../presets/drawn_lines.h ../presets/scratch_directory.h LIBS praxis::manipulator praxis::config spdlog::spdlog praxis_test_support)
praxis_add_test(render_controls_window_test.cpp HEADERS fixtures.h window_stage.h drawn_columns.h drawn_ellipsoids.h velocity_kinematics_stage.h ../presets/drawn_lines.h LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(velocity_configuration_test.cpp HEADERS fixtures.h window_stage.h drawn_columns.h drawn_ellipsoids.h velocity_kinematics_stage.h ../presets/drawn_lines.h ../presets/scratch_directory.h LIBS praxis::manipulator praxis::config spdlog::spdlog praxis_test_support)
praxis_add_test(velocity_kinematics_window_test.cpp HEADERS fixtures.h window_stage.h drawn_columns.h drawn_ellipsoids.h velocity_kinematics_stage.h ../presets/drawn_lines.h LIBS praxis::manipulator spdlog::spdlog praxis_test_support)
praxis_add_test(trajectory_preview_window_test.cpp HEADERS fixtures.h window_stage.h ../presets/drawn_lines.h LIBS praxis::manipulator praxis::evaluation spdlog::spdlog praxis_test_support)
praxis_add_test(joint_curve_window_test.cpp HEADERS fixtures.h window_stage.h LIBS praxis::manipulator praxis::trajectory spdlog::spdlog praxis_test_support)

# The fold and the case generator are headers the module keeps to itself, so the suites that hold
# them are given the module's own source root and nothing else of it.
praxis_add_test(chain_placement_test.cpp LIBS praxis::manipulator)
target_include_directories(chain_placement_test PRIVATE ${PROJECT_SOURCE_DIR}/lib/praxis-manipulator/src)

praxis_add_test(manipulability_test.cpp LIBS praxis::manipulator)
target_include_directories(manipulability_test PRIVATE ${PROJECT_SOURCE_DIR}/lib/praxis-manipulator/src)

praxis_add_test(ellipsoid_figure_test.cpp LIBS praxis::manipulator)
target_include_directories(ellipsoid_figure_test PRIVATE ${PROJECT_SOURCE_DIR}/lib/praxis-manipulator/src)

praxis_add_test(capped_body_test.cpp LIBS praxis::manipulator)
target_include_directories(capped_body_test PRIVATE ${PROJECT_SOURCE_DIR}/lib/praxis-manipulator/src)

praxis_add_test(column_arrow_test.cpp LIBS praxis::manipulator)
target_include_directories(column_arrow_test PRIVATE ${PROJECT_SOURCE_DIR}/lib/praxis-manipulator/src)

praxis_add_test(ellipsoid_drawing_test.cpp HEADERS fixtures.h drawn_ellipsoids.h ../presets/drawn_lines.h ../support/drawn_chain.h LIBS praxis::manipulator praxis_test_support)
target_include_directories(ellipsoid_drawing_test PRIVATE ${PROJECT_SOURCE_DIR}/lib/praxis-manipulator/src)

praxis_add_test(ellipsoid_reading_test.cpp HEADERS fixtures.h drawn_ellipsoids.h ../presets/drawn_lines.h LIBS praxis::manipulator praxis_test_support)

praxis_add_test(ellipsoid_switch_test.cpp HEADERS fixtures.h window_stage.h drawn_columns.h drawn_ellipsoids.h velocity_kinematics_stage.h ../presets/drawn_lines.h LIBS praxis::manipulator spdlog::spdlog praxis_test_support)

praxis_add_test(column_arrow_drawing_test.cpp HEADERS fixtures.h drawn_columns.h drawn_ellipsoids.h ../presets/drawn_lines.h ../support/drawn_chain.h LIBS praxis::manipulator spdlog::spdlog praxis_test_support)

praxis_add_test(opw_geometry_test.cpp HEADERS recorded_chain.h LIBS praxis::manipulator cartan::analytical)
target_include_directories(opw_geometry_test PRIVATE ${PROJECT_SOURCE_DIR}/lib/praxis-manipulator/src)

praxis_add_test(evaluation_table_test.cpp HEADERS bent_manipulator.h bent_motions.h bend_kinds.h described_slots.h LIBS praxis::manipulator praxis::evaluation)
target_include_directories(evaluation_table_test PRIVATE ${PROJECT_SOURCE_DIR}/lib/praxis-manipulator/src)

praxis_add_test(evaluation_chain_test.cpp HEADERS bent_chain.h LIBS praxis::manipulator praxis::evaluation)
target_include_directories(evaluation_chain_test PRIVATE ${PROJECT_SOURCE_DIR}/lib/praxis-manipulator/src)

praxis_add_test(solve_row_target_test.cpp HEADERS solve_rows.h bent_solver.h LIBS praxis::manipulator praxis::evaluation)
target_include_directories(solve_row_target_test PRIVATE ${PROJECT_SOURCE_DIR}/lib/praxis-manipulator/src)

praxis_add_test(solve_row_bounds_test.cpp HEADERS solve_rows.h bent_solver.h LIBS praxis::manipulator praxis::evaluation)
target_include_directories(solve_row_bounds_test PRIVATE ${PROJECT_SOURCE_DIR}/lib/praxis-manipulator/src)

praxis_add_test(motion_row_bounds_test.cpp LIBS praxis::manipulator praxis::evaluation praxis::rigid_motion)
target_include_directories(motion_row_bounds_test PRIVATE ${PROJECT_SOURCE_DIR}/lib/praxis-manipulator/src)

praxis_add_test(empty_difference_test.cpp HEADERS solve_rows.h bent_solver.h bent_chain.h bent_manipulator.h bent_motions.h bend_kinds.h LIBS praxis::manipulator praxis::evaluation)
target_include_directories(empty_difference_test PRIVATE ${PROJECT_SOURCE_DIR}/lib/praxis-manipulator/src)

praxis_add_test(manipulator_bend_control_test.cpp HEADERS bent_manipulator.h bent_motions.h bend_kinds.h bend_probe.h described_slots.h LIBS praxis::manipulator praxis::evaluation)
target_include_directories(manipulator_bend_control_test PRIVATE ${PROJECT_SOURCE_DIR}/lib/praxis-manipulator/src)

praxis_add_test(manipulator_ledger_test.cpp HEADERS bent_manipulator.h bent_motions.h bend_kinds.h described_slots.h LIBS praxis::manipulator praxis::evaluation praxis::trajectory)

praxis_add_test(manipulator_bend_ledger_test.cpp HEADERS bent_manipulator.h bent_motions.h bend_kinds.h described_slots.h LIBS praxis::manipulator praxis::evaluation)

praxis_add_test(motion_rows_test.cpp HEADERS bent_manipulator.h bent_motions.h bend_kinds.h described_slots.h LIBS praxis::manipulator praxis::evaluation praxis::trajectory)
target_include_directories(motion_rows_test PRIVATE ${PROJECT_SOURCE_DIR}/lib/praxis-manipulator/src)

praxis_add_test(task_trajectory_row_test.cpp HEADERS bent_manipulator.h bent_motions.h bend_kinds.h LIBS praxis::manipulator praxis::evaluation praxis::trajectory)

# The descriptions are deployed beside the demonstration executable, which a configure without the
# demonstration never creates; a test reading them skips when the directory it is pointed at is not
# there.
if (TARGET manipulator_demo)
    foreach (reader screw_chain_test baseline_modeling_test chain_placement_test)
        target_compile_definitions(${reader} PRIVATE PRAXIS_DEMO_RESOURCE_DIR="$<TARGET_FILE_DIR:manipulator_demo>/urdf")
    endforeach ()
endif ()

# A Catch2 binary whose every case skipped exits 4 -- its "no tests ran" code -- which is a failure
# to ctest unless the run code is declared a skip. A run with one case executed exits 0 as usual, so
# this only ever fires for the configuration that deploys no descriptions.
set_tests_properties(screw_chain_test baseline_modeling_test chain_placement_test PROPERTIES SKIP_RETURN_CODE 4)

# A case that drains a scheduler hangs rather than failing if the gate stops posting.
set_tests_properties(arm_snapshot_test published_manipulability_test published_iterates_test solution_publication_test multi_start_test window_lifetime_test node_ownership_test robot_stencil_models_test robot_decoration_test solution_figures_test ellipsoid_drawing_test ellipsoid_reading_test ellipsoid_switch_test column_arrow_drawing_test teardown_lifetime_test tool_axes_test tool_transform_guard_test screw_jog_value_test pose_orientation_value_test transform_window_angle_value_test trajectory_recording_window_test trajectory_recording_test robot_controller_test preset_composition_tolerance_test compose_arm_joint_guard_test refusal_standing_test unpublished_arm_test joint_control_window_test task_space_window_test tool_jog_window_test screw_jog_window_test idle_window_test ik_branch_window_test ik_seed_window_test ik_iterate_window_test ik_convergence_window_test edited_list_window_test robot_view_window_test screw_modeling_window_test pose_path_test trajectory_preview_window_test joint_curve_window_test path_comparison_window_test velocity_kinematics_window_test velocity_configuration_test render_controls_window_test render_configuration_test PROPERTIES TIMEOUT 30)
