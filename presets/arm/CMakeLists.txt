target_sources(praxis_manipulator_presets PRIVATE frame_markers.h arm.cpp arm_forward.cpp arm_modeling.cpp arm_tooling.cpp arm_ik_windows.h arm_ik_windows.cpp arm_numerical_ik.cpp arm_analytic_ik.cpp
    arm_trajectory_windows.h arm_trajectory_windows.cpp arm_point_to_point.cpp arm_via_point.cpp arm_path_comparison.cpp arm_velocity.cpp
    screw_table.cpp screw_table_keys.h screw_table_keys.cpp
    arm_keys.h arm_keys.cpp arm_composers.h arm_keyspace.cpp arm_scenarios.cpp arm_document.cpp arm_registration.cpp
    arm_windows.h arm_windows_configuration.cpp)
