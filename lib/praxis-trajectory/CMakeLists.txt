praxis_add_module(trajectory
    SOURCES
    include/praxis/trajectory.h
    include/praxis/trajectory/path.h
    include/praxis/trajectory/types.h
    include/praxis/trajectory/slots.h
    include/praxis/trajectory/evaluation.h
    include/praxis/trajectory/trajectory.h
    include/praxis/trajectory/capabilities.h
    include/praxis/trajectory/time_scaling.h
    include/praxis/trajectory/pose_trajectory.h
    include/praxis/trajectory/baseline/path.h
    include/praxis/trajectory/baseline/trajectory.h
    include/praxis/trajectory/baseline/time_scaling.h
    include/praxis/trajectory/baseline/pose_trajectory.h
    include/praxis/trajectory/detail/finite_difference.h
    src/descriptors.cpp
    src/evaluation.cpp
    src/evaluation_cases.h
    src/evaluation_path.cpp
    src/evaluation_tables.h
    src/evaluation_cases.cpp
    src/evaluation_driving.h
    src/evaluation_driving.cpp
    src/evaluation_generators.cpp
    src/evaluation_time_scaling.cpp
    src/evaluation_waypoint_cases.cpp
    src/inert_path.cpp
    src/inert_trajectory.cpp
    src/inert_time_scaling.cpp
    src/inert_pose_trajectory.cpp
    src/baseline/path.cpp
    src/baseline/trajectory.cpp
    src/baseline/capabilities.cpp
    src/baseline/time_scaling.cpp
    src/baseline/swept_pose.h
    src/baseline/swept_pose.cpp
    src/baseline/straight_line.h
    src/baseline/straight_line.cpp
    src/baseline/pose_trajectory.cpp
    PUBLIC_LIBS
    praxis::extension
    praxis::evaluation
    praxis::scene
    praxis::rigid_motion
    Eigen3::Eigen
    PRIVATE_LIBS
    ctrlpp::ctrlpp
    spdlog::spdlog
)

# The renderer and the GUI library arrive with the scene this extension's preset composes against, so
# both are published surface here and neither is on the list. ctrlpp is on it even though this module
# links it: the assertion is not that the dependency is absent but that the private link never
# reaches the published interface, and the same holds for the solver, which arrives privately through
# the extension below, and for the model and logging dependencies.
praxis_assert_no_include_leak(praxis_trajectory cartan ctrlpp meios spdlog)

set(PRAXIS_PUBLIC_DEPENDENCIES Eigen3)

# A trajectory is drawn through a configuration space, not from a robot description; the
# link-isolation gate reads this list and fails on the model library reaching this target's closure.
set(PRAXIS_CLOSURE_ABSENT meios)

# NAMESPACE prefixes the target name rather than the alias, so without EXPORT_NAME a consumer
# writing praxis::trajectory is told about a target it never named.
set_target_properties(praxis_trajectory PROPERTIES EXPORT_NAME trajectory)
