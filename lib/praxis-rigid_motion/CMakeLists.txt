praxis_add_module(rigid_motion
    SOURCES
    include/praxis/rigid_motion.h
    include/praxis/rigid_motion/axes.h
    include/praxis/rigid_motion/types.h
    include/praxis/rigid_motion/frame.h
    include/praxis/rigid_motion/screw.h
    include/praxis/rigid_motion/slots.h
    include/praxis/rigid_motion/angles.h
    include/praxis/rigid_motion/axis_order.h
    include/praxis/rigid_motion/evaluation.h
    include/praxis/rigid_motion/frame_tree.h
    include/praxis/rigid_motion/capabilities.h
    include/praxis/rigid_motion/frame_window.h
    include/praxis/rigid_motion/screw_window.h
    include/praxis/rigid_motion/configuration.h
    include/praxis/rigid_motion/frame_stencil.h
    include/praxis/rigid_motion/screw_widgets.h
    include/praxis/rigid_motion/matrix_readout.h
    include/praxis/rigid_motion/two_pose_window.h
    include/praxis/rigid_motion/twist_axis_window.h
    include/praxis/rigid_motion/frame_roster_window.h
    include/praxis/rigid_motion/rotation_axis_window.h
    include/praxis/rigid_motion/decoupled_path_window.h
    include/praxis/rigid_motion/frame_selector_window.h
    include/praxis/rigid_motion/baseline/frame.h
    include/praxis/rigid_motion/baseline/screw.h
    src/axes.cpp
    src/axis_order.cpp
    src/evaluation.cpp
    src/descriptors.cpp
    src/inert_frame.cpp
    src/inert_screw.cpp
    src/frame_tree.cpp
    src/frame_window.cpp
    src/frame_roster_window.cpp
    src/frame_selector_window.cpp
    src/frame_window_arrangement.cpp
    src/screw_window.cpp
    src/twist_axis_window.cpp
    src/rotation_axis_window.cpp
    src/two_pose_window.cpp
    src/two_pose_window_panel.cpp
    src/decoupled_path_window.cpp
    src/frame_stencil.cpp
    src/frame_stencil_marker.cpp
    src/configuration.cpp
    src/matrix_readout.cpp
    src/configuration_keys.h
    src/path_sampling.h
    src/evaluation_tables.h
    src/evaluation_frame.cpp
    src/evaluation_screw.cpp
    src/configuration_keys.cpp
    src/evaluation_screw_exponential.cpp
    src/baseline/frame.cpp
    src/baseline/screw.cpp
    src/baseline/capabilities.cpp
    PUBLIC_LIBS
    praxis::extension
    praxis::evaluation
    praxis::config
    praxis::scene
    Eigen3::Eigen
    PRIVATE_LIBS
    cartan::lie
    spdlog::spdlog
)

# The renderer and the GUI library arrive with the scene this extension's stencil and window build
# on, so both are published surface here and neither is on the list. cartan and the logging library
# are on it even though this module links them: the assertion is not that the dependency is absent
# but that the private link never reaches the published interface, and the same holds for the control
# and model dependencies, which this module never links at all.
praxis_assert_no_include_leak(praxis_rigid_motion cartan ctrlpp meios spdlog)

set(PRAXIS_PUBLIC_DEPENDENCIES Eigen3)

# Rigid-body mathematics needs neither the control library nor a robot description; the
# link-isolation gate reads this list and fails on either reaching this target's closure.
set(PRAXIS_CLOSURE_ABSENT ctrlpp meios)

# NAMESPACE prefixes the target name rather than the alias, so without EXPORT_NAME a consumer
# writing praxis::rigid_motion is told about a target it never named.
set_target_properties(praxis_rigid_motion PROPERTIES EXPORT_NAME rigid_motion)
