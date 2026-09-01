praxis_add_module(scene
    SOURCES
    include/praxis/scene/rates.h
    include/praxis/scene/preset.h
    include/praxis/scene/stencil.h
    include/praxis/scene/widgets.h
    include/praxis/scene/log_buffer.h
    include/praxis/scene/plot_window.h
    include/praxis/scene/visualizer.h
    include/praxis/scene/composition.h
    include/praxis/scene/preset_site.h
    include/praxis/scene/imgui_window.h
    include/praxis/scene/preset_registry.h
    include/praxis/scene/coverage_report.h
    include/praxis/scene/labeled_value_window.h
    src/preset.cpp
    src/log_buffer.cpp
    src/visualizer.cpp
    src/composition.cpp
    src/plot_window.cpp
    src/imgui_window.cpp
    src/preset_registry.cpp
    src/coverage_report.cpp
    src/visualizer_scene.cpp
    src/labeled_value_window.cpp
    src/windows/log_window.h
    src/windows/log_window.cpp
    src/windows/preset_window.h
    src/windows/preset_window.cpp
    src/windows/view_gizmo_window.h
    src/windows/view_gizmo_window.cpp
    src/windows/stepped_work_window.h
    src/windows/stepped_work_window.cpp
    PUBLIC_LIBS
    praxis::extension
    praxis::scheduler
    praxis::config
    imgui
    threepp::threepp
    Eigen3::Eigen
    PRIVATE_LIBS
    glfw
    implot
    spdlog::spdlog
)

# The renderer and the GUI library are published surface here and are therefore absent from the list.
praxis_assert_no_include_leak(praxis_scene cartan ctrlpp spdlog meios)

set(PRAXIS_PUBLIC_DEPENDENCIES imgui threepp Eigen3)

# The core names no domain, so the solver, the control library and the model library stay out of
# this closure entirely; the link-isolation gate reads the list and fails on any of them reaching it.
set(PRAXIS_CLOSURE_ABSENT cartan ctrlpp meios)

# NAMESPACE prefixes the target name rather than the alias, so without EXPORT_NAME a consumer
# writing praxis::scene is told about a target it never named.
set_target_properties(praxis_scene PROPERTIES EXPORT_NAME scene)
