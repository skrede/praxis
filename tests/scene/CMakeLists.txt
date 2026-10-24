praxis_add_test(held_handle_test.cpp LIBS praxis::scene)
praxis_add_test(imgui_window_test.cpp LIBS praxis::scene)
praxis_add_test(imgui_ini_placement_test.cpp LIBS praxis::scene praxis_test_support)
praxis_add_test(opening_placement_test.cpp LIBS praxis::scene praxis_test_support)
praxis_add_test(log_buffer_test.cpp LIBS praxis::scene)
praxis_add_test(preset_lifetime_test.cpp LIBS praxis::scene)
praxis_add_test(unload_prompt_test.cpp HEADERS held_composition.h LIBS praxis::scene)
praxis_add_test(coverage_report_test.cpp LIBS praxis::scene)
praxis_add_test(preset_composition_test.cpp LIBS praxis::scene)
praxis_add_test(widget_callable_test.cpp LIBS praxis::scene praxis_test_support)
praxis_add_test(preset_registration_test.cpp LIBS praxis::scene)
praxis_add_test(labeled_value_window_test.cpp LIBS praxis::scene praxis_test_support)
praxis_add_test(plot_window_test.cpp LIBS praxis::scene implot praxis_test_support)
praxis_add_test(plot_pick_test.cpp LIBS praxis::scene implot praxis_test_support)
praxis_add_test(plot_legend_test.cpp LIBS praxis::scene implot praxis_test_support)

set_tests_properties(preset_lifetime_test preset_composition_test PROPERTIES TIMEOUT 30)

# The canvas's hidden-window parameter hides a window on a machine that has a display; it does not
# remove the display requirement, because the renderer needs a graphics context the null platform
# does not supply.
if (PRAXIS_BUILD_DISPLAY_TESTS)
    praxis_add_display_test(display_teardown_test.cpp HEADERS scratch_rooted.h ../presets/scratch_directory.h LIBS praxis::scene)
    praxis_add_display_test(display_visualizer_test.cpp HEADERS scratch_rooted.h ../presets/scratch_directory.h LIBS praxis::scene)
    praxis_add_display_test(display_plot_window_test.cpp HEADERS scratch_rooted.h ../presets/scratch_directory.h LIBS praxis::scene)
    praxis_add_display_test(display_opening_view_test.cpp LIBS praxis::scene)
endif ()
