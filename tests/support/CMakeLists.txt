add_library(praxis_test_support INTERFACE)
target_sources(praxis_test_support INTERFACE answered.h held_home.h panel_keys.h drawn_chain.h imgui_frame.h captured_log.h panel_labels.h two_link_arm.h three_link_arm.h)
target_include_directories(praxis_test_support INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(praxis_test_support INTERFACE praxis::scene)
