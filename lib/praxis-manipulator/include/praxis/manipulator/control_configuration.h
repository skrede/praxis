#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_CONTROL_CONFIGURATION_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_CONTROL_CONFIGURATION_H

#include "praxis/manipulator/tool_jog_window.h"
#include "praxis/manipulator/robot_controller.h"
#include "praxis/manipulator/screw_jog_window.h"
#include "praxis/manipulator/task_space_window.h"
#include "praxis/manipulator/joint_control_window.h"
#include "praxis/manipulator/control_parameters_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <vector>
#include <string_view>

namespace praxis::manipulator {

// `at` is a key path the caller owns. Each of these declares its own leaves beneath it and declares
// nothing above it, so how many machines a composition carries and what keys them stays the
// composing side's choice. A read takes the same path with any collection already addressed.

void declare_joint_control(config::declaration &shape, std::string_view at);
joint_control_window::settings read_joint_control(const config::document &values, std::string_view at);
std::vector<config::edit> write_joint_control(const joint_control_window::settings &state, std::string_view at);

void declare_task_space(config::declaration &shape, std::string_view at);
task_space_window::settings read_task_space(const config::document &values, std::string_view at);
std::vector<config::edit> write_task_space(const task_space_window::settings &state, std::string_view at);

void declare_tool_jog(config::declaration &shape, std::string_view at);
tool_jog_window::settings read_tool_jog(const config::document &values, std::string_view at);
std::vector<config::edit> write_tool_jog(const tool_jog_window::settings &state, std::string_view at);

void declare_screw_jog(config::declaration &shape, std::string_view at);
screw_jog_window::settings read_screw_jog(const config::document &values, std::string_view at);
std::vector<config::edit> write_screw_jog(const screw_jog_window::settings &state, std::string_view at);

void declare_control_parameters(config::declaration &shape, std::string_view at);
control_parameters_window::settings read_control_parameters(const config::document &values, std::string_view at);
std::vector<config::edit> write_control_parameters(const control_parameters_window::settings &state, std::string_view at);

// The directory is carried as it was written and resolved nowhere here.
void declare_recording(config::declaration &shape, std::string_view at);
recording_parameters read_recording(const config::document &values, std::string_view at);
std::vector<config::edit> write_recording(const recording_parameters &state, std::string_view at);

}

#endif
