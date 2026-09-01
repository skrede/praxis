#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_TRAJECTORY_CONFIGURATION_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_TRAJECTORY_CONFIGURATION_H

#include "praxis/manipulator/joint_curve_window.h"
#include "praxis/manipulator/path_comparison_window.h"
#include "praxis/manipulator/trajectory_preview_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <vector>
#include <cstddef>
#include <string_view>

namespace praxis::manipulator {

// `at` is a key path the caller owns. These declare their own leaves beneath it and declare nothing
// above it. A document carrying some of them leaves the rest where the settings struct opens them.
void declare_trajectory_preview(config::declaration &shape, std::string_view at);
trajectory_preview_window::settings read_trajectory_preview(const config::document &values, std::string_view at);
std::vector<config::edit> write_trajectory_preview(const trajectory_preview_window::settings &state, std::string_view at);

void declare_joint_curves(config::declaration &shape, std::string_view at);
joint_curve_window::settings read_joint_curves(const config::document &values, std::string_view at);
std::vector<config::edit> write_joint_curves(const joint_curve_window::settings &state, std::string_view at);

// Each end carries its joint values in radians, separated by single spaces. An end of a width other
// than `joints` is declined by name and the end beside it is still read; an end the document names
// no value for leaves the window opening at the pair its own opening answers.
void declare_path_comparison(config::declaration &shape, std::string_view at);
path_comparison_window::settings read_path_comparison(const config::document &values, std::string_view at, std::size_t joints);
std::vector<config::edit> write_path_comparison(const path_comparison_window::settings &state, std::string_view at);

}

#endif
