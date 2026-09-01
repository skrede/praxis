#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_VIEW_CONFIGURATION_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_VIEW_CONFIGURATION_H

#include "praxis/manipulator/robot_view_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <vector>
#include <string_view>

namespace praxis::manipulator {

// `at` is a key path the caller owns, and the four leaves are declared beneath it. A document
// carrying some of them leaves the rest where the settings struct opens them.
void declare_robot_view(config::declaration &shape, std::string_view at);
robot_view_window::settings read_robot_view(const config::document &values, std::string_view at);
std::vector<config::edit> write_robot_view(const robot_view_window::settings &state, std::string_view at);

}

#endif
