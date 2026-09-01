#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_VELOCITY_CONFIGURATION_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_VELOCITY_CONFIGURATION_H

#include "praxis/manipulator/velocity_kinematics_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <vector>
#include <string_view>

namespace praxis::manipulator {

// `at` is a key path the caller owns. These declare their own leaves beneath it and declare nothing
// above it. A document carrying some of them leaves the rest where the settings struct opens them.
void declare_velocity_kinematics(config::declaration &shape, std::string_view at);
velocity_kinematics_window::settings read_velocity_kinematics(const config::document &values, std::string_view at);
std::vector<config::edit> write_velocity_kinematics(const velocity_kinematics_window::settings &state, std::string_view at);

}

#endif
