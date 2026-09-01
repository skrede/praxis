#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_RENDER_CONFIGURATION_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_RENDER_CONFIGURATION_H

#include "praxis/manipulator/render_controls_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <vector>
#include <string_view>

namespace praxis::manipulator {

// `at` is a key path the caller owns. These declare their own leaves beneath it and declare nothing
// above it. A document carrying some of them leaves the rest where the settings struct opens them.
// A length at or below zero is read back as absent, so a value the document does not name and a
// value it names as nothing are the same thing: the stencil keeps whatever it opened at. Lengths
// are in drawn metres and the cut is a multiple of the block's own ellipsoid scale.
void declare_render_controls(config::declaration &shape, std::string_view at);
render_controls_window::settings read_render_controls(const config::document &values, std::string_view at);
std::vector<config::edit> write_render_controls(const render_controls_window::settings &state, std::string_view at);

}

#endif
