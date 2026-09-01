#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_TOOL_CONFIGURATION_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_TOOL_CONFIGURATION_H

#include "praxis/manipulator/tool_window.h"
#include "praxis/manipulator/world_object_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <vector>
#include <string_view>

namespace praxis::manipulator {

void declare_tool(config::declaration &shape, std::string_view at);
tool_window::settings read_tool(const config::document &values, std::string_view at);
std::vector<config::edit> write_tool(const tool_window::settings &state, std::string_view at);

void declare_world_object(config::declaration &shape, std::string_view at);
world_object_window::settings read_world_object(const config::document &values, std::string_view at);
std::vector<config::edit> write_world_object(const world_object_window::settings &state, std::string_view at);

}

#endif
