#include "configuration_keys.h"

#include "praxis/manipulator/view_configuration.h"

#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <string_view>

namespace praxis::manipulator {

namespace {

// A reach of no length is no reach at all, so the leaf spells absence with it: the decoration then
// keeps the reach it opened at, which is proportioned to the rendered arm's own size.
constexpr float unnamed_reach = 0.f;

struct robot_view_names
{
    static constexpr std::string_view model      = "model";
    static constexpr std::string_view reach      = "axis_reach";
    static constexpr std::string_view decoration = "screw_axes";
};

const robot_view_window::settings &robot_view_fallbacks()
{
    static const robot_view_window::settings state;
    return state;
}

std::string model_text(model_render value)
{
    return std::string(model_render_labels()[static_cast<std::size_t>(value)]);
}

std::string reach_text(const std::optional<double> &reach)
{
    return keys::text_of(static_cast<float>(reach.value_or(unnamed_reach)));
}

std::optional<double> reach_of(float read)
{
    return read > unnamed_reach ? std::optional<double>(read) : std::nullopt;
}

}

void declare_robot_view(config::declaration &shape, std::string_view at)
{
    const robot_view_window::settings &was = robot_view_fallbacks();

    shape.group(std::string(at));
    shape.choice(keys::under(at, robot_view_names::model), keys::spelled(model_render_labels()), model_text(was.model));
    shape.field(keys::under(at, robot_view_names::decoration), config::field_kind::flag, was.decoration ? "true" : "false");
    shape.field(keys::under(at, robot_view_names::reach), config::field_kind::real, reach_text(was.axis_reach));
}

robot_view_window::settings read_robot_view(const config::document &values, std::string_view at)
{
    const robot_view_window::settings &was = robot_view_fallbacks();

    robot_view_window::settings state;
    state.model      = static_cast<model_render>(keys::indexed(values, keys::under(at, robot_view_names::model), model_render_labels(), static_cast<std::size_t>(was.model)));
    state.decoration = keys::flag_at(values, keys::under(at, robot_view_names::decoration), was.decoration);
    state.axis_reach = reach_of(keys::real_at(values, keys::under(at, robot_view_names::reach), static_cast<float>(was.axis_reach.value_or(unnamed_reach))));

    return state;
}

std::vector<config::edit> write_robot_view(const robot_view_window::settings &state, std::string_view at)
{
    std::vector<config::edit> changes;
    changes.push_back(config::edit{keys::under(at, robot_view_names::model), model_text(state.model)});
    changes.push_back(config::edit{keys::under(at, robot_view_names::decoration), state.decoration ? "true" : "false"});
    changes.push_back(config::edit{keys::under(at, robot_view_names::reach), reach_text(state.axis_reach)});

    return changes;
}

}
