#include "configuration_keys.h"

#include "praxis/manipulator/trajectory_configuration.h"

#include <string>
#include <vector>
#include <string_view>

namespace praxis::manipulator {

namespace {

struct preview_names
{
    static constexpr std::string_view rate        = "rate";
    static constexpr std::string_view parameter   = "parameter";
    static constexpr std::string_view rate_change = "rate_change";
};

}

void declare_trajectory_preview(config::declaration &shape, std::string_view at)
{
    const trajectory_preview_window::settings was;

    shape.group(std::string(at));
    shape.field(keys::under(at, preview_names::rate), config::field_kind::flag, was.rate ? "true" : "false");
    shape.field(keys::under(at, preview_names::parameter), config::field_kind::flag, was.parameter ? "true" : "false");
    shape.field(keys::under(at, preview_names::rate_change), config::field_kind::flag, was.rate_change ? "true" : "false");
}

trajectory_preview_window::settings read_trajectory_preview(const config::document &values, std::string_view at)
{
    const trajectory_preview_window::settings was;

    return trajectory_preview_window::settings{keys::flag_at(values, keys::under(at, preview_names::parameter), was.parameter),
                                               keys::flag_at(values, keys::under(at, preview_names::rate), was.rate),
                                               keys::flag_at(values, keys::under(at, preview_names::rate_change), was.rate_change)};
}

std::vector<config::edit> write_trajectory_preview(const trajectory_preview_window::settings &state, std::string_view at)
{
    return std::vector<config::edit>{
            config::edit{keys::under(at, preview_names::rate), state.rate ? "true" : "false"},
            config::edit{keys::under(at, preview_names::parameter), state.parameter ? "true" : "false"},
            config::edit{keys::under(at, preview_names::rate_change), state.rate_change ? "true" : "false"},
    };
}

}
