#include "configuration_keys.h"

#include "praxis/manipulator/control_configuration.h"

#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <string_view>

namespace praxis::manipulator {

namespace {

constexpr std::array<const char *, 2> motion_shapes{"ptp", "lin"};

struct control_names
{
    static constexpr std::string_view shape       = "shape";
    static constexpr std::string_view velocity    = "velocity_factor";
    static constexpr std::string_view scaling     = "time_scaling";
    static constexpr std::string_view rate        = "path_rate";
    static constexpr std::string_view rate_change = "path_rate_change";
};

// A rate at or below zero bounds no motion, so it is how a document says the trapezoid takes what
// the arm's limits give rather than an override.
constexpr float no_override = 0.f;

std::optional<path_parameter_bounds> bounds_at(const config::document &values, std::string_view at)
{
    const float rate        = keys::real_at(values, keys::under(at, control_names::rate), no_override);
    const float rate_change = keys::real_at(values, keys::under(at, control_names::rate_change), no_override);
    if(!(rate > 0.f) || !(rate_change > 0.f))
        return std::nullopt;

    return path_parameter_bounds{static_cast<double>(rate), static_cast<double>(rate_change)};
}

time_scaling_choice scaling_at(const config::document &values, std::string_view at, time_scaling_choice fallback)
{
    const std::size_t named = keys::indexed(values, keys::under(at, control_names::scaling), time_scaling_labels, static_cast<std::size_t>(fallback));

    return time_scaling_options[named];
}

struct recording_names
{
    static constexpr std::string_view active    = "active";
    static constexpr std::string_view directory = "directory";
};

}

void declare_joint_control(config::declaration &shape, std::string_view at)
{
    const joint_control_window::settings was;

    shape.group(std::string(at));
    keys::declare_mode(shape, at, was.mode);
}

joint_control_window::settings read_joint_control(const config::document &values, std::string_view at)
{
    const joint_control_window::settings was;

    return joint_control_window::settings{keys::read_mode(values, at, was.mode)};
}

std::vector<config::edit> write_joint_control(const joint_control_window::settings &state, std::string_view at)
{
    return std::vector<config::edit>{keys::written_mode(state.mode, at)};
}

void declare_task_space(config::declaration &shape, std::string_view at)
{
    const task_space_window::settings was;

    shape.group(std::string(at));
    shape.choice(keys::under(at, control_names::shape), keys::spelled(motion_shapes), motion_shapes[static_cast<std::size_t>(was.shape)]);
    keys::declare_mode(shape, at, was.mode);
}

task_space_window::settings read_task_space(const config::document &values, std::string_view at)
{
    task_space_window::settings state;
    state.shape = static_cast<task_space_window::motion_shape>(keys::indexed(values, keys::under(at, control_names::shape), motion_shapes, static_cast<std::size_t>(state.shape)));
    state.mode  = keys::read_mode(values, at, state.mode);
    return state;
}

std::vector<config::edit> write_task_space(const task_space_window::settings &state, std::string_view at)
{
    return std::vector<config::edit>{
            config::edit{keys::under(at, control_names::shape), motion_shapes[static_cast<std::size_t>(state.shape)]},
            keys::written_mode(state.mode, at),
    };
}

void declare_tool_jog(config::declaration &shape, std::string_view at)
{
    const tool_jog_window::settings was;

    shape.group(std::string(at));
    keys::declare_mode(shape, at, was.mode);
}

tool_jog_window::settings read_tool_jog(const config::document &values, std::string_view at)
{
    const tool_jog_window::settings was;

    return tool_jog_window::settings{keys::read_mode(values, at, was.mode)};
}

std::vector<config::edit> write_tool_jog(const tool_jog_window::settings &state, std::string_view at)
{
    return std::vector<config::edit>{keys::written_mode(state.mode, at)};
}

void declare_screw_jog(config::declaration &shape, std::string_view at)
{
    const screw_jog_window::settings was;

    shape.group(std::string(at));
    keys::declare_mode(shape, at, was.mode);
}

screw_jog_window::settings read_screw_jog(const config::document &values, std::string_view at)
{
    const screw_jog_window::settings was;

    return screw_jog_window::settings{keys::read_mode(values, at, was.mode)};
}

std::vector<config::edit> write_screw_jog(const screw_jog_window::settings &state, std::string_view at)
{
    return std::vector<config::edit>{keys::written_mode(state.mode, at)};
}

void declare_control_parameters(config::declaration &shape, std::string_view at)
{
    const control_parameters_window::settings was;

    shape.group(std::string(at));
    shape.field(keys::under(at, control_names::velocity), config::field_kind::real, keys::text_of(was.velocity));
    shape.choice(keys::under(at, control_names::scaling), keys::spelled(time_scaling_labels), time_scaling_labels[static_cast<std::size_t>(was.scaling)]);
    shape.field(keys::under(at, control_names::rate), config::field_kind::real, keys::text_of(no_override));
    shape.field(keys::under(at, control_names::rate_change), config::field_kind::real, keys::text_of(no_override));
}

control_parameters_window::settings read_control_parameters(const config::document &values, std::string_view at)
{
    const control_parameters_window::settings was;

    return control_parameters_window::settings{keys::real_at(values, keys::under(at, control_names::velocity), was.velocity), scaling_at(values, at, was.scaling),
                                               bounds_at(values, at)};
}

std::vector<config::edit> write_control_parameters(const control_parameters_window::settings &state, std::string_view at)
{
    const std::optional<path_parameter_bounds> held = state.trapezoid;

    return std::vector<config::edit>{
            config::edit{keys::under(at, control_names::velocity), keys::text_of(state.velocity)},
            config::edit{keys::under(at, control_names::scaling), time_scaling_labels[static_cast<std::size_t>(state.scaling)]},
            config::edit{keys::under(at, control_names::rate), keys::text_of(held ? static_cast<float>(held->max_rate) : no_override)},
            config::edit{keys::under(at, control_names::rate_change), keys::text_of(held ? static_cast<float>(held->max_rate_change) : no_override)},
    };
}

void declare_recording(config::declaration &shape, std::string_view at)
{
    const recording_parameters was;

    shape.group(std::string(at));
    shape.field(keys::under(at, recording_names::active), config::field_kind::flag, was.active ? "true" : "false");
    shape.field(keys::under(at, recording_names::directory), config::field_kind::text, was.directory.string());
}

recording_parameters read_recording(const config::document &values, std::string_view at)
{
    recording_parameters state;
    state.active    = keys::flag_at(values, keys::under(at, recording_names::active), state.active);
    state.directory = keys::text_at(values, keys::under(at, recording_names::directory), state.directory.string());
    return state;
}

std::vector<config::edit> write_recording(const recording_parameters &state, std::string_view at)
{
    return std::vector<config::edit>{
            config::edit{keys::under(at, recording_names::active), state.active ? "true" : "false"},
            config::edit{keys::under(at, recording_names::directory), state.directory.string()},
    };
}

}
