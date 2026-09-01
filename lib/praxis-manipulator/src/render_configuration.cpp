#include "configuration_keys.h"

#include "praxis/manipulator/render_configuration.h"

#include <string>
#include <vector>
#include <optional>
#include <string_view>

namespace praxis::manipulator {

namespace {

// A drawing of no extent is no drawing at all, and neither is a cut of none, so every one of these
// leaves spells absence with a value at or below zero: the value then keeps whatever the stencil
// opened it at, which is proportioned to the rendered arm's own size.
constexpr float unnamed_value = 0.f;

struct render_names
{
    static constexpr std::string_view linear_scale         = "linear_scale";
    static constexpr std::string_view angular_scale        = "angular_scale";
    static constexpr std::string_view force_cap_ratio      = "force_cap_ratio";
    static constexpr std::string_view linear_column_scale  = "linear_column_scale";
    static constexpr std::string_view angular_column_scale = "angular_column_scale";
};

std::string value_text(const std::optional<double> &value)
{
    return keys::text_of(static_cast<float>(value.value_or(unnamed_value)));
}

std::optional<double> value_at(const config::document &values, std::string_view at, std::string_view leaf)
{
    const float read = keys::real_at(values, keys::under(at, leaf), unnamed_value);

    return read > unnamed_value ? std::optional<double>(read) : std::nullopt;
}

void declare_value(config::declaration &shape, std::string_view at, std::string_view leaf, const std::optional<double> &opened)
{
    shape.field(keys::under(at, leaf), config::field_kind::real, value_text(opened));
}

}

void declare_render_controls(config::declaration &shape, std::string_view at)
{
    const render_controls_window::settings opened;

    shape.group(std::string(at));
    declare_value(shape, at, render_names::angular_scale, opened.angular_scale);
    declare_value(shape, at, render_names::linear_scale, opened.linear_scale);
    declare_value(shape, at, render_names::angular_column_scale, opened.angular_column_scale);
    declare_value(shape, at, render_names::linear_column_scale, opened.linear_column_scale);
    declare_value(shape, at, render_names::force_cap_ratio, opened.force_cap_ratio);
}

render_controls_window::settings read_render_controls(const config::document &values, std::string_view at)
{
    render_controls_window::settings state;
    state.angular_scale        = value_at(values, at, render_names::angular_scale);
    state.linear_scale         = value_at(values, at, render_names::linear_scale);
    state.angular_column_scale = value_at(values, at, render_names::angular_column_scale);
    state.linear_column_scale  = value_at(values, at, render_names::linear_column_scale);
    state.force_cap_ratio      = value_at(values, at, render_names::force_cap_ratio);

    return state;
}

std::vector<config::edit> write_render_controls(const render_controls_window::settings &state, std::string_view at)
{
    std::vector<config::edit> changes;
    changes.push_back(config::edit{keys::under(at, render_names::angular_scale), value_text(state.angular_scale)});
    changes.push_back(config::edit{keys::under(at, render_names::linear_scale), value_text(state.linear_scale)});
    changes.push_back(config::edit{keys::under(at, render_names::angular_column_scale), value_text(state.angular_column_scale)});
    changes.push_back(config::edit{keys::under(at, render_names::linear_column_scale), value_text(state.linear_column_scale)});
    changes.push_back(config::edit{keys::under(at, render_names::force_cap_ratio), value_text(state.force_cap_ratio)});

    return changes;
}

}
