#include "configuration_keys.h"

#include "praxis/manipulator/velocity_configuration.h"

#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <string_view>

namespace praxis::manipulator {

namespace {

struct velocity_names
{
    static constexpr std::string_view frame   = "frame";
    static constexpr std::string_view linear  = "linear";
    static constexpr std::string_view capped  = "capped";
    static constexpr std::string_view angular = "angular";
    static constexpr std::string_view reading = "reading";
    static constexpr std::string_view columns = "columns";
};

// In the enumerations' own order, which is what reading one back as an index and casting relies on.
constexpr std::array<const char *, 2> frame_spellings{"space", "body"};
constexpr std::array<const char *, 2> reading_spellings{"velocity", "force"};

void declare_switch(config::declaration &shape, std::string_view at, std::string_view leaf, bool opened)
{
    shape.field(keys::under(at, leaf), config::field_kind::flag, opened ? "true" : "false");
}

}

void declare_velocity_kinematics(config::declaration &shape, std::string_view at)
{
    const velocity_kinematics_window::settings opened;

    shape.group(std::string(at));
    shape.choice(keys::under(at, velocity_names::frame), keys::spelled(frame_spellings), frame_spellings[static_cast<std::size_t>(opened.frame)]);
    shape.choice(keys::under(at, velocity_names::reading), keys::spelled(reading_spellings), reading_spellings[static_cast<std::size_t>(opened.reading)]);
    declare_switch(shape, at, velocity_names::angular, opened.angular);
    declare_switch(shape, at, velocity_names::linear, opened.linear);
    declare_switch(shape, at, velocity_names::columns, opened.columns);
    declare_switch(shape, at, velocity_names::capped, opened.capped);
}

velocity_kinematics_window::settings read_velocity_kinematics(const config::document &values, std::string_view at)
{
    const velocity_kinematics_window::settings opened;

    velocity_kinematics_window::settings state;
    state.frame   = static_cast<jacobian_frame>(keys::indexed(values, keys::under(at, velocity_names::frame), frame_spellings, static_cast<std::size_t>(opened.frame)));
    state.reading = static_cast<ellipsoid_view>(keys::indexed(values, keys::under(at, velocity_names::reading), reading_spellings, static_cast<std::size_t>(opened.reading)));
    state.angular = keys::flag_at(values, keys::under(at, velocity_names::angular), opened.angular);
    state.linear  = keys::flag_at(values, keys::under(at, velocity_names::linear), opened.linear);
    state.columns = keys::flag_at(values, keys::under(at, velocity_names::columns), opened.columns);
    state.capped  = keys::flag_at(values, keys::under(at, velocity_names::capped), opened.capped);

    return state;
}

std::vector<config::edit> write_velocity_kinematics(const velocity_kinematics_window::settings &state, std::string_view at)
{
    std::vector<config::edit> changes;
    changes.push_back(config::edit{keys::under(at, velocity_names::frame), frame_spellings[static_cast<std::size_t>(state.frame)]});
    changes.push_back(config::edit{keys::under(at, velocity_names::reading), reading_spellings[static_cast<std::size_t>(state.reading)]});
    changes.push_back(config::edit{keys::under(at, velocity_names::angular), state.angular ? "true" : "false"});
    changes.push_back(config::edit{keys::under(at, velocity_names::linear), state.linear ? "true" : "false"});
    changes.push_back(config::edit{keys::under(at, velocity_names::columns), state.columns ? "true" : "false"});
    changes.push_back(config::edit{keys::under(at, velocity_names::capped), state.capped ? "true" : "false"});

    return changes;
}

}
