#include "configuration_keys.h"

#include "praxis/manipulator/control_configuration.h"

#include <string>
#include <vector>
#include <string_view>

namespace praxis::manipulator {

namespace {

struct screw_names
{
    static constexpr std::string_view pitch = "pitch";
    static constexpr std::string_view theta = "theta";
    static constexpr std::string_view q     = "q";
    static constexpr std::string_view w     = "w";
};

void write_screw(std::vector<config::edit> &changes, const screw_jog_window::settings &state, std::string_view at)
{
    changes.push_back(config::edit{keys::under(at, screw_names::pitch), keys::text_of(state.pitch)});
    changes.push_back(config::edit{keys::under(at, screw_names::theta), keys::text_of(state.theta_degrees)});
    keys::write_vector(changes, keys::under(at, screw_names::q), state.q);
    keys::write_vector(changes, keys::under(at, screw_names::w), state.w);
}

}

void declare_screw_jog(config::declaration &shape, std::string_view at)
{
    const screw_jog_window::settings was;

    shape.group(std::string(at));
    keys::declare_mode(shape, at, was.mode);
    shape.field(keys::under(at, screw_names::pitch), config::field_kind::real, keys::text_of(was.pitch));
    shape.field(keys::under(at, screw_names::theta), config::field_kind::real, keys::text_of(was.theta_degrees));
    keys::declare_vector(shape, keys::under(at, screw_names::q), was.q);
    keys::declare_vector(shape, keys::under(at, screw_names::w), was.w);
}

screw_jog_window::settings read_screw_jog(const config::document &values, std::string_view at)
{
    const screw_jog_window::settings was;

    return screw_jog_window::settings{keys::read_mode(values, at, was.mode), keys::read_vector(values, keys::under(at, screw_names::q), was.q),
                                      keys::read_vector(values, keys::under(at, screw_names::w), was.w), keys::real_at(values, keys::under(at, screw_names::pitch), was.pitch),
                                      keys::real_at(values, keys::under(at, screw_names::theta), was.theta_degrees)};
}

std::vector<config::edit> write_screw_jog(const screw_jog_window::settings &state, std::string_view at)
{
    return write_screw_jog(state, at, at);
}

std::vector<config::edit> write_screw_jog(const screw_jog_window::settings &state, std::string_view at, std::string_view screw_at)
{
    std::vector<config::edit> changes{keys::written_mode(state.mode, at)};
    if(!screw_at.empty())
        write_screw(changes, state, screw_at);

    return changes;
}

}
