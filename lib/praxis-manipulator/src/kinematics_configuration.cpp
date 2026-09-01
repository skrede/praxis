#include "configuration_keys.h"

#include "praxis/manipulator/kinematics_configuration.h"

#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace praxis::manipulator {

namespace {

struct branch_names
{
    static constexpr std::string_view figures = "figures";
};

struct iterate_names
{
    static constexpr std::string_view start = "start";
};

struct convergence_names
{
    static constexpr std::string_view linear  = "linear";
    static constexpr std::string_view angular = "angular";
};

// The rows are counted from one in the document and from zero in memory, so a value below one names
// no row and leaves the choice where it stood.
std::size_t counted_start(const config::document &values, const std::string &key, std::size_t fallback)
{
    const expected<std::int64_t, config::error> read = values.integer(key);

    return read && *read >= 1 ? static_cast<std::size_t>(*read - 1) : fallback;
}

}

void declare_ik_branch(config::declaration &shape, std::string_view at)
{
    const ik_branch_window::settings was;

    shape.group(std::string(at));
    shape.field(keys::under(at, branch_names::figures), config::field_kind::flag, was.figures ? "true" : "false");
    keys::declare_mode(shape, at, was.mode);
}

ik_branch_window::settings read_ik_branch(const config::document &values, std::string_view at)
{
    const ik_branch_window::settings was;

    return ik_branch_window::settings{keys::read_mode(values, at, was.mode), keys::flag_at(values, keys::under(at, branch_names::figures), was.figures)};
}

std::vector<config::edit> write_ik_branch(const ik_branch_window::settings &state, std::string_view at)
{
    return std::vector<config::edit>{
            config::edit{keys::under(at, branch_names::figures), state.figures ? "true" : "false"},
            keys::written_mode(state.mode, at),
    };
}

void declare_ik_iterates(config::declaration &shape, std::string_view at)
{
    const ik_iterate_window::settings was;

    shape.group(std::string(at));
    shape.field(keys::under(at, iterate_names::start), config::field_kind::integer, std::to_string(was.start + 1u));
    keys::declare_mode(shape, at, was.mode);
}

ik_iterate_window::settings read_ik_iterates(const config::document &values, std::string_view at)
{
    const ik_iterate_window::settings was;

    return ik_iterate_window::settings{counted_start(values, keys::under(at, iterate_names::start), was.start), keys::read_mode(values, at, was.mode)};
}

std::vector<config::edit> write_ik_iterates(const ik_iterate_window::settings &state, std::string_view at)
{
    return std::vector<config::edit>{
            config::edit{keys::under(at, iterate_names::start), std::to_string(state.start + 1u)},
            keys::written_mode(state.mode, at),
    };
}

void declare_ik_convergence(config::declaration &shape, std::string_view at)
{
    const ik_convergence_window::settings was;

    shape.group(std::string(at));
    shape.field(keys::under(at, convergence_names::linear), config::field_kind::flag, was.linear ? "true" : "false");
    shape.field(keys::under(at, convergence_names::angular), config::field_kind::flag, was.angular ? "true" : "false");
}

ik_convergence_window::settings read_ik_convergence(const config::document &values, std::string_view at)
{
    const ik_convergence_window::settings was;

    return ik_convergence_window::settings{keys::flag_at(values, keys::under(at, convergence_names::angular), was.angular),
                                           keys::flag_at(values, keys::under(at, convergence_names::linear), was.linear)};
}

std::vector<config::edit> write_ik_convergence(const ik_convergence_window::settings &state, std::string_view at)
{
    return std::vector<config::edit>{
            config::edit{keys::under(at, convergence_names::linear), state.linear ? "true" : "false"},
            config::edit{keys::under(at, convergence_names::angular), state.angular ? "true" : "false"},
    };
}

}
