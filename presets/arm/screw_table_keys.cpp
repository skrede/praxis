#include "screw_table_keys.h"

#include "praxis/presets/screw_table.h"

#include "praxis/config/error.h"

#include "praxis/compat/expected.h"

#include <Eigen/Core>

#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <charconv>
#include <optional>
#include <algorithm>
#include <string_view>

namespace praxis::presets::keys {

namespace {

constexpr std::array<const char *, 3> components{"x", "y", "z"};

bool carried(const config::document &values, const std::string &key)
{
    return values.origin_of(key).kind == config::origin_kind::source;
}

double real_at(const config::document &values, const std::string &key, double fallback)
{
    if(!carried(values, key))
        return fallback;

    const expected<double, config::error> read = values.real(key);

    return read ? read.value() : fallback;
}

// A zero of either sign is written as the unsigned zero the declaration falls back to.
std::string shortest_text(float value)
{
    std::array<char, 32> printed{};
    const std::to_chars_result written = std::to_chars(printed.data(), printed.data() + printed.size(), value == 0.0f ? 0.0f : value);

    return std::string(printed.data(), written.ptr);
}

config::error unreadable(const std::string &identity, const std::string &fault)
{
    return config::error{config::error_code::rejected_content, "the chain kept here addresses a row by '" + identity + "', which " + fault};
}

}

std::string under(std::string_view at, std::string_view leaf)
{
    std::string key(at);
    key += '/';
    key.append(leaf);

    return key;
}

std::optional<std::size_t> ordinal_of(const std::string &identity)
{
    std::size_t named                 = 0u;
    const char *const last            = identity.data() + identity.size();
    const std::from_chars_result read = std::from_chars(identity.data(), last, named);
    const bool canonical              = read.ec == std::errc() && read.ptr == last && named >= 1u && std::to_string(named) == identity;

    return canonical ? std::optional<std::size_t>(named) : std::optional<std::size_t>();
}

expected<std::size_t, config::error> reach_of(const std::vector<std::string> &present, std::size_t joints)
{
    std::size_t reach = joints;
    for(const std::string &identity : present)
    {
        if(identity.empty())
            continue;

        const std::optional<std::size_t> named = ordinal_of(identity);
        if(!named)
            return unexpected(unreadable(identity, "names no joint's place in a chain"));
        if(*named > joints + screw_table_greatest_surplus)
            return unexpected(unreadable(identity, "names a joint further past the end of this chain than a chain is read out to"));

        reach = std::max(reach, *named);
    }

    return reach;
}

void declare_triple(config::declaration &shape, const std::string &at)
{
    shape.group(at);
    for(const char *component : components)
        shape.field(under(at, component), config::field_kind::real, "0");
}

std::optional<std::string> instance_at(const config::document &values, const std::string &collection, const std::string &named)
{
    const std::vector<std::string> present = values.identities(collection);
    for(std::size_t which = 0u; which < present.size(); ++which)
        if(present[which] == named)
            return collection + "[" + std::to_string(which) + "]";

    return std::nullopt;
}

Eigen::Vector3d read_triple(const config::document &values, const std::string &at, const Eigen::Vector3d &fallback)
{
    Eigen::Vector3d read = fallback;
    for(std::size_t axis = 0u; axis < components.size(); ++axis)
    {
        const Eigen::Index component = static_cast<Eigen::Index>(axis);
        read[component]              = real_at(values, under(at, components[axis]), fallback[component]);
    }

    return read;
}

void write_shortest(std::vector<config::edit> &into, const std::string &at, const Eigen::Vector3f &value)
{
    for(std::size_t axis = 0u; axis < components.size(); ++axis)
        into.push_back(config::edit{under(at, components[axis]), shortest_text(value[static_cast<Eigen::Index>(axis)])});
}

void write_exact(std::vector<config::edit> &into, const std::string &at, const Eigen::Vector3d &value)
{
    for(std::size_t axis = 0u; axis < components.size(); ++axis)
        into.push_back(config::edit{under(at, components[axis]), config::exact_text(value[static_cast<Eigen::Index>(axis)])});
}

}
