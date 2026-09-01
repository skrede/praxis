#include "screw_table_keys.h"

#include "praxis/config/error.h"

#include "praxis/compat/expected.h"

#include <Eigen/Core>

#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <charconv>
#include <optional>
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

std::string shortest_text(float value)
{
    std::array<char, 32> printed{};
    const std::to_chars_result written = std::to_chars(printed.data(), printed.data() + printed.size(), value);

    return std::string(printed.data(), written.ptr);
}

}

std::string under(std::string_view at, std::string_view leaf)
{
    std::string key(at);
    key += '/';
    key.append(leaf);

    return key;
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
