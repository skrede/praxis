#include "configuration_keys.h"

#include <span>
#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <charconv>
#include <string_view>

namespace praxis::manipulator::keys {

namespace {

constexpr std::array<const char *, 3> components{"x", "y", "z"};
constexpr std::array<const char *, 2> control_modes{"preview", "simulation"};

constexpr std::string_view mode_leaf = "mode";

}

std::string under(std::string_view at, std::string_view leaf)
{
    std::string key(at);
    key += '/';
    key.append(leaf);
    return key;
}

std::string text_of(float value)
{
    std::array<char, 32> printed{};
    const std::to_chars_result written = std::to_chars(printed.data(), printed.data() + printed.size(), value);
    return std::string(printed.data(), written.ptr);
}

void declare_mode(config::declaration &shape, std::string_view at, control_mode fallback)
{
    shape.choice(under(at, mode_leaf), spelled(control_modes), control_modes[static_cast<std::size_t>(fallback)]);
}

control_mode read_mode(const config::document &values, std::string_view at, control_mode fallback)
{
    return static_cast<control_mode>(indexed(values, under(at, mode_leaf), control_modes, static_cast<std::size_t>(fallback)));
}

config::edit written_mode(control_mode mode, std::string_view at)
{
    return config::edit{under(at, mode_leaf), control_modes[static_cast<std::size_t>(mode)]};
}

std::vector<std::string> spelled(std::span<const char *const> labels)
{
    return std::vector<std::string>(labels.begin(), labels.end());
}

bool flag_at(const config::document &values, const std::string &key, bool fallback)
{
    const expected<bool, config::error> read = values.flag(key);
    return read ? read.value() : fallback;
}

float real_at(const config::document &values, const std::string &key, float fallback)
{
    const expected<double, config::error> read = values.real(key);
    return read ? static_cast<float>(read.value()) : fallback;
}

std::string text_at(const config::document &values, const std::string &key, std::string_view fallback)
{
    const expected<std::string, config::error> read = values.text(key);
    return read ? read.value() : std::string(fallback);
}

std::size_t indexed(const config::document &values, const std::string &key, std::span<const char *const> labels, std::size_t fallback)
{
    const expected<std::string, config::error> read = values.text(key);
    if(!read)
        return fallback;

    for(std::size_t option = 0u; option < labels.size(); ++option)
        if(read.value() == labels[option])
            return option;

    return fallback;
}

void declare_vector(config::declaration &shape, const std::string &at, const Eigen::Vector3f &fallback)
{
    shape.group(at);
    for(std::size_t axis = 0u; axis < components.size(); ++axis)
        shape.field(under(at, components[axis]), config::field_kind::real, text_of(fallback[static_cast<Eigen::Index>(axis)]));
}

Eigen::Vector3f read_vector(const config::document &values, const std::string &at, const Eigen::Vector3f &fallback)
{
    Eigen::Vector3f read = fallback;
    for(std::size_t axis = 0u; axis < components.size(); ++axis)
    {
        const Eigen::Index component = static_cast<Eigen::Index>(axis);
        read[component]              = real_at(values, under(at, components[axis]), fallback[component]);
    }
    return read;
}

void write_vector(std::vector<config::edit> &into, const std::string &at, const Eigen::Vector3f &value)
{
    for(std::size_t axis = 0u; axis < components.size(); ++axis)
        into.push_back(config::edit{under(at, components[axis]), text_of(value[static_cast<Eigen::Index>(axis)])});
}

void declare_order(config::declaration &shape, const std::string &key, axis_order fallback)
{
    shape.choice(key, spelled(axis_order_labels()), order_text(fallback));
}

axis_order read_order(const config::document &values, const std::string &key, axis_order fallback)
{
    return static_cast<axis_order>(indexed(values, key, axis_order_labels(), static_cast<std::size_t>(fallback)));
}

std::string order_text(axis_order value)
{
    return std::string(axis_order_labels()[static_cast<std::size_t>(value)]);
}

}
