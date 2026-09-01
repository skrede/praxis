#include "configuration_keys.h"

#include <span>
#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <charconv>
#include <optional>
#include <string_view>

namespace praxis::rigid_motion::keys {

namespace {

constexpr std::array<const char *, 3> components{"x", "y", "z"};

std::vector<std::string> spelled(std::span<const char *const> labels)
{
    return std::vector<std::string>(labels.begin(), labels.end());
}

bool carried(const config::document &values, const std::string &key)
{
    return values.origin_of(key).kind == config::origin_kind::source;
}

float real_at(const config::document &values, const std::string &key, float fallback)
{
    if(!carried(values, key))
        return fallback;

    const expected<double, config::error> read = values.real(key);
    return read ? static_cast<float>(read.value()) : fallback;
}

// The position of the document's value in `labels`, or `fallback` where the document carries no
// value there, or one the list does not name.
std::size_t indexed(const config::document &values, const std::string &key, std::span<const char *const> labels, std::size_t fallback)
{
    if(!carried(values, key))
        return fallback;

    const expected<std::string, config::error> read = values.text(key);
    if(!read)
        return fallback;

    for(std::size_t option = 0u; option < labels.size(); ++option)
        if(read.value() == labels[option])
            return option;

    return fallback;
}

void write_placement(std::vector<config::edit> &into, const std::string &instance, const frame_window::placement &one, std::span<const std::string> objects)
{
    into.push_back(config::edit{under(instance, arrangement_names::parent), name_of(one.parent, objects)});
    into.push_back(config::edit{under(instance, arrangement_names::order), order_text(one.order)});
    write_vector(into, under(instance, arrangement_names::position), one.position);
    write_vector(into, under(instance, arrangement_names::euler), one.euler_degrees);
}

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

std::optional<std::string> carried_text(const config::document &values, const std::string &key)
{
    if(!carried(values, key))
        return std::nullopt;

    const expected<std::string, config::error> read = values.text(key);
    if(!read)
        return std::nullopt;

    return read.value();
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

std::string name_of(std::optional<std::size_t> which, std::span<const std::string> objects)
{
    return which && *which < objects.size() ? objects[*which] : std::string();
}

std::optional<std::string> instance_at(const config::document &values, const std::string &collection, const std::string &named)
{
    const std::vector<std::string> present = values.identities(collection);
    for(std::size_t ordinal = 0u; ordinal < present.size(); ++ordinal)
        if(present[ordinal] == named)
            return collection + "[" + std::to_string(ordinal) + "]";

    return std::nullopt;
}

std::vector<config::edit> moved_leaves(const std::string &instance, const frame_window::placement &was, const frame_window::placement &now, std::span<const std::string> objects)
{
    std::vector<config::edit> before;
    std::vector<config::edit> after;
    write_placement(before, instance, was, objects);
    write_placement(after, instance, now, objects);

    std::vector<config::edit> moved;
    for(std::size_t leaf = 0u; leaf < after.size(); ++leaf)
        if(before[leaf].value != after[leaf].value)
            moved.push_back(after[leaf]);

    return moved;
}

frame_window::placement placed_at(const frame_window::settings &state, std::size_t which)
{
    return which < state.objects.size() ? state.objects[which] : frame_window::placement();
}

}
