#include "demo_configuration.h"

#include "demo_write_back.h"

#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace praxis::demo {

namespace {

constexpr const char *window_width  = "window/width";
constexpr const char *window_height = "window/height";
constexpr const char *window_x      = "window/x";
constexpr const char *window_y      = "window/y";
constexpr const char *level_key     = "reporting/level";

// In the enumeration's own order, which is what reading one back as an index and casting relies on.
constexpr std::array<const char *, 4> reporting_levels{"debug", "info", "warning", "error"};

std::optional<int> stored(const config::document &values, const char *key)
{
    if(values.origin_of(key).kind != config::origin_kind::source)
        return std::nullopt;

    const expected<std::int64_t, config::error> read = values.integer(key);
    if(!read)
        return std::nullopt;

    return static_cast<int>(read.value());
}

std::optional<int> stored_extent(const config::document &values, const char *key)
{
    const std::optional<int> read = stored(values, key);

    return read && *read > 0 ? read : std::nullopt;
}

void append(std::vector<config::edit> &changes, const char *key, const std::optional<int> &value)
{
    if(value)
        changes.push_back(config::edit{key, std::to_string(*value)});
}

}

config::declaration preferences_keyspace()
{
    config::declaration shape("preferences");
    shape.group("window");
    shape.field(window_width, config::field_kind::integer, "0");
    shape.field(window_height, config::field_kind::integer, "0");
    shape.field(window_x, config::field_kind::integer, "0");
    shape.field(window_y, config::field_kind::integer, "0");

    // The fallback is the level in force before any document has been read.
    shape.group("reporting");
    shape.choice(level_key, std::vector<std::string>(reporting_levels.begin(), reporting_levels.end()), reporting_levels[static_cast<std::size_t>(scene::reporting_level())]);
    declare_leaving(shape);

    return shape;
}

scene::visualizer::geometry preferred_geometry(const config::document &values)
{
    scene::visualizer::geometry where;
    where.width  = stored_extent(values, window_width);
    where.height = stored_extent(values, window_height);
    where.x      = stored(values, window_x);
    where.y      = stored(values, window_y);

    return where;
}

scene::severity preferred_level(const config::document &values)
{
    const expected<std::string, config::error> read = values.text(level_key);
    for(std::size_t option = 0u; read && option < reporting_levels.size(); ++option)
        if(read.value() == reporting_levels[option])
            return static_cast<scene::severity>(option);

    return scene::reporting_level();
}

std::vector<config::edit> preferences_edits(const scene::visualizer::geometry &left, scene::severity level)
{
    std::vector<config::edit> changes;
    append(changes, window_width, left.width);
    append(changes, window_height, left.height);
    append(changes, window_x, left.x);
    append(changes, window_y, left.y);
    changes.push_back(config::edit{level_key, reporting_levels[static_cast<std::size_t>(level)]});

    return changes;
}

config::declaration demonstration_keyspace()
{
    config::declaration shape("demonstration");
    shape.group("presets").collection(preset_instances, "name");
    shape.field(std::string(preset_instances) + "/" + document_leaf, config::field_kind::text, "");

    return shape;
}

std::vector<std::string> preset_keys(const config::document &values)
{
    return values.identities(preset_instances);
}

expected<config::location, config::error> preset_document(const config::document &values, const std::string &key, const documents &mine)
{
    const expected<std::string, config::error> addressed = values.key(preset_instances, key, document_leaf);
    if(!addressed)
        return unexpected(addressed.error());

    const expected<std::string, config::error> named = values.text(addressed.value());
    if(!named)
        return unexpected(named.error());

    return mine.reading(named.value());
}

std::vector<config::location> preset_locations(const config::document &values, const documents &mine)
{
    std::vector<config::location> reading;
    for(const std::string &key : preset_keys(values))
    {
        const expected<config::location, config::error> at = preset_document(values, key, mine);
        if(at && !at.value().given.empty())
            reading.push_back(at.value());
    }

    return reading;
}

}
