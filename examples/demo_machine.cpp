#include "demo_machine.h"
#include "demo_configuration.h"

#include <spdlog/spdlog.h>

#include <format>
#include <string>
#include <utility>
#include <optional>
#include <filesystem>

namespace praxis::demo {

std::optional<config::binding> machine_screw_table(const config::document &values, const documents &mine)
{
    const expected<std::string, config::error> named = values.text("screw_table/document");
    if(!named || named.value().empty())
        return std::nullopt;

    return config::binding{presets::screw_table_keyspace(), mine.writing(named.value()), config::expectation::partial};
}

config::binding machine_binding(const std::filesystem::path &named, const documents &mine)
{
    return config::binding{machine_keyspace(), mine.reading(named), config::expectation::partial};
}

expected<config::binding, config::error> preset_binding(const config::document &values, const std::string &key, const documents &mine)
{
    const expected<config::location, config::error> at = preset_document(values, key, mine);
    if(!at)
        return unexpected(at.error());

    return machine_binding(at.value().given, mine);
}

// The binding a preset composes through, and nothing where the application's document names no
// document for it or where that document names no description. A preset that is not offered is
// named where it is left out.
std::optional<offered_document> offered_preset(const config::document &values, const std::string &key, const documents &mine)
{
    const expected<config::binding, config::error> bound = preset_binding(values, key, mine);
    if(!bound)
    {
        spdlog::error(std::format("The preset {} is not offered: {}", key, bound.error().message));

        return std::nullopt;
    }

    config::outcome read = config::load_or_defaults(bound.value());
    if(machine_description(read.values).empty())
    {
        spdlog::error(std::format("The preset {} is not offered: its document names no description", key));

        return std::nullopt;
    }

    return offered_document{bound.value(), std::move(read.values)};
}

}
