#include "engine.h"
#include "announce.h"

#include "praxis/config/store.h"

#include <nucleus/log_sink.h>
#include <nucleus/config_space.h>

#include <nucleus/xml/xml_source.h>

#include <nucleus/config_source/source_stack.h>

#include <spdlog/spdlog.h>

#include <string>
#include <memory>
#include <cstdint>
#include <fstream>
#include <utility>
#include <optional>
#include <exception>
#include <filesystem>
#include <system_error>

namespace praxis::config {
namespace {

detail::defaults_map fallbacks_of(const declaration &shape)
{
    detail::defaults_map named;
    for(const node &declared : shape.nodes())
        if(declared.shape == node_kind::leaf)
            named.emplace(declared.path, detail::leaf_default{declared.kind, declared.fallback});
    return named;
}

detail::identity_map identities_of(const declaration &shape)
{
    detail::identity_map keyed;
    for(const node &declared : shape.nodes())
        if(declared.shape == node_kind::collection)
            keyed.emplace(declared.path, declared.identity);
    return keyed;
}

// A document folded from nothing at all: every read misses the keyspace and lands on the fallback
// the declaration named, which is what makes a refused file still answerable.
document fallbacks_only(const declaration &shape, const std::filesystem::path &from)
{
    return document(std::make_shared<const detail::held_document>(nucleus::config(), from, fallbacks_of(shape), identities_of(shape)));
}

// The engine reports a document whose root is not the declared one and a document that does not
// parse at all with the same code, and only a pull that does not require the root separates them.
bool parses_without_the_declared_root(const location &at)
{
    nucleus::xml_source reader = nucleus::xml_source::from(nucleus::xml_source_options::of_file(at.resolved.string()));
    return reader.pull().has_value();
}

// Everything the file's own state decides, before the engine is handed a path at all.
std::optional<error> unusable_before_reading(const std::filesystem::path &resolved)
{
    std::error_code probing;
    const std::filesystem::file_status state = std::filesystem::status(resolved, probing);
    if(!std::filesystem::exists(state))
        return error{error_code::absent_source, "there is no configuration file at " + resolved.string()};
    if(!std::filesystem::is_regular_file(state))
        return error{error_code::unreadable_source, "the path " + resolved.string() + " does not name a file that can be read"};

    std::error_code sizing;
    const std::uintmax_t bytes = std::filesystem::file_size(resolved, sizing);
    if(sizing)
        return error{error_code::unreadable_source, "the configuration file at " + resolved.string() + " cannot be read: " + sizing.message()};

    std::ifstream reachable(resolved, std::ios::binary);
    if(!reachable)
        return error{error_code::unreadable_source, "the configuration file at " + resolved.string() + " cannot be opened for reading"};
    if(bytes == 0)
        return error{error_code::empty_source, "the configuration file at " + resolved.string() + " is empty"};

    return std::nullopt;
}

nucleus::source_stack sourced(const declaration &shape, const location &at)
{
    nucleus::xml_source reader = nucleus::xml_source::from(nucleus::xml_source_options::of_file(at.resolved.string()));
    reader.with_space_name(shape.space());
    return nucleus::source_stack(std::move(reader));
}

// A null sink is the engine's own spelling of "say nothing", so a read that installs none is silent
// whatever the engine has to remark on.
expected<document, error> folded_through(const nucleus::config_space &space, const declaration &shape, const location &at, nucleus::log_sink *sink)
{
    try
    {
        nucleus::load_options options;
        options.log = sink;

        nucleus::source_stack stack = sourced(shape, at);
        nucleus::load_result folded = nucleus::load_config(space, stack, options);
        if(!folded)
        {
            if(folded.error().code == nucleus::errc::malformed_source && parses_without_the_declared_root(at))
                return unexpected(error{error_code::mismatched_space, folded.error().message});
            return unexpected(translated(folded.error()));
        }

        return document(std::make_shared<const detail::held_document>(std::move(folded).value(), at.resolved, fallbacks_of(shape), identities_of(shape)));
    }
    catch(const std::exception &thrown)
    {
        return unexpected(error{error_code::malformed_source, "the configuration at " + at.resolved.string() + " could not be read: " + thrown.what()});
    }
}

expected<document, error> load_through(const declaration &shape, const location &at, nucleus::log_sink *sink)
{
    const expected<nucleus::config_space, error> space = sealed_space(shape);
    if(!space)
        return unexpected(space.error());

    if(const std::optional<error> refused = unusable_before_reading(at.resolved); refused)
        return unexpected(*refused);

    return folded_through(space.value(), shape, at, sink);
}

}

expected<document, error> load(const declaration &shape, const location &at)
{
    return load_through(shape, at, nullptr);
}

outcome load_or_defaults(const declaration &shape, const location &at, expectation carries)
{
    report(at);

    nucleus::log_sink_f bridge([](nucleus::log_level, std::string_view message) { spdlog::debug("praxis: {}", message); });

    expected<document, error> loaded = load_through(shape, at, &bridge);
    if(!loaded)
    {
        announce_refusal(at, loaded.error(), carries);
        return outcome{fallbacks_only(shape, at.resolved), loaded.error()};
    }

    announce_substitutions(shape, loaded.value(), carries);
    return outcome{std::move(loaded).value(), std::nullopt};
}

}
