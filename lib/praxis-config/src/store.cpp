#include "fold.h"
#include "check.h"
#include "engine.h"
#include "announce.h"
#include "source_text.h"

#include "praxis/config/store.h"

#include <string>
#include <memory>
#include <cstdint>
#include <fstream>
#include <utility>
#include <optional>
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

document held(const declaration &shape, detail::entry_map values, const std::filesystem::path &from)
{
    return document(std::make_shared<const detail::held_document>(std::move(values), from, fallbacks_of(shape), identities_of(shape)));
}

// A document holding no value at all: every read lands on the fallback the declaration named, which
// is what makes a refused file still answerable.
document fallbacks_only(const declaration &shape, const std::filesystem::path &from)
{
    return held(shape, detail::entry_map(), from);
}

// Everything the file's own state decides, before a byte of it is read.
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

expected<document, error> load_through(const declaration &shape, const location &at)
{
    const expected<nucleus::config_space, error> space = sealed_space(shape);
    if(!space)
        return unexpected(space.error());

    if(const std::optional<error> refused = unusable_before_reading(at.resolved); refused)
        return unexpected(*refused);

    const std::optional<std::string> source = slurped(at.resolved);
    if(!source)
        return unexpected(error{error_code::unreadable_source, "the configuration file at " + at.resolved.string() + " cannot be opened for reading"});

    expected<folding, error> walked = folded(*source, shape, at.resolved);
    if(!walked)
        return unexpected(walked.error());
    if(const std::optional<error> refused = refused_content(walked.value(), shape, at.resolved); refused)
        return unexpected(*refused);
    return held(shape, std::move(walked.value().entries), at.resolved);
}

}

expected<document, error> load(const declaration &shape, const location &at)
{
    return load_through(shape, at);
}

outcome load_or_defaults(const declaration &shape, const location &at, expectation carries)
{
    report(at);

    const expected<document, error> loaded = load_through(shape, at);
    if(!loaded)
    {
        announce_refusal(at, loaded.error(), carries);
        return outcome{fallbacks_only(shape, at.resolved), loaded.error()};
    }

    announce_substitutions(shape, loaded.value(), carries);
    return outcome{loaded.value(), std::nullopt};
}

}
