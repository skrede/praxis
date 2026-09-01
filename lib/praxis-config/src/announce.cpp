#include "announce.h"
#include "key_path.h"

#include <spdlog/spdlog.h>

#include <string>
#include <vector>
#include <cstddef>
#include <utility>

namespace praxis::config {
namespace {

// The keys a declared leaf stands for once the collections it hangs under are addressed by ordinal:
// one per instance combination, and none at all where a collection carries no instance.
std::vector<std::string> addressed(const document &values, const declaration &shape, const std::string &leaf_path)
{
    std::vector<std::string> forms{leaf_path};
    for(const node &declared : shape.nodes())
    {
        if(declared.shape != node_kind::collection || !hangs_under(leaf_path, declared.path))
            continue;

        const std::size_t depth = segments_in(declared.path);
        std::vector<std::string> instanced;
        for(const std::string &form : forms)
        {
            const std::string prefix  = leading_segments(form, depth);
            const std::size_t present = values.identities(prefix).size();
            for(std::size_t ordinal = 0; ordinal < present; ++ordinal)
                instanced.push_back(prefix + "[" + std::to_string(ordinal) + "]" + form.substr(prefix.size()));
        }
        forms = std::move(instanced);
    }
    return forms;
}

// The keys the document carries no value for, and how many keys the declaration addresses in it.
struct substitutions
{
    std::vector<std::string> keys;
    std::size_t addressable;
};

substitutions substituted_in(const declaration &shape, const document &values)
{
    substitutions found{{}, 0};
    for(const node &declared : shape.nodes())
    {
        if(declared.shape != node_kind::leaf)
            continue;
        for(const std::string &key : addressed(values, shape, declared.path))
        {
            ++found.addressable;
            if(values.origin_of(key).kind == origin_kind::fallback)
                found.keys.push_back(key);
        }
    }
    return found;
}

}

// A change deciding where a file is read from is reported where an operator sees it without
// turning anything on, and both paths are named whenever they differ.
void report(const location &at)
{
    if(at.given == at.resolved)
        spdlog::info("praxis: reading the configuration from {}", at.resolved.string());
    else
        spdlog::info("praxis: reading the configuration from {}, resolved from {}", at.resolved.string(), at.given.string());
}

// A malformed document is never what a caller expected, whatever it expected the document to carry,
// so only the absent-source arm follows the expectation down to the detail level.
void announce_refusal(const location &at, const error &refused, expectation carries)
{
    if(refused.code != error_code::absent_source)
    {
        spdlog::error("praxis: the configuration at {} could not be used ({}): {}; every value comes from the declared fallbacks", at.resolved.string(), error_name(refused.code),
                      refused.message);
        return;
    }

    const spdlog::level::level_enum level = carries == expectation::partial ? spdlog::level::debug : spdlog::level::warn;
    if(spdlog::should_log(level))
        spdlog::log(level, "praxis: there is no configuration file at {}; every value comes from the declared fallbacks", at.resolved.string());
}

// The count is published where an operator sees it without turning anything on; the per-key list
// below it is what the detail level is for, and is assembled only when that level is admitted.
void announce_substitutions(const declaration &shape, const document &values, expectation carries)
{
    const substitutions found = substituted_in(shape, values);
    if(found.keys.empty())
        return;

    const spdlog::level::level_enum level = carries == expectation::partial ? spdlog::level::debug : spdlog::level::info;
    if(spdlog::should_log(level))
        spdlog::log(level, "praxis: {} of {} declared values are not in {} and carry the fallbacks the declaration named", found.keys.size(), found.addressable,
                    values.source().string());

    if(!spdlog::should_log(spdlog::level::debug))
        return;
    for(const std::string &key : found.keys)
        spdlog::debug("praxis: '{}' is not in the configuration file and carries the fallback the declaration named", key);
}

}
