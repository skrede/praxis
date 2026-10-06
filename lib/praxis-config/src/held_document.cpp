#include "engine.h"
#include "locator.h"
#include "key_path.h"

#include <set>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <algorithm>
#include <filesystem>
#include <string_view>

namespace praxis::config {
namespace {

bool indexed(std::string_view segment)
{
    return !segment.empty() && segment.back() == ']';
}

// The shallowest segment at which `held` names an instance that `asked` leaves unnamed, provided
// `held` reaches at least as deep and agrees with `asked` on every name and every ordinal it spells.
std::optional<std::size_t> omitted_ordinal(const std::vector<std::string_view> &held, const std::vector<std::string_view> &asked)
{
    if(held.size() < asked.size())
        return std::nullopt;

    std::optional<std::size_t> omitted;
    for(std::size_t at = 0; at < asked.size(); ++at)
    {
        const step stored = parsed(held[at]);
        const step wanted = parsed(asked[at]);
        if(stored.name != wanted.name || (indexed(asked[at]) && (!indexed(held[at]) || stored.ordinal != wanted.ordinal)))
            return std::nullopt;
        if(!omitted && indexed(held[at]) && !indexed(asked[at]))
            omitted = at;
    }
    return omitted;
}

std::vector<std::size_t> ordinals_of(std::string_view key)
{
    std::vector<std::size_t> ordinals;
    for(const std::string_view segment : segments_of(key))
        ordinals.push_back(parsed(segment).ordinal);
    return ordinals;
}

}

namespace detail {

held_document::held_document(entry_map values, std::filesystem::path from, defaults_map fallbacks, identity_map identities)
        : m_values(std::move(values))
        , m_from(std::move(from))
        , m_fallbacks(std::move(fallbacks))
        , m_identities(std::move(identities))
{
}

std::optional<std::string_view> held_document::value_at(std::string_view key) const
{
    const entry_map::const_iterator found = m_values.find(key);
    if(found == m_values.end())
        return std::nullopt;
    return std::string_view(found->second);
}

bool held_document::stored(std::string_view key) const
{
    return m_values.contains(key);
}

std::optional<crossed> held_document::crossing(std::string_view key) const
{
    const std::vector<std::string_view> asked = segments_of(key);
    std::optional<std::size_t> shallowest;
    std::set<std::size_t> named;
    for(const std::pair<const std::string, std::string> &value : m_values)
    {
        const std::vector<std::string_view> held = segments_of(value.first);
        const std::optional<std::size_t> at      = omitted_ordinal(held, asked);
        if(!at || (shallowest && *at > *shallowest))
            continue;
        if(!shallowest || *at < *shallowest)
            named.clear();
        shallowest = at;
        named.insert(parsed(held[*at]).ordinal);
    }

    if(!shallowest)
        return std::nullopt;
    return crossed{leading_segments(std::string(key), *shallowest + 1), named.size()};
}

std::vector<std::string> held_document::identities_in(std::string_view collection_path) const
{
    const std::optional<std::string> keyed_by = identity_of(collection_path);
    if(!keyed_by)
        return {};

    const std::string wanted = std::string(collection_path) + "/" + *keyed_by;
    std::vector<std::pair<std::vector<std::size_t>, std::string>> found;
    for(const std::pair<const std::string, std::string> &value : m_values)
        if(declared_path(value.first) == wanted)
            found.emplace_back(ordinals_of(value.first), value.second);
    std::ranges::stable_sort(found, {}, &std::pair<std::vector<std::size_t>, std::string>::first);

    std::vector<std::string> identities;
    for(std::pair<std::vector<std::size_t>, std::string> &one : found)
        identities.push_back(std::move(one.second));
    return identities;
}

const std::filesystem::path &held_document::from() const noexcept
{
    return m_from;
}

std::optional<leaf_default> held_document::fallback(std::string_view declared) const
{
    const defaults_map::const_iterator found = m_fallbacks.find(declared);
    if(found == m_fallbacks.end())
        return std::nullopt;
    return found->second;
}

std::optional<std::string> held_document::identity_of(std::string_view collection_path) const
{
    const identity_map::const_iterator found = m_identities.find(collection_path);
    if(found == m_identities.end())
        return std::nullopt;
    return found->second;
}

const identity_map &held_document::keyed() const noexcept
{
    return m_identities;
}

}
}
