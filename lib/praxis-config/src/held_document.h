#ifndef HPP_GUARD_PRAXIS_CONFIG_HELD_DOCUMENT_H
#define HPP_GUARD_PRAXIS_CONFIG_HELD_DOCUMENT_H

#include "praxis/config/declaration.h"

#include <map>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <filesystem>
#include <functional>
#include <string_view>

namespace praxis::config::detail {

struct leaf_default
{
    field_kind kind;
    std::string text;
};

using identity_map = std::map<std::string, std::string, std::less<>>;
using defaults_map = std::map<std::string, leaf_default, std::less<>>;
using entry_map    = std::map<std::string, std::string, std::less<>>;

// A collection a key passes without naming an instance, and how many instances stored keys name there.
struct crossed
{
    std::string container;
    std::size_t count;
};

// What every copy of a document shares: the text of every value the file carries, keyed by path with
// each instance's ordinal, the file it was read from, and the two things a read needs that the file
// does not carry -- the fallback each declared leaf named, and which leaf keys each collection's
// instances.
class held_document
{
public:
    held_document(entry_map values, std::vector<std::string> instances, std::filesystem::path from, defaults_map fallbacks, identity_map identities);

    std::optional<std::string_view> value_at(std::string_view key) const;

    bool stored(std::string_view key) const;

    std::optional<crossed> crossing(std::string_view key) const;

    // One slot per instance element of the declared collection `collection_path` across every outer
    // instance, in ordinal order, holding its identity value or nothing where it carries none; no
    // slot at all for a path no collection is declared at.
    std::vector<std::optional<std::string>> identities_in(std::string_view collection_path) const;

    const std::filesystem::path &from() const noexcept;

    std::optional<leaf_default> fallback(std::string_view declared) const;

    std::optional<std::string> identity_of(std::string_view collection_path) const;

    const identity_map &keyed() const noexcept;

private:
    entry_map m_values;
    std::vector<std::string> m_instances;
    std::filesystem::path m_from;
    defaults_map m_fallbacks;
    identity_map m_identities;
};

}

#endif
