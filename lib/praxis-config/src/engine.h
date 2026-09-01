#ifndef HPP_GUARD_PRAXIS_CONFIG_ENGINE_H
#define HPP_GUARD_PRAXIS_CONFIG_ENGINE_H

#include "praxis/config/error.h"
#include "praxis/config/declaration.h"

#include "praxis/compat/expected.h"

#include <nucleus/error.h>
#include <nucleus/config.h>
#include <nucleus/config_space.h>

#include <map>
#include <string>
#include <cstdint>
#include <optional>
#include <functional>
#include <filesystem>
#include <string_view>

namespace praxis::config {

error translated(const nucleus::error &reason);

std::optional<bool> as_flag(std::string_view text);

std::optional<double> as_real(std::string_view text);

std::optional<std::int64_t> as_integer(std::string_view text);

expected<nucleus::config_space, error> sealed_space(const declaration &shape);

namespace detail {

struct leaf_default
{
    field_kind kind;
    std::string text;
};

using identity_map = std::map<std::string, std::string, std::less<>>;
using defaults_map = std::map<std::string, leaf_default, std::less<>>;

// What every copy of a document shares: the folded keyspace, the file it was folded from, and the
// two things a read needs that the fold does not carry -- the fallback each declared leaf named,
// and which leaf keys each collection's instances.
class held_document
{
public:
    held_document(nucleus::config folded, std::filesystem::path from, defaults_map fallbacks, identity_map identities);

    const nucleus::config &folded() const noexcept;

    const std::filesystem::path &from() const noexcept;

    std::optional<leaf_default> fallback(std::string_view declared) const;

    std::optional<std::string> identity_of(std::string_view collection_path) const;

    const identity_map &keyed() const noexcept;

private:
    nucleus::config m_folded;
    std::filesystem::path m_from;
    defaults_map m_fallbacks;
    identity_map m_identities;
};

}
}

#endif
