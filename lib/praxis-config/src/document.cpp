#include "engine.h"
#include "key_path.h"

#include "praxis/config/document.h"

#include <string>
#include <memory>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <filesystem>
#include <string_view>

namespace praxis::config {
namespace {

// The ordinal that addresses `collection_path` is carried by the segment at that collection's own
// depth, so a key reaching past it without one is addressed at no instance.
bool addressed_through(std::string_view key, const std::string &collection_path)
{
    const std::vector<std::string_view> parts = segments_of(key);
    const std::size_t depth                   = segments_in(collection_path);
    if(parts.size() < depth)
        return true;
    return !parts[depth - 1].empty() && parts[depth - 1].back() == ']';
}

bool reads_as(field_kind declared, field_kind wanted)
{
    if(wanted == field_kind::text)
        return declared == field_kind::text || declared == field_kind::choice;
    return declared == wanted;
}

std::optional<std::string> as_text(std::string_view text)
{
    return std::string(text);
}

template<typename T>
expected<T, error> read(const detail::held_document &held, std::string_view key, field_kind wanted, std::optional<T> (*from_text)(std::string_view))
{
    const nucleus::expected<T, nucleus::error> found = held.folded().get_as<T>(key);
    if(found)
        return found.value();
    if(found.error().code != nucleus::errc::absent_key)
        return unexpected(translated(found.error()));

    const std::optional<detail::leaf_default> named = held.fallback(declared_path(key));
    if(!named)
        return unexpected(translated(found.error()));
    if(!reads_as(named->kind, wanted))
        return unexpected(error{error_code::mismatched_kind, "the leaf '" + std::string(key) + "' is not declared as that kind"});

    const std::optional<T> value = from_text(named->text);
    if(!value)
        return unexpected(error{error_code::malformed_source, "the fallback declared for '" + std::string(key) + "' is not of its kind"});
    return *value;
}

}

document::document(std::shared_ptr<const detail::held_document> held)
        : m_held(std::move(held))
{
}

expected<bool, error> document::flag(std::string_view key) const
{
    return read<bool>(*m_held, key, field_kind::flag, as_flag);
}

expected<double, error> document::real(std::string_view key) const
{
    return read<double>(*m_held, key, field_kind::real, as_real);
}

expected<std::string, error> document::text(std::string_view key) const
{
    return read<std::string>(*m_held, key, field_kind::text, as_text);
}

expected<std::int64_t, error> document::integer(std::string_view key) const
{
    return read<std::int64_t>(*m_held, key, field_kind::integer, as_integer);
}

bool document::holds(std::string_view key) const
{
    return m_held->folded().contains(key) || m_held->fallback(declared_path(key)).has_value();
}

std::vector<std::string> document::identities(std::string_view collection_path) const
{
    const std::optional<std::string> keyed_by = m_held->identity_of(collection_path);
    if(!keyed_by)
        return {};
    return m_held->folded().get_all(std::string(collection_path) + "/" + *keyed_by);
}

expected<std::string, error> document::key(std::string_view collection_path, std::string_view identity, std::string_view leaf) const
{
    const std::vector<std::string> present = identities(collection_path);
    for(std::size_t ordinal = 0; ordinal < present.size(); ++ordinal)
        if(present[ordinal] == identity)
            return std::string(collection_path) + "[" + std::to_string(ordinal) + "]/" + std::string(leaf);

    return unexpected(error{error_code::unlocatable_key, "the collection '" + std::string(collection_path) + "' carries no instance identified as '" + std::string(identity) + "'"});
}

std::optional<field_kind> document::kind_of(std::string_view key) const
{
    const std::optional<detail::leaf_default> named = m_held->fallback(declared_path(key));
    if(!named)
        return std::nullopt;
    return named->kind;
}

value_origin document::origin_of(std::string_view key) const
{
    if(const nucleus::origin *winner = m_held->folded().provenance_of(key); winner != nullptr)
        return value_origin{origin_kind::source, winner->layer};

    const std::string declared = declared_path(key);
    if(!m_held->fallback(declared))
        return value_origin{origin_kind::undeclared, std::string()};

    for(const std::pair<const std::string, std::string> &collection : m_held->keyed())
        if(hangs_under(declared, collection.first) && !addressed_through(key, collection.first) && !identities(collection.first).empty())
            return value_origin{origin_kind::instance_required, std::string()};

    return value_origin{origin_kind::fallback, std::string()};
}

const std::filesystem::path &document::source() const noexcept
{
    return m_held->from();
}

namespace detail {

held_document::held_document(nucleus::config folded, std::filesystem::path from, defaults_map fallbacks, identity_map identities)
        : m_folded(std::move(folded))
        , m_from(std::move(from))
        , m_fallbacks(std::move(fallbacks))
        , m_identities(std::move(identities))
{
}

const nucleus::config &held_document::folded() const noexcept
{
    return m_folded;
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
