#include "engine.h"
#include "read_back.h"

#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"

#include <span>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <filesystem>
#include <string_view>

namespace praxis::config {
namespace {

// Addressing inside a collection is by ordinal, so a written key carries brackets the declaration
// never had; dropping them is what turns it back into a declared one.
std::string declared_path(std::string_view key)
{
    std::string plain;
    plain.reserve(key.size());
    bool inside = false;
    for(const char letter : key)
    {
        if(letter == '[')
            inside = true;
        else if(letter == ']')
            inside = false;
        else if(!inside)
            plain.push_back(letter);
    }
    return plain;
}

// The instance the segment before `key`'s leaf addresses, or nothing where it addresses none.
std::optional<std::size_t> addressed_at(const std::string &key)
{
    const std::string instance = key.substr(0, key.rfind('/'));
    const std::size_t opens    = instance.find('[', instance.rfind('/') + 1);
    if(opens == std::string::npos)
        return std::nullopt;

    std::size_t ordinal = 0;
    for(const char digit : std::string_view(instance).substr(opens + 1))
    {
        if(digit < '0' || digit > '9')
            break;
        ordinal = ordinal * 10 + static_cast<std::size_t>(digit - '0');
    }
    return ordinal;
}

// A collection's identity is matched rather than addressed, so a key naming one reads back as the
// identity the instance at that key's ordinal carries, and as nothing where the key names no
// declared identity or the collection carries no instance there.
std::optional<std::string> carried_identity(const declaration &shape, const document &reloaded, const std::string &key)
{
    const std::string plain = declared_path(key);
    for(const node &declared : shape.nodes())
    {
        if(declared.shape != node_kind::collection || plain != declared.path + "/" + declared.identity)
            continue;

        const std::vector<std::string> present  = reloaded.identities(declared.path);
        const std::optional<std::size_t> at_one = addressed_at(key);
        if(!at_one || *at_one >= present.size())
            return std::nullopt;
        return present[*at_one];
    }
    return std::nullopt;
}

}

field_kind declared_kind(const declaration &shape, const std::string &key)
{
    const std::string plain = declared_path(key);
    for(const node &declared : shape.nodes())
        if(declared.shape == node_kind::leaf && declared.path == plain)
            return declared.kind;
    return field_kind::text;
}

std::optional<std::string> reading(const document &reloaded, field_kind kind, const std::string &key)
{
    if(kind == field_kind::flag)
        return reloaded.flag(key) ? std::optional<std::string>(reloaded.flag(key).value() ? "true" : "false") : std::nullopt;
    if(kind == field_kind::real)
        return reloaded.real(key) ? std::optional<std::string>(exact_text(reloaded.real(key).value())) : std::nullopt;
    if(kind == field_kind::integer)
        return reloaded.integer(key) ? std::optional<std::string>(std::to_string(reloaded.integer(key).value())) : std::nullopt;
    return reloaded.text(key) ? std::optional<std::string>(reloaded.text(key).value()) : std::nullopt;
}

std::optional<std::string> canonical(field_kind kind, const std::string &value)
{
    if(kind == field_kind::flag)
        return as_flag(value) ? std::optional<std::string>(*as_flag(value) ? "true" : "false") : std::nullopt;
    if(kind == field_kind::real)
        return as_real(value) ? std::optional<std::string>(exact_text(*as_real(value))) : std::nullopt;
    if(kind == field_kind::integer)
        return as_integer(value) ? std::optional<std::string>(std::to_string(*as_integer(value))) : std::nullopt;
    return value;
}

expected<void, error> reads_as_written(const declaration &shape, const std::filesystem::path &candidate, std::span<const std::string> keys, std::span<const std::string> values)
{
    const expected<document, error> reloaded = load(shape, resolve(candidate, candidate.parent_path()));
    if(!reloaded)
        return unexpected(reloaded.error());

    for(std::size_t which = 0; which < keys.size(); ++which)
    {
        const field_kind kind                    = declared_kind(shape, keys[which]);
        const std::optional<std::string> matched = carried_identity(shape, reloaded.value(), keys[which]);
        const std::optional<std::string> read    = matched ? matched : reading(reloaded.value(), kind, keys[which]);
        const std::optional<std::string> meant   = canonical(kind, values[which]);
        if(read && meant && *read == *meant)
            continue;
        return unexpected(
                error{error_code::rejected_content, "'" + keys[which] + "' was written as '" + values[which] + "' and reads back as '" + read.value_or("nothing of its kind") + "'"});
    }
    return {};
}

}
