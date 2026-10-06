#include "engine.h"
#include "removal.h"
#include "key_path.h"
#include "read_back.h"
#include "source_text.h"

#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"

#include <span>
#include <cmath>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <filesystem>
#include <string_view>

namespace praxis::config {
namespace {

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

// The first of `keys` the document does not read as the matching entry of `values`, reported by
// naming the key together with what was written and what came back.
std::optional<error> disagreeing(const declaration &shape, const document &reloaded, std::span<const std::string> keys, std::span<const std::string> values)
{
    for(std::size_t which = 0; which < keys.size(); ++which)
    {
        const field_kind kind                    = declared_kind(shape, keys[which]);
        const std::optional<std::string> matched = carried_identity(shape, reloaded, keys[which]);
        const std::optional<std::string> read    = matched ? matched : reading(reloaded, kind, keys[which]);
        if(read && one_value(kind, *read, values[which]))
            continue;

        return error{error_code::rejected_content, "'" + keys[which] + "' was written as '" + values[which] + "' and reads back as '" + read.value_or("nothing of its kind") + "'"};
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

bool one_value(field_kind kind, std::string_view one, std::string_view other)
{
    const auto alike = [](const auto &first, const auto &second) { return first.has_value() && first == second; };
    if(kind == field_kind::flag)
        return alike(as_flag(one), as_flag(other));
    if(kind == field_kind::real)
        return alike(as_real(one), as_real(other));
    if(kind == field_kind::integer)
        return alike(as_integer(one), as_integer(other));
    return one == other;
}

std::vector<edit> as_written(const declaration &shape, std::span<const edit> changes)
{
    std::vector<edit> spelled(changes.begin(), changes.end());
    for(edit &one : spelled)
    {
        if(one.kind != edit_kind::bound || declared_kind(shape, one.key) != field_kind::real)
            continue;
        if(const std::optional<double> read = as_real(one.value); read && *read == 0.0 && std::signbit(*read))
            one.value = "0";
    }
    return spelled;
}

expected<void, error> reads_as_written(const declaration &shape, const std::filesystem::path &candidate, std::span<const std::string> keys, std::span<const std::string> values,
                                       std::span<const edit> gone)
{
    const expected<document, error> reloaded = load(shape, resolve(candidate, candidate.parent_path()));
    if(!reloaded)
        return unexpected(reloaded.error());

    if(const std::optional<error> refused = disagreeing(shape, reloaded.value(), keys, values); refused)
        return unexpected(*refused);
    const std::optional<std::string> staged = slurped(candidate);
    if(!staged)
        return unexpected(error{error_code::unreadable_source, "the configuration staged at " + candidate.string() + " could not be read back"});
    if(const std::string refused = still_carried(shape, *staged, gone); !refused.empty())
        return unexpected(error{error_code::rejected_content, refused});

    return {};
}

}
