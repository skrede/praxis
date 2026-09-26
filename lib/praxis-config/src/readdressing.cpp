#include "locator.h"
#include "readdressing.h"

#include <span>
#include <string>
#include <cstddef>
#include <optional>
#include <algorithm>
#include <string_view>

namespace praxis::config {

std::optional<std::size_t> ordinal_in(const std::string &key, const std::string &stem)
{
    if(key.size() <= stem.size() || key.compare(0, stem.size(), stem) != 0 || (key[stem.size()] != '[' && key[stem.size()] != '/'))
        return std::nullopt;

    const std::size_t slash  = stem.rfind('/');
    const std::size_t begins = slash == std::string::npos ? 0u : slash + 1u;
    const std::size_t ends   = key.find('/', stem.size());
    return parsed(std::string_view(key).substr(begins, ends == std::string::npos ? std::string::npos : ends - begins)).ordinal;
}

std::string renumbered(const std::string &key, std::span<const taken> gone)
{
    for(const taken &one : gone)
    {
        const std::optional<std::size_t> named = ordinal_in(key, one.stem);
        if(!named)
            continue;

        const auto in_front       = [&one, named](const taken &other) { return other.stem == one.stem && other.ordinal < *named; };
        const std::size_t stepped = *named - static_cast<std::size_t>(std::count_if(gone.begin(), gone.end(), in_front));
        if(stepped == *named)
            return key;

        const std::size_t closes = key.find(']', one.stem.size());
        return std::string(key).replace(one.stem.size() + 1u, closes - one.stem.size() - 1u, std::to_string(stepped));
    }
    return key;
}

std::string moved_collection(std::span<const taken> gone)
{
    for(const taken &one : gone)
        for(const taken &other : gone)
            if(ordinal_in(one.stem, other.stem))
                return "is asked to take an instance out of '" + one.stem + "' and one out of '" + other.stem + "', which it stands under";

    return std::string();
}

std::string written_into_taken(std::span<const edit> changes, std::span<const taken> gone)
{
    for(const edit &one : changes)
        for(const taken &instance : gone)
            if(one.kind == edit_kind::bound && ordinal_in(one.key, instance.stem) == instance.ordinal)
                return "is asked to write '" + one.key + "' into an instance of '" + instance.stem + "' it takes out";

    return std::string();
}

}
