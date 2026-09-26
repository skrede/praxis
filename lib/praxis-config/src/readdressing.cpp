#include "locator.h"
#include "readdressing.h"

#include <span>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <algorithm>

namespace praxis::config {
namespace {

// Whether `walked` passes through the collection `collection` names: every step in front of the
// collection's own equal in name and ordinal, and the collection's own equal in name.
bool passes_through(const std::vector<step> &walked, const std::vector<step> &collection)
{
    if(collection.empty() || walked.size() < collection.size())
        return false;
    return std::equal(collection.begin(), collection.end() - 1, walked.begin()) && walked[collection.size() - 1].name == collection.back().name;
}

// `key` with the segment at `depth` spelled as `now`, every other segment as it was.
std::string respelled(const std::string &key, std::size_t depth, const step &now)
{
    std::size_t begins = 0;
    for(std::size_t skipped = 0; skipped < depth; ++skipped)
        begins = key.find('/', begins) + 1;

    const std::size_t ends = key.find('/', begins);
    return std::string(key).replace(begins, ends == std::string::npos ? std::string::npos : ends - begins, now.name + "[" + std::to_string(now.ordinal) + "]");
}

}

std::optional<std::size_t> ordinal_in(const std::string &key, const std::string &stem)
{
    const std::vector<step> walked     = steps_of(key);
    const std::vector<step> collection = steps_of(stem);
    if(walked.size() <= collection.size() || !passes_through(walked, collection))
        return std::nullopt;

    return walked[collection.size() - 1].ordinal;
}

std::string renumbered(const std::string &key, std::span<const taken> gone)
{
    for(const taken &one : gone)
    {
        const std::optional<std::size_t> named = ordinal_in(key, one.stem);
        if(!named)
            continue;

        const std::vector<step> collection = steps_of(one.stem);
        const auto in_front                = [&collection, named](const taken &other)
        {
            const std::vector<step> theirs = steps_of(other.stem);
            return theirs.size() == collection.size() && passes_through(theirs, collection) && other.ordinal < *named;
        };
        const std::size_t stepped = *named - static_cast<std::size_t>(std::count_if(gone.begin(), gone.end(), in_front));
        return stepped == *named ? key : respelled(key, collection.size() - 1, step{collection.back().name, stepped});
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
